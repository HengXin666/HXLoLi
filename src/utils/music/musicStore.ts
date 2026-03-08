/**
 * 音乐播放器全局状态管理 (基于 Zustand)
 *
 * 负责:
 * 1. 管理 HTMLAudioElement 播放
 * 2. 跨 Tab 同步 (Leader/Follower 模式)
 *    - 音频播放固定在 Leader Tab（仅 Leader 关闭时轮换）
 *    - Follower 通过时间插值实现流畅的 ASS 歌词渲染
 * 3. 状态持久化 (sessionStorage 恢复播放)
 */
import type { MusicTrack } from '@site/src/config/musicData';
import { create } from 'zustand';
import { CrossTabMusicSync, type MusicCommand, type MusicState } from './crossTabSync';
import { loadPlaylist } from './musicDataLoader';

/** 播放模式 */
export type PlayMode = 'list-loop' | 'single-loop' | 'shuffle';

/** 播放器 UI 状态 */
interface MusicPlayerState {
    /** 播放列表 */
    playlist: MusicTrack[];
    /** 当前曲目索引 */
    trackIndex: number;
    /** 当前播放时间 */
    currentTime: number;
    /** 歌曲总时长 */
    duration: number;
    /** 是否正在播放 */
    isPlaying: boolean;
    /** 音量 0-1 */
    volume: number;
    /** 播放模式 */
    playMode: PlayMode;
    /** 当前 Tab 是否为 Leader (负责实际音频播放) */
    isLeader: boolean;
    /** 播放器是否已初始化 */
    initialized: boolean;
    /** 歌词悬浮窗是否显示 */
    showLyrics: boolean;
    /** 歌词是否全屏 */
    lyricsFullscreen: boolean;
    /** 播放器面板是否展开 */
    panelExpanded: boolean;
    /**
     * 最近一次状态同步的时间戳 (ms)
     * Follower 用它来插值计算当前真实播放位置，避免 ASS 歌词一卡一卡
     */
    lastSyncTimestamp: number;
}

interface MusicPlayerActions {
    /** 初始化播放器 */
    init: () => void;
    /** 播放 */
    play: () => void;
    /** 暂停 */
    pause: () => void;
    /** 播放/暂停切换 */
    toggle: () => void;
    /** 下一曲 */
    next: () => void;
    /** 上一曲 */
    prev: () => void;
    /** 跳转到指定时间 */
    seek: (time: number) => void;
    /** 设置曲目 */
    setTrack: (index: number) => void;
    /** 设置音量 */
    setVolume: (vol: number) => void;
    /** 切换歌词显示 */
    toggleLyrics: () => void;
    /** 切换歌词全屏 */
    toggleLyricsFullscreen: () => void;
    /** 切换播放模式 */
    cyclePlayMode: () => void;
    /** 切换播放器面板 */
    togglePanel: () => void;
    /** 关闭面板 */
    closePanel: () => void;
    /** 销毁播放器 */
    dispose: () => void;
    /**
     * 获取插值后的当前播放时间 (秒)
     * - Leader: 直接返回 audio.currentTime (精确)
     * - Follower: 基于上次同步时间 + 本地流逝时间推算 (流畅)
     */
    getInterpolatedTime: () => number;
}

type MusicStore = MusicPlayerState & MusicPlayerActions;

// ---- 单例: 音频元素 & 跨 Tab 同步 ----
let audio: HTMLAudioElement | null = null;
let crossTab: CrossTabMusicSync | null = null;
let rafId: number | null = null;
let broadcastWorker: Worker | null = null;

/**
 * Follower 端播放状态（用于 Follower RAF 循环判断）
 */
let followerIsPlaying = false;

const STORAGE_KEY = 'hxloli-music-state';
// 跨 Tab 共享的播放状态 key（存储在 localStorage 中，所有 Tab 可见）
const SHARED_STATE_KEY = 'hxloli-music-shared-state';

// Leader 状态广播间隔 (ms)，越小 Follower 插值越准，但增加 IPC 开销
const STATE_BROADCAST_INTERVAL = 200;

/** 保存状态到 sessionStorage */
function saveState(state: MusicPlayerState): void {
    try {
        const data: MusicState = {
            trackIndex: state.trackIndex,
            currentTime: state.currentTime,
            duration: state.duration,
            isPlaying: state.isPlaying,
            volume: state.volume,
            showLyrics: state.showLyrics,
            timestamp: Date.now(),
        };
        sessionStorage.setItem(STORAGE_KEY, JSON.stringify(data));
    } catch {
        // sessionStorage 可能不可用
    }
}

/** 从 sessionStorage 恢复状态 */
function loadState(): MusicState | null {
    try {
        const raw = sessionStorage.getItem(STORAGE_KEY);
        if (!raw) return null;
        return JSON.parse(raw) as MusicState;
    } catch {
        return null;
    }
}

/** 保存共享播放状态到 localStorage（所有 Tab 可见） */
function saveSharedState(state: MusicPlayerState): void {
    try {
        const data: MusicState = {
            trackIndex: state.trackIndex,
            currentTime: state.currentTime,
            duration: state.duration,
            isPlaying: state.isPlaying,
            volume: state.volume,
            showLyrics: state.showLyrics,
            timestamp: Date.now(),
        };
        localStorage.setItem(SHARED_STATE_KEY, JSON.stringify(data));
    } catch { /* ignore */ }
}

/** 从 localStorage 读取共享播放状态 */
function loadSharedState(): MusicState | null {
    try {
        const raw = localStorage.getItem(SHARED_STATE_KEY);
        if (!raw) return null;
        return JSON.parse(raw) as MusicState;
    } catch {
        return null;
    }
}

/** 从 localStorage 读取持久化的音量 */
function loadVolume(): number {
    try {
        const v = localStorage.getItem('hxloli-music-volume');
        if (v !== null) return parseFloat(v);
    } catch { /* ignore */ }
    return 0.7;
}

/** 持久化音量到 localStorage */
function saveVolume(vol: number): void {
    try {
        localStorage.setItem('hxloli-music-volume', String(vol));
    } catch { /* ignore */ }
}

/** 从 localStorage 读取播放模式 */
function loadPlayMode(): PlayMode {
    try {
        const m = localStorage.getItem('hxloli-music-playmode');
        if (m === 'list-loop' || m === 'single-loop' || m === 'shuffle') return m;
    } catch { /* ignore */ }
    return 'list-loop';
}

/** 持久化播放模式到 localStorage */
function savePlayMode(mode: PlayMode): void {
    try {
        localStorage.setItem('hxloli-music-playmode', mode);
    } catch { /* ignore */ }
}

/** 获取下一曲索引 (根据播放模式) */
function getNextIndex(current: number, total: number, mode: PlayMode): number {
    if (total === 0) return 0;
    switch (mode) {
        case 'single-loop':
            return current;
        case 'shuffle': {
            if (total <= 1) return 0;
            let next = current;
            while (next === current) {
                next = Math.floor(Math.random() * total);
            }
            return next;
        }
        case 'list-loop':
        default:
            return (current + 1) % total;
    }
}

/** 获取上一曲索引 (根据播放模式) */
function getPrevIndex(current: number, total: number, mode: PlayMode): number {
    if (total === 0) return 0;
    switch (mode) {
        case 'single-loop':
            return current;
        case 'shuffle': {
            if (total <= 1) return 0;
            let prev = current;
            while (prev === current) {
                prev = Math.floor(Math.random() * total);
            }
            return prev;
        }
        case 'list-loop':
        default:
            return (current - 1 + total) % total;
    }
}

export const useMusicStore = create<MusicStore>((set, get) => {
    /** 更新时间 RAF 循环 (仅 Leader) */
    function startTimeUpdate(): void {
        stopTimeUpdate();
        const tick = () => {
            if (audio && !audio.paused) {
                const currentTime = audio.currentTime;
                const duration = audio.duration || 0;
                set({ currentTime, duration });
            }
            rafId = requestAnimationFrame(tick);
        };
        rafId = requestAnimationFrame(tick);
    }

    function stopTimeUpdate(): void {
        if (rafId !== null) {
            cancelAnimationFrame(rafId);
            rafId = null;
        }
    }

    // Follower 不需要 RAF 循环写 currentTime
    // currentTime 由 onStateUpdate 直接从 Leader 广播写入
    // ASS 歌词渲染通过 getInterpolatedTime() 在读取时做帧级插值

    /**
     * Leader 定期广播状态给 Follower
     *
     * 关键: 使用内联 Web Worker 做定时器，因为:
     * - setInterval 在后台 Tab 会被浏览器节流到 >=1000ms
     * - requestAnimationFrame 在后台 Tab 直接暂停
     * - Web Worker 的 setInterval 不受 Tab 可见性影响
     */
    function startStateBroadcast(): void {
        stopStateBroadcast();
        // 用于记录上次保存的状态，避免无变化时重复写入 storage
        let lastSavedTime = -1;
        let lastSavedTrack = -1;
        let lastSavedPlaying = false;

        const broadcastTick = () => {
            const s = get();
            if (crossTab?.isLeader) {
                // 直接从 audio 元素读取 currentTime，而不是依赖 RAF 更新的 store 值
                // 因为 RAF 在后台 Tab 也会暂停，store.currentTime 可能是过时的
                const realTime = audio ? audio.currentTime : s.currentTime;
                const realDuration = audio ? (audio.duration || 0) : s.duration;
                // 同时更新 store（补偿 RAF 暂停导致的 store 滞后）
                if (audio && Math.abs(realTime - s.currentTime) > 0.05) {
                    set({ currentTime: realTime, duration: realDuration });
                }
                crossTab.broadcastState({
                    trackIndex: s.trackIndex,
                    currentTime: realTime,
                    duration: realDuration,
                    isPlaying: s.isPlaying,
                    volume: s.volume,
                    showLyrics: s.showLyrics,
                    timestamp: Date.now(),
                });
            }
            // 只在播放状态变化或时间有明显变化时才保存到 storage
            const timeChanged = Math.abs(s.currentTime - lastSavedTime) > 1;
            const trackChanged = s.trackIndex !== lastSavedTrack;
            const playingChanged = s.isPlaying !== lastSavedPlaying;
            if (timeChanged || trackChanged || playingChanged) {
                saveState(s);
                saveSharedState(s);
                lastSavedTime = s.currentTime;
                lastSavedTrack = s.trackIndex;
                lastSavedPlaying = s.isPlaying;
            }
        };

        // 创建内联 Worker: 仅负责定时发 tick 消息
        try {
            const workerCode = `setInterval(()=>postMessage('tick'),${STATE_BROADCAST_INTERVAL})`;
            const blob = new Blob([workerCode], { type: 'application/javascript' });
            const url = URL.createObjectURL(blob);
            broadcastWorker = new Worker(url);
            URL.revokeObjectURL(url);
            broadcastWorker.onmessage = () => broadcastTick();
        } catch {
            // Worker 创建失败（极罕见），降级回 setInterval
            console.warn('[MusicPlayer] Worker 创建失败，降级使用 setInterval');
            const fallbackId = setInterval(broadcastTick, STATE_BROADCAST_INTERVAL);
            // 用一个假 Worker 对象方便 stopStateBroadcast 清理
            broadcastWorker = { terminate: () => clearInterval(fallbackId) } as any;
        }
    }

    function stopStateBroadcast(): void {
        if (broadcastWorker) {
            broadcastWorker.terminate();
            broadcastWorker = null;
        }
    }

    /** 加载并播放指定曲目 (仅 Leader 调用) */
    function loadTrack(index: number, autoPlay: boolean = false, seekTo?: number): void {
        if (!audio) return;
        const list = get().playlist;
        if (index < 0 || index >= list.length) return;
        const track = list[index];

        // 先暂停当前播放，避免 play() 的 AbortError
        audio.pause();

        // 如果是 autoPlay，保持 isPlaying: true 避免 UI 闪烁为暂停状态
        set({ trackIndex: index, currentTime: seekTo ?? 0, duration: 0, isPlaying: autoPlay });

        audio.src = track.audioUrl;

        const onCanPlay = () => {
            if (!audio) return;
            audio.removeEventListener('canplay', onCanPlay);
            audio.removeEventListener('error', onError);

            set({ duration: audio.duration || 0 });

            if (seekTo !== undefined && seekTo > 0) {
                audio.currentTime = seekTo;
            }

            if (autoPlay) {
                audio.volume = get().volume;
                const playPromise = audio.play();
                if (playPromise) {
                    playPromise.then(() => {
                        set({ isPlaying: true });
                        flushSharedState();
                    }).catch(() => {
                        set({ isPlaying: false });
                        flushSharedState();
                    });
                }
            }
        };

        const onError = () => {
            if (!audio) return;
            audio.removeEventListener('canplay', onCanPlay);
            audio.removeEventListener('error', onError);
            console.error('[MusicPlayer] 加载音频失败:', track.audioUrl);
            set({ isPlaying: false });
        };

        audio.addEventListener('canplay', onCanPlay);
        audio.addEventListener('error', onError);
    }

    /** 关键操作后立即同步共享状态（不等 interval） */
    function flushSharedState(): void {
        saveSharedState(get());
    }

    /** 执行命令 (Leader 收到命令后执行) */
    function executeCommand(cmd: MusicCommand): void {
        if (!audio) return;
        const s = get();

        switch (cmd.action) {
            case 'play':
                if (s.playlist.length === 0) return;
                {
                    const p = audio.play();
                    if (p) {
                        p.then(() => { set({ isPlaying: true }); flushSharedState(); })
                         .catch(() => { set({ isPlaying: false }); flushSharedState(); });
                    }
                }
                break;
            case 'pause':
                audio.pause();
                set({ isPlaying: false });
                flushSharedState();
                break;
            case 'toggle':
                if (s.playlist.length === 0) return;
                if (audio.paused) {
                    const p = audio.play();
                    if (p) {
                        p.then(() => { set({ isPlaying: true }); flushSharedState(); })
                         .catch(() => { set({ isPlaying: false }); flushSharedState(); });
                    }
                } else {
                    audio.pause();
                    set({ isPlaying: false });
                    flushSharedState();
                }
                break;
            case 'next': {
                if (s.playlist.length === 0) return;
                const next = getNextIndex(s.trackIndex, s.playlist.length, s.playMode);
                loadTrack(next, true);
                break;
            }
            case 'prev': {
                if (s.playlist.length === 0) return;
                const prev = getPrevIndex(s.trackIndex, s.playlist.length, s.playMode);
                loadTrack(prev, true);
                break;
            }
            case 'seek':
                audio.currentTime = cmd.time;
                set({ currentTime: cmd.time });
                flushSharedState();
                break;
            case 'setTrack':
                loadTrack(cmd.trackIndex, true);
                break;
            case 'setVolume':
                audio.volume = cmd.volume;
                set({ volume: cmd.volume });
                saveVolume(cmd.volume);
                break;
        }
    }

    return {
        // ---- 状态 ----
        playlist: [],
        trackIndex: 0,
        currentTime: 0,
        duration: 0,
        isPlaying: false,
        volume: loadVolume(),
        playMode: loadPlayMode(),
        isLeader: false,
        initialized: false,
        showLyrics: false,
        lyricsFullscreen: false,
        panelExpanded: false,
        lastSyncTimestamp: 0,

        // ---- 动作 ----
        init: () => {
            if (get().initialized) return;
            set({ initialized: true });

            // 仅在浏览器环境中运行
            if (typeof window === 'undefined') return;

            // 异步加载播放列表 (从 HXLoLi-Music 仓库 CDN)
            loadPlaylist().then(playlistData => {
                set({ playlist: playlistData });
                // 如果已经有 Leader 且 playlist 之前为空, 触发恢复
                const s = get();
                if (s.isLeader && playlistData.length > 0 && audio && !audio.src) {
                    const shared = loadSharedState();
                    const saved = loadState();
                    const restoreTrack = shared?.trackIndex ?? saved?.trackIndex ?? 0;
                    const restoreTime = shared?.currentTime ?? saved?.currentTime ?? 0;
                    const restorePlaying = shared?.isPlaying ?? saved?.isPlaying ?? false;
                    if (restoreTrack < playlistData.length) {
                        loadTrack(restoreTrack, restorePlaying, restoreTime);
                    } else {
                        loadTrack(0, false);
                    }
                    // CDN 慢时 onBecomeLeader 中 list.length===0 会跳过 showLyrics 恢复
                    // 在这里补充恢复, 确保 showLyrics 状态不丢失
                    const lyricsState = shared?.showLyrics ?? saved?.showLyrics;
                    if (lyricsState !== undefined) {
                        set({ showLyrics: lyricsState });
                    }
                }
            }).catch(err => {
                console.error('[MusicPlayer] 加载播放列表失败:', err);
            });

            // 尝试恢复 showLyrics 状态 (无论 Leader/Follower 都需要)
            const savedInit = loadState();
            if (savedInit?.showLyrics !== undefined) {
                set({ showLyrics: savedInit.showLyrics });
            }

            // 从 localStorage 共享状态预初始化 (所有 Tab 可见)
            // 这样在第一次 STATE_SYNC 到达之前，UI 就能显示合理的状态
            const preShared = loadSharedState();
            if (preShared) {
                let preTime = preShared.currentTime;
                // 如果之前在播放，补偿时间差
                if (preShared.isPlaying && preShared.timestamp) {
                    const elapsed = (Date.now() - preShared.timestamp) / 1000;
                    preTime += Math.max(0, elapsed);
                    if (preShared.duration > 0) preTime = Math.min(preTime, preShared.duration);
                }
                followerIsPlaying = preShared.isPlaying;
                // 同步 UI 状态
                set({
                    trackIndex: preShared.trackIndex,
                    currentTime: preTime,
                    duration: preShared.duration,
                    isPlaying: preShared.isPlaying,
                    volume: preShared.volume,
                    lastSyncTimestamp: performance.now(),
                });
            }

            // 初始化跨 Tab 同步

            crossTab = new CrossTabMusicSync();
            crossTab.init({
                onBecomeLeader: () => {
                    // 成为 Leader: 创建 audio 元素并恢复状态
                    set({ isLeader: true });
                    if (!audio) {
                        audio = new Audio();
                        audio.volume = get().volume;

                        // 监听播放结束 -> 根据播放模式决定下一曲
                        audio.addEventListener('ended', (e) => {
                            const el = e.target as HTMLAudioElement;
                            if (!el || audio !== el) return;
                            const s = get();
                            if (s.playlist.length > 0) {
                                if (s.playMode === 'single-loop') {
                                    el.currentTime = 0;
                                    const p = el.play();
                                    if (p) {
                                        p.then(() => set({ isPlaying: true }))
                                         .catch(() => set({ isPlaying: false }));
                                    }
                                } else {
                                    const next = getNextIndex(s.trackIndex, s.playlist.length, s.playMode);
                                    loadTrack(next, true);
                                }
                            }
                        });

                        // 监听 duration 变化
                        audio.addEventListener('loadedmetadata', (e) => {
                            const el = e.target as HTMLAudioElement;
                            if (!el || audio !== el) return;
                            set({ duration: el.duration || 0 });
                        });
                    }

                    // 恢复播放状态
                    // 优先级: localStorage 共享状态 > sessionStorage 本地状态
                    const shared = loadSharedState();
                    const saved = loadState();
                    const list = get().playlist;

                    const hasSharedState = shared !== null && (shared.currentTime > 0 || shared.trackIndex > 0);

                    let restoreTrack: number;
                    let restoreTime: number;
                    let restorePlaying: boolean;
                    let restoreVolume: number;

                    if (hasSharedState) {
                        restoreTrack = shared!.trackIndex;
                        restoreTime = shared!.currentTime;
                        restorePlaying = shared!.isPlaying;
                        restoreVolume = shared!.volume;
                        // 补偿时间差：如果之前在播放，根据时间戳推算当前位置
                        if (restorePlaying && shared!.timestamp) {
                            const elapsed = (Date.now() - shared!.timestamp) / 1000;
                            restoreTime = Math.min(restoreTime + elapsed, shared!.duration || Infinity);
                        }
                    } else if (saved) {
                        restoreTrack = saved.trackIndex;
                        restoreTime = saved.currentTime;
                        restorePlaying = saved.isPlaying;
                        restoreVolume = saved.volume;
                    } else {
                        restoreTrack = 0;
                        restoreTime = 0;
                        restorePlaying = false;
                        restoreVolume = get().volume;
                    }

                    if (list.length > 0 && restoreTrack < list.length) {
                        loadTrack(restoreTrack, restorePlaying, restoreTime);
                        set({ volume: restoreVolume });
                        if (audio) audio.volume = restoreVolume;
                        // 恢复歌词悬浮窗状态
                        const lyricsState = shared?.showLyrics ?? saved?.showLyrics;
                        if (lyricsState !== undefined) {
                            set({ showLyrics: lyricsState });
                        }
                    } else if (list.length > 0) {
                        loadTrack(0, false);
                    }

                    startTimeUpdate();
                    startStateBroadcast();
                },
                onLoseLeadership: () => {
                    // 失去 Leader（仅在 Leader 选举冲突时发生，正常情况下不会触发）
                    // 写入当前真实状态供新 Leader 读取
                    const currentState = get();
                    const realPlaying = audio ? !audio.paused : currentState.isPlaying;
                    const realTime = audio ? audio.currentTime : currentState.currentTime;
                    saveSharedState({
                        ...currentState,
                        isPlaying: realPlaying,
                        currentTime: realTime,
                    } as MusicPlayerState);
                    set({ isLeader: false });
                    stopTimeUpdate();
                    stopStateBroadcast();

                    // 直接销毁 audio
                    if (audio) {
                        try {
                            audio.pause();
                            audio.src = '';
                        } catch { /* ignore */ }
                        audio = null;
                    }

                    // 降级为 Follower
                    followerIsPlaying = realPlaying;
                    set({ lastSyncTimestamp: performance.now() });
                },
                onStateUpdate: (state: MusicState) => {
                    // Follower 收到 Leader 的状态同步
                    // 核心原则: 直接使用 Leader 广播的 currentTime 作为真值
                    // ASS 歌词渲染通过 getInterpolatedTime() 在两次同步间做线性插值
                    followerIsPlaying = state.isPlaying;

                    const update: Partial<MusicPlayerState> = {
                        trackIndex: state.trackIndex,
                        currentTime: state.currentTime,
                        duration: state.duration,
                        isPlaying: state.isPlaying,
                        volume: state.volume,
                        lastSyncTimestamp: performance.now(),
                    };
                    if (state.showLyrics !== undefined) {
                        update.showLyrics = state.showLyrics;
                    }
                    set(update);
                    // 也保存到 sessionStorage, 以便这个 Tab 变成 Leader 时恢复
                    saveState(get());
                },
                onCommand: (cmd: MusicCommand) => {
                    executeCommand(cmd);
                },
            });
        },

        /**
         * 获取插值后的当前播放时间 (秒)
         * - Leader: 直接返回 store 中的 currentTime (由 RAF 循环从 audio.currentTime 更新)
         * - Follower: 基于上次同步的 currentTime + (now - lastSyncTimestamp) 推算
         *   这样 ASS 歌词渲染在 Follower 上也是流畅的，不受 BroadcastChannel 同步间隔限制
         */
        getInterpolatedTime: () => {
            const s = get();
            if (s.isLeader) {
                // Leader 有本地 audio，currentTime 由 RAF 循环实时更新
                return s.currentTime;
            }
            // Follower: 在两次 STATE_SYNC 之间做线性插值
            // currentTime 是上次收到的 Leader 真值
            // lastSyncTimestamp 是收到时的 performance.now()
            // 如果正在播放，推算 = currentTime + (now - lastSyncTimestamp) / 1000
            if (followerIsPlaying && s.lastSyncTimestamp > 0) {
                const elapsed = (performance.now() - s.lastSyncTimestamp) / 1000;
                const interpolated = s.currentTime + elapsed;
                return s.duration > 0 ? Math.min(interpolated, s.duration) : interpolated;
            }
            return s.currentTime;
        },

        play: () => crossTab?.sendCommand({ action: 'play' }),
        pause: () => crossTab?.sendCommand({ action: 'pause' }),
        toggle: () => crossTab?.sendCommand({ action: 'toggle' }),
        next: () => crossTab?.sendCommand({ action: 'next' }),
        prev: () => crossTab?.sendCommand({ action: 'prev' }),
        seek: (time: number) => crossTab?.sendCommand({ action: 'seek', time }),
        setTrack: (index: number) => crossTab?.sendCommand({ action: 'setTrack', trackIndex: index }),
        setVolume: (vol: number) => crossTab?.sendCommand({ action: 'setVolume', volume: vol }),

        cyclePlayMode: () => {
            const modes: PlayMode[] = ['list-loop', 'single-loop', 'shuffle'];
            const current = get().playMode;
            const idx = modes.indexOf(current);
            const next = modes[(idx + 1) % modes.length];
            set({ playMode: next });
            savePlayMode(next);
        },
        toggleLyrics: () => {
            set((s) => ({ showLyrics: !s.showLyrics }));
            saveState(get());
            // 同步到 localStorage, 让 Leader 广播时也使用新状态
            saveSharedState(get());
        },
        toggleLyricsFullscreen: () => {
            set((s) => ({ lyricsFullscreen: !s.lyricsFullscreen }));
        },
        togglePanel: () => set((s) => ({ panelExpanded: !s.panelExpanded })),
        closePanel: () => set({ panelExpanded: false }),

        dispose: () => {
            stopTimeUpdate();
            stopStateBroadcast();
            crossTab?.dispose();
            crossTab = null;
            if (audio) {
                audio.pause();
                audio.src = '';
                audio = null;
            }
        },
    };
});
