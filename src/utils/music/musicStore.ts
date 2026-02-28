/**
 * 音乐播放器全局状态管理 (基于 Zustand)
 *
 * 负责:
 * 1. 管理 HTMLAudioElement 播放
 * 2. 跨 Tab 同步 (Leader/Follower 模式)
 * 3. 状态持久化 (sessionStorage 恢复播放)
 */
import { playlist, type MusicTrack } from '@site/src/config/musicData';
import { create } from 'zustand';
import { CrossTabMusicSync, type MusicCommand, type MusicState } from './crossTabSync';

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
}

type MusicStore = MusicPlayerState & MusicPlayerActions;

// ---- 单例: 音频元素 & 跨 Tab 同步 ----
let audio: HTMLAudioElement | null = null;
let crossTab: CrossTabMusicSync | null = null;
let rafId: number | null = null;
let stateInterval: ReturnType<typeof setInterval> | null = null;

const STORAGE_KEY = 'hxloli-music-state';

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

    /** Leader 定期广播状态给 Follower */
    function startStateBroadcast(): void {
        stopStateBroadcast();
        stateInterval = setInterval(() => {
            const s = get();
            if (crossTab?.isLeader) {
                crossTab.broadcastState({
                    trackIndex: s.trackIndex,
                    currentTime: s.currentTime,
                    duration: s.duration,
                    isPlaying: s.isPlaying,
                    volume: s.volume,
                    showLyrics: s.showLyrics,
                    timestamp: Date.now(),
                });
            }
            // 定期保存状态
            saveState(s);
        }, 500);
    }

    function stopStateBroadcast(): void {
        if (stateInterval) {
            clearInterval(stateInterval);
            stateInterval = null;
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

        set({ trackIndex: index, currentTime: seekTo ?? 0, duration: 0, isPlaying: false });

        // 设置 src 会自动触发加载，不需要手动调 load()
        audio.src = track.audioUrl;

        // 等待媒体元数据加载完成后再 seek 和 play
        const onReady = () => {
            audio!.removeEventListener('loadedmetadata', onReady);
            audio!.removeEventListener('error', onError);

            set({ duration: audio!.duration || 0 });

            if (seekTo !== undefined && seekTo > 0) {
                audio!.currentTime = seekTo;
            }

            if (autoPlay) {
                const playPromise = audio!.play();
                if (playPromise) {
                    playPromise.then(() => {
                        set({ isPlaying: true });
                    }).catch(() => {
                        // 浏览器自动播放限制
                        set({ isPlaying: false });
                    });
                }
            }
        };

        const onError = () => {
            audio!.removeEventListener('loadedmetadata', onReady);
            audio!.removeEventListener('error', onError);
            console.error('[MusicPlayer] 加载音频失败:', track.audioUrl);
            set({ isPlaying: false });
        };

        audio.addEventListener('loadedmetadata', onReady);
        audio.addEventListener('error', onError);
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
                        p.then(() => set({ isPlaying: true }))
                         .catch(() => set({ isPlaying: false }));
                    }
                }
                break;
            case 'pause':
                audio.pause();
                set({ isPlaying: false });
                break;
            case 'toggle':
                if (s.playlist.length === 0) return;
                if (audio.paused) {
                    const p = audio.play();
                    if (p) {
                        p.then(() => set({ isPlaying: true }))
                         .catch(() => set({ isPlaying: false }));
                    }
                } else {
                    audio.pause();
                    set({ isPlaying: false });
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
        playlist,
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

        // ---- 动作 ----
        init: () => {
            if (get().initialized) return;
            set({ initialized: true });

            // 仅在浏览器环境中运行
            if (typeof window === 'undefined') return;

            // 尝试恢复 showLyrics 状态 (无论 Leader/Follower 都需要)
            const savedInit = loadState();
            if (savedInit?.showLyrics !== undefined) {
                set({ showLyrics: savedInit.showLyrics });
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
                        audio.addEventListener('ended', () => {
                            const s = get();
                            if (s.playlist.length > 0) {
                                if (s.playMode === 'single-loop') {
                                    // 单曲循环：重新播放当前曲目
                                    audio!.currentTime = 0;
                                    const p = audio!.play();
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
                        audio.addEventListener('loadedmetadata', () => {
                            set({ duration: audio!.duration || 0 });
                        });
                    }

                    // 尝试恢复之前的播放状态
                    const saved = loadState();
                    const list = get().playlist;
                    if (saved && list.length > 0 && saved.trackIndex < list.length) {
                        loadTrack(saved.trackIndex, saved.isPlaying, saved.currentTime);
                        set({ volume: saved.volume });
                        audio!.volume = saved.volume;
                        // 恢复歌词悬浮窗状态
                        if (saved.showLyrics !== undefined) {
                            set({ showLyrics: saved.showLyrics });
                        }
                    } else if (list.length > 0) {
                        // 没有保存状态时，预加载第一首歌（不自动播放）
                        loadTrack(0, false);
                    }

                    startTimeUpdate();
                    startStateBroadcast();
                },
                onLoseLeadership: () => {
                    // 失去 Leader: 销毁 audio
                    set({ isLeader: false });
                    stopTimeUpdate();
                    stopStateBroadcast();
                    if (audio) {
                        audio.pause();
                        audio.src = '';
                        audio = null;
                    }
                },
                onStateUpdate: (state: MusicState) => {
                    // Follower 收到状态同步
                    const update: any = {
                        trackIndex: state.trackIndex,
                        currentTime: state.currentTime,
                        duration: state.duration,
                        isPlaying: state.isPlaying,
                        volume: state.volume,
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
        toggleLyrics: () => set((s) => ({ showLyrics: !s.showLyrics })),
        toggleLyricsFullscreen: () => set((s) => ({ lyricsFullscreen: !s.lyricsFullscreen })),
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
