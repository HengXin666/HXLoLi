/**
 * 跨 Tab 音乐播放状态同步
 *
 * 核心机制:
 * 1. Leader Election: 多个 Tab 中只有一个是 Leader, 负责实际播放音频
 * 2. BroadcastChannel: Tab 间广播消息, 同步播放状态
 * 3. 心跳检测: Leader 定期发送心跳, Follower 检测 Leader 是否存活
 */

/** 播放状态消息类型 */
export type MusicMessageType =
    | 'LEADER_HEARTBEAT'    // Leader 心跳
    | 'LEADER_CLAIM'        // 声明成为 Leader
    | 'LEADER_RESIGN'       // Leader 退位 (Tab 关闭)
    | 'STATE_SYNC'          // 状态同步 (Leader -> Followers)
    | 'COMMAND'             // 操作命令 (任意 Tab -> Leader)
    | 'REQUEST_STATE';      // 请求当前状态 (新 Tab 请求)

/** 播放命令类型 */
export type MusicCommand =
    | { action: 'play' }
    | { action: 'pause' }
    | { action: 'toggle' }
    | { action: 'next' }
    | { action: 'prev' }
    | { action: 'seek'; time: number }
    | { action: 'setTrack'; trackIndex: number }
    | { action: 'setVolume'; volume: number };

/** 播放状态 */
export interface MusicState {
    /** 当前播放的曲目索引 */
    trackIndex: number;
    /** 当前播放时间 (秒) */
    currentTime: number;
    /** 歌曲总时长 (秒) */
    duration: number;
    /** 是否正在播放 */
    isPlaying: boolean;
    /** 音量 0-1 */
    volume: number;
    /** 时间戳, 用于计算实际播放位置 */
    timestamp: number;
}

/** BroadcastChannel 消息 */
export interface MusicMessage {
    type: MusicMessageType;
    tabId: string;
    state?: MusicState;
    command?: MusicCommand;
}

const CHANNEL_NAME = 'hxloli-music-sync';
const HEARTBEAT_INTERVAL = 2000; // Leader 心跳间隔 (ms)
const HEARTBEAT_TIMEOUT = 5000;  // Leader 超时时间 (ms)

/** 生成唯一 Tab ID */
function generateTabId(): string {
    return `tab_${Date.now()}_${Math.random().toString(36).slice(2, 8)}`;
}

/**
 * 跨 Tab 音乐同步管理器
 */
export class CrossTabMusicSync {
    private channel: BroadcastChannel | null = null;
    private tabId: string;
    private _isLeader: boolean = false;
    private heartbeatTimer: ReturnType<typeof setInterval> | null = null;
    private leaderCheckTimer: ReturnType<typeof setInterval> | null = null;
    private lastLeaderHeartbeat: number = 0;
    private leaderId: string | null = null;

    // 回调函数
    private onBecomeLeader: (() => void) | null = null;
    private onLoseLeadership: (() => void) | null = null;
    private onStateUpdate: ((state: MusicState) => void) | null = null;
    private onCommand: ((cmd: MusicCommand) => void) | null = null;

    constructor() {
        this.tabId = generateTabId();
    }

    get isLeader(): boolean {
        return this._isLeader;
    }

    /** 初始化跨 Tab 通信 */
    init(callbacks: {
        onBecomeLeader: () => void;
        onLoseLeadership: () => void;
        onStateUpdate: (state: MusicState) => void;
        onCommand: (cmd: MusicCommand) => void;
    }): void {
        this.onBecomeLeader = callbacks.onBecomeLeader;
        this.onLoseLeadership = callbacks.onLoseLeadership;
        this.onStateUpdate = callbacks.onStateUpdate;
        this.onCommand = callbacks.onCommand;

        // 检查 BroadcastChannel 支持
        if (typeof BroadcastChannel === 'undefined') {
            // 不支持的浏览器直接当 Leader
            this._isLeader = true;
            this.onBecomeLeader?.();
            return;
        }

        this.channel = new BroadcastChannel(CHANNEL_NAME);
        this.channel.onmessage = (event: MessageEvent<MusicMessage>) => {
            this.handleMessage(event.data);
        };

        // 尝试成为 Leader
        this.tryClaimLeader();

        // 监听页面关闭
        window.addEventListener('beforeunload', this.handleUnload);
    }

    /** 处理接收到的消息 */
    private handleMessage(msg: MusicMessage): void {
        if (msg.tabId === this.tabId) return; // 忽略自己的消息

        switch (msg.type) {
            case 'LEADER_CLAIM':
                // 另一个 Tab 声明成为 Leader
                if (this._isLeader) {
                    // 如果自己已经是 Leader, 比较 tabId 决定谁让步
                    // tabId 更小的获胜 (简单确定性策略)
                    if (this.tabId < msg.tabId) {
                        // 自己赢了, 重新广播
                        this.broadcastLeaderClaim();
                    } else {
                        // 自己输了, 退位
                        this._isLeader = false;
                        this.stopHeartbeat();
                        this.leaderId = msg.tabId;
                        this.lastLeaderHeartbeat = Date.now();
                        this.startLeaderCheck();
                        this.onLoseLeadership?.();
                    }
                } else {
                    // 自己不是 Leader, 记录新 Leader
                    this.leaderId = msg.tabId;
                    this.lastLeaderHeartbeat = Date.now();
                    this.stopLeaderCheck();
                    this.startLeaderCheck();
                }
                break;

            case 'LEADER_HEARTBEAT':
                if (msg.tabId === this.leaderId || !this._isLeader) {
                    this.leaderId = msg.tabId;
                    this.lastLeaderHeartbeat = Date.now();
                }
                break;

            case 'LEADER_RESIGN':
                if (msg.tabId === this.leaderId) {
                    // Leader 退出, 尝试接管
                    this.leaderId = null;
                    setTimeout(() => {
                        if (!this.leaderId) {
                            this.tryClaimLeader();
                        }
                    }, Math.random() * 500); // 随机延迟避免冲突
                }
                break;

            case 'STATE_SYNC':
                if (!this._isLeader && msg.state) {
                    this.onStateUpdate?.(msg.state);
                }
                break;

            case 'COMMAND':
                if (this._isLeader && msg.command) {
                    this.onCommand?.(msg.command);
                }
                break;

            case 'REQUEST_STATE':
                // 新 Tab 请求状态, Leader 应该回应
                // (由 Leader 的心跳机制自动同步, 无需额外处理)
                break;
        }
    }

    /** 尝试成为 Leader */
    private tryClaimLeader(): void {
        this._isLeader = true;
        this.broadcastLeaderClaim();
        this.startHeartbeat();
        this.onBecomeLeader?.();

        // 请求当前状态 (如果有其他 Tab 正在播放)
        this.send({ type: 'REQUEST_STATE', tabId: this.tabId });
    }

    /** 广播 Leader 声明 */
    private broadcastLeaderClaim(): void {
        this.send({ type: 'LEADER_CLAIM', tabId: this.tabId });
    }

    /** 开始 Leader 心跳 */
    private startHeartbeat(): void {
        this.stopHeartbeat();
        this.heartbeatTimer = setInterval(() => {
            if (this._isLeader) {
                this.send({ type: 'LEADER_HEARTBEAT', tabId: this.tabId });
            }
        }, HEARTBEAT_INTERVAL);
    }

    private stopHeartbeat(): void {
        if (this.heartbeatTimer) {
            clearInterval(this.heartbeatTimer);
            this.heartbeatTimer = null;
        }
    }

    /** 开始检测 Leader 存活 */
    private startLeaderCheck(): void {
        this.stopLeaderCheck();
        this.leaderCheckTimer = setInterval(() => {
            if (!this._isLeader && this.leaderId) {
                const elapsed = Date.now() - this.lastLeaderHeartbeat;
                if (elapsed > HEARTBEAT_TIMEOUT) {
                    // Leader 超时, 尝试接管
                    this.leaderId = null;
                    this.tryClaimLeader();
                }
            }
        }, HEARTBEAT_TIMEOUT);
    }

    private stopLeaderCheck(): void {
        if (this.leaderCheckTimer) {
            clearInterval(this.leaderCheckTimer);
            this.leaderCheckTimer = null;
        }
    }

    /** 发送消息 */
    private send(msg: MusicMessage): void {
        try {
            this.channel?.postMessage(msg);
        } catch {
            // Channel 可能已关闭
        }
    }

    /** 发送操作命令 (任意 Tab 都可以调用) */
    sendCommand(cmd: MusicCommand): void {
        if (this._isLeader) {
            // 自己就是 Leader, 直接执行
            this.onCommand?.(cmd);
        } else {
            this.send({ type: 'COMMAND', tabId: this.tabId, command: cmd });
        }
    }

    /** Leader 广播状态更新 */
    broadcastState(state: MusicState): void {
        if (this._isLeader) {
            this.send({ type: 'STATE_SYNC', tabId: this.tabId, state });
        }
    }

    /** 页面关闭时处理 */
    private handleUnload = (): void => {
        if (this._isLeader) {
            // 同步发送退位消息
            try {
                this.channel?.postMessage({
                    type: 'LEADER_RESIGN',
                    tabId: this.tabId,
                } as MusicMessage);
            } catch {
                // ignore
            }
        }
    };

    /** 销毁 */
    dispose(): void {
        window.removeEventListener('beforeunload', this.handleUnload);
        this.handleUnload();
        this.stopHeartbeat();
        this.stopLeaderCheck();
        this.channel?.close();
        this.channel = null;
    }
}
