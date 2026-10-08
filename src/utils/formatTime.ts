// 格式化为 mm:ss, 如 01:02
/**
 * HXLoLi 接入 Agent Notes v2
 * .agents/notes/implemented/process/2026-10-08-repository-agent-notes-v2-adoption.md
 */
export function formatMmSs(durationSeconds: number): string {
    const minutes: number = Math.floor(durationSeconds / 60);
    const seconds: number = durationSeconds % 60;
    return `${minutes.toString().padStart(2, "0")}:${seconds
        .toString()
        .padStart(2, "0")}`;
}