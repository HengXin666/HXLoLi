// 格式化为 mm:ss, 如 01:02
export function formatMmSs(durationSeconds: number): string {
    const minutes: number = Math.floor(durationSeconds / 60);
    const seconds: number = durationSeconds % 60;
    return `${minutes.toString().padStart(2, "0")}:${seconds
        .toString()
        .padStart(2, "0")}`;
}