// utils/wordCountUtils.ts
import type { RecordItem } from "@site/data/wordStats";

export interface ProcessedData {
  date: string;
  total: number;
  delta: number;
  commit: string;
  isIncrease: boolean;
}

export function processWordCountData(items: RecordItem[]): ProcessedData[] {
  const sorted = [...items].sort((a, b) =>
    new Date(a.date).getTime() - new Date(b.date).getTime()
  );

  return sorted.map((item, index) => {
    const prev = sorted[index - 1]?.wordCount || 0;

    const localDate = new Date(item.date);
    const formattedDate = localDate.toLocaleString(undefined, {
      year: "numeric",
      month: "2-digit",
      day: "2-digit",
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
      hour12: false
    }).replace(/\//g, '-'); // 可选：将 `/` 替换为 `-`，统一日期格式

    return {
      date: formattedDate,
      total: item.wordCount,
      delta: item.wordCount - prev,
      commit: item.commit,
      isIncrease: item.wordCount - prev >= 0,
    };
  });
}