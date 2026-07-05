// utils/wordCountUtils.ts
import type { RecordItem } from "@site/data/wordStats";

export interface ProcessedData {
  date: string;
  total: number;
  docsBlogTotal: number;
  aiDocsTotal: number;
  delta: number;
  docsBlogDelta: number;
  aiDocsDelta: number;
  commit: string;
  message: string;
  isIncrease: boolean;
}

export function processWordCountData(items: RecordItem[]): ProcessedData[] {
  const sorted = [...items].sort((a, b) =>
    new Date(a.date).getTime() - new Date(b.date).getTime()
  );

  return sorted.map((item, index) => {
    const docsBlogTotal = item.docsBlogWordCount ?? item.wordCount;
    const aiDocsTotal = item.aiDocsWordCount ?? 0;
    const total = item.wordCount ?? docsBlogTotal + aiDocsTotal;

    const prevItem = sorted[index - 1];
    const prevDocsBlogTotal = prevItem?.docsBlogWordCount ?? prevItem?.wordCount ?? 0;
    const prevAiDocsTotal = prevItem?.aiDocsWordCount ?? 0;
    const prevTotal = prevItem?.wordCount ?? prevDocsBlogTotal + prevAiDocsTotal;

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
      total,
      docsBlogTotal,
      aiDocsTotal,
      delta: total - prevTotal,
      docsBlogDelta: docsBlogTotal - prevDocsBlogTotal,
      aiDocsDelta: aiDocsTotal - prevAiDocsTotal,
      commit: item.commit,
      message: item.message,
      isIncrease: total - prevTotal >= 0,
    };
  });
}
