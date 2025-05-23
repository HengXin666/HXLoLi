#!/usr/bin/env ts-node
import { execSync } from "child_process";
import * as fs from "fs";
import * as path from "path";

interface RecordItem {
  commit: string;
  date: string;
  wordCount: number; // 累计字符数
}

interface Cache {
  lastProcessed?: string;
  lastWordCount: number;
}

const ROOT_DIR = path.resolve(__dirname, "..");
const DATA_DIR = path.resolve(ROOT_DIR, "data");
const CACHE_FILE = path.resolve(DATA_DIR, ".md_count_cache.json");
// 输出 TSX 文件
const OUTPUT_TS = path.resolve(ROOT_DIR, "./data/wordStats.ts");

// 屏蔽的目录
const EXCLUDE_PREFIXES: string[] = [
  "node_modules/",
  "scripts/",
];

if (!fs.existsSync(DATA_DIR)) {
  fs.mkdirSync(DATA_DIR, { recursive: true });
}

function loadCache(): Cache {
  if (fs.existsSync(CACHE_FILE)) {
    return JSON.parse(fs.readFileSync(CACHE_FILE, "utf-8")) as Cache;
  }
  return { lastWordCount: 0 };
}

function saveCache(cache: Cache): void {
  fs.writeFileSync(CACHE_FILE, JSON.stringify(cache, null, 2), "utf-8");
}

function isExcluded(filePath: string): boolean {
  return EXCLUDE_PREFIXES.some((prefix) => filePath.startsWith(prefix));
}

function countTextLength(text: string): number {
  return text.replace(/\s+/g, "").length;
}

function parseDiffAndCount(diff: string): number {
  let added = 0;
  let removed = 0;
  const lines = diff.split("\n");
  let skip = false;
  for (const line of lines) {
    if (line.startsWith("diff --git")) {
      // determine file path
      const parts = line.split(" ");
      const file = parts[2].slice(2);
      skip = isExcluded(file);
      continue;
    }
    if (skip) continue;
    if (
      line.startsWith("+++") ||
      line.startsWith("---") ||
      line.startsWith("@@")
    )
      continue;
    if (line.startsWith("+")) added += countTextLength(line.slice(1));
    else if (line.startsWith("-")) removed += countTextLength(line.slice(1));
  }
  return added - removed;
}

function getMdDiff(commit: string): string {
  try {
    const isWin = process.platform === "win32";
    const shell = isWin ? "powershell.exe" : "/bin/bash";
    return execSync(`git diff --unified=0 ${commit}^ ${commit} -- "*.md"`, {
      encoding: "utf8",
      maxBuffer: 20 * 1024 * 1024,
      shell,
    });
  } catch {
    return "";
  }
}

function getAllCommits(): string[] {
  const out = execSync("git rev-list --reverse HEAD", { encoding: "utf-8" });
  return out.trim().split("\n").filter(Boolean);
}

function getCommitDate(hash: string): string {
  return execSync(`git show -s --format=%cI ${hash}`, {
    encoding: "utf-8",
  }).trim();
}

function writeTS(data: RecordItem[]): void {
  const header = `// 此文件由脚本自动生成，包含每次提交的 Markdown 累计字数统计
export interface RecordItem { commit: string; date: string; wordCount: number; }

export const stats: RecordItem[] = `;
  const content = header + JSON.stringify(data, null, 2) + ";\n";
  fs.writeFileSync(OUTPUT_TS, content, "utf-8");
  console.log(`已写入 TS 文件：${OUTPUT_TS}`);
}

function main(): void {
  const cache = loadCache();
  const commits = getAllCommits();
  const startIdx = cache.lastProcessed
    ? commits.indexOf(cache.lastProcessed) + 1
    : 0;
  let lastCount = cache.lastWordCount || 0;
  const result: RecordItem[] = [];

  for (const commit of commits.slice(startIdx)) {
    const date = getCommitDate(commit);
    const diff = getMdDiff(commit);
    const delta = parseDiffAndCount(diff);
    lastCount = Math.max(0, lastCount + delta);
    result.push({ commit, date, wordCount: lastCount });
    console.log(`提交 ${commit}: 累计 ${lastCount} (增量 ${delta})`);
    cache.lastProcessed = commit;
    cache.lastWordCount = lastCount;
  }

  if (result.length) {
    writeTS(result);
    saveCache(cache);
    console.log("脚本完成。");
  } else {
    console.log("无新提交需要处理。");
  }
}

main();
