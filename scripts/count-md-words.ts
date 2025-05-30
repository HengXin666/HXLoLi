#!/usr/bin/env ts-node
import { execSync } from "child_process";
import * as fs from "fs";
import * as path from "path";

interface RecordItem {
  commit: string;
  date: string;
  wordCount: number;
  message: string; // 提交信息
}

interface Cache {
  lastProcessed?: string;
  lastWordCount: number;
}

const ROOT_DIR: string = path.resolve(__dirname, "..");
const DATA_DIR: string = path.resolve(ROOT_DIR, "data");
const CACHE_FILE: string = path.resolve(DATA_DIR, ".md_count_cache.json");
const OUTPUT_TS: string = path.resolve(DATA_DIR, "wordStats.ts");

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
  return EXCLUDE_PREFIXES.some(prefix => filePath.startsWith(prefix));
}

function countTextLength(text: string): number {
  return text.replace(/\s+/g, "").length;
}

function parseDiffAndCount(diff: string): number {
  let added: number = 0;
  let removed: number = 0;
  let skip: boolean = false;
  for (const line of diff.split("\n")) {
    if (line.startsWith("diff --git")) {
      const parts: string[] = line.split(" ");
      const file: string = parts[2].slice(2);
      skip = isExcluded(file);
      continue;
    }
    if (skip) continue;
    if (line.startsWith("+++") || line.startsWith("---") || line.startsWith("@@")) {
      continue;
    }
    if (line.startsWith("+")) {
      added += countTextLength(line.slice(1));
    } else if (line.startsWith("-")) {
      removed += countTextLength(line.slice(1));
    }
  }
  return added - removed;
}

function getMdDiff(commit: string): string {
  try {
    const isWin: boolean = process.platform === "win32";
    const shell: string = isWin ? "powershell.exe" : "/bin/bash";
    const cmd: string = commit === "WORKING_DIR"
      ? `git diff --unified=0 HEAD -- "*.md"`
      : `git diff --unified=0 ${commit}^ ${commit} -- "*.md"`;
    return execSync(cmd, { encoding: "utf8", maxBuffer: 20 * 1024 * 1024, shell });
  } catch {
    return "";
  }
}

function getCommitDate(hash: string): string {
  return execSync(`git show -s --format=%cI ${hash}`, { encoding: "utf-8" }).trim();
}

function getCommitMessage(hash: string): string {
  return execSync(`git show -s --format=%s ${hash}`, { encoding: "utf-8" }).trim();
}

function getAllCommits(): string[] {
  const out: string = execSync("git rev-list --reverse HEAD", { encoding: "utf-8" });
  return out.trim().split("\n").filter(Boolean);
}

function loadExistingStats(): RecordItem[] {
  if (!fs.existsSync(OUTPUT_TS)) return [];
  const content: string = fs.readFileSync(OUTPUT_TS, "utf-8");
  const match: RegExpMatchArray | null = content.match(/export const stats: RecordItem\[\] = ([\s\S]*);/);
  if (match && match[1]) {
    const items: RecordItem[] = JSON.parse(match[1]);
    return items.filter(item => item.commit !== "WORKING_DIR");
  }
  return [];
}

function writeTS(data: RecordItem[]): void {
  const header: string = `// 此文件由脚本自动生成，包含每次提交及当前工作区的 Markdown 累计字数统计，包括提交信息
export interface RecordItem { commit: string; date: string; wordCount: number; message: string; }

export const stats: RecordItem[] = `;
  const content: string = header + JSON.stringify(data, null, 2) + ";\n";
  fs.writeFileSync(OUTPUT_TS, content, "utf-8");
  console.log(`已写入 TS 文件：${OUTPUT_TS}`);
}

function main(): void {
  const args: string[] = process.argv.slice(2);
  if (args.length < 1) {
    console.error("请提供未提交状态的提交信息作为参数");
    process.exit(1);
  }
  const commitMessage: string = args.join(' ');

  const cache: Cache = loadCache();
  const commits: string[] = getAllCommits();
  const existingData: RecordItem[] = loadExistingStats();
  const existingCommits: Set<string> = new Set(existingData.map(item => item.commit));
  const startIdx: number = cache.lastProcessed ? commits.indexOf(cache.lastProcessed) + 1 : 0;

  let lastCount: number = cache.lastWordCount || 0;
  const result: RecordItem[] = [...existingData];

  for (const commit of commits.slice(startIdx)) {
    if (existingCommits.has(commit)) continue;
    const date: string = getCommitDate(commit);
    const diff: string = getMdDiff(commit);
    const delta: number = parseDiffAndCount(diff);
    lastCount = Math.max(0, lastCount + delta);
    const msg: string = getCommitMessage(commit);
    result.push({ commit, date, wordCount: lastCount, message: msg });
    console.log(`提交 ${commit}: 累计 ${lastCount} (增量 ${delta}), 信息: ${msg}`);
    cache.lastProcessed = commit;
    cache.lastWordCount = lastCount;
  }

  // 追加最新工作区状态，使用用户输入的提交信息
  const workingDiff: string = getMdDiff("WORKING_DIR");
  const workingDelta: number = parseDiffAndCount(workingDiff);
  const workingTotal: number = Math.max(0, lastCount + workingDelta);
  const workingDate: string = new Date().toISOString();
  result.push({
    commit: "WORKING_DIR",
    date: workingDate,
    wordCount: workingTotal,
    message: commitMessage,
  });
  console.log(`工作区: 累计 ${workingTotal} (相较 HEAD 差 ${workingDelta}), 信息: ${commitMessage}`);

  writeTS(result);
  saveCache(cache);
  console.log("脚本完成。");
}

main();
