#!/usr/bin/env ts-node

import { execSync } from "child_process";
import * as fs from "fs";
import * as path from "path";

interface RecordItem {
  commit: string;
  date: string;
  wordCount: number;
  docsBlogWordCount: number;
  aiDocsWordCount: number;
  message: string; // 提交信息
}

interface Cache {
  schemaVersion?: number;
  lastProcessed?: string;
  lastWordCount: number;
  lastDocsBlogWordCount?: number;
  lastAiDocsWordCount?: number;
}

const ROOT_DIR: string = path.resolve(__dirname, "..");
const DATA_DIR: string = path.resolve(ROOT_DIR, "data");
const CACHE_FILE: string = path.resolve(DATA_DIR, ".md_count_cache.json");
const OUTPUT_TS: string = path.resolve(DATA_DIR, "wordStats.ts");
const CACHE_SCHEMA_VERSION = 2;

const EXCLUDE_PREFIXES: string[] = [
  "node_modules/",
  "scripts/",
];

// 新增: 需要过滤的提交信息前缀
const IGNORE_COMMIT_PREFIXES: string[] = [
  "[feat]",
  "[fix]",
  "[LoLi-Bot]",
  "Merge"
];

if (!fs.existsSync(DATA_DIR)) {
  fs.mkdirSync(DATA_DIR, { recursive: true });
}

type CountField = "docsBlogWordCount" | "aiDocsWordCount";

interface WordCounts {
  wordCount: number;
  docsBlogWordCount: number;
  aiDocsWordCount: number;
}

function createEmptyCounts(): WordCounts {
  return {
    wordCount: 0,
    docsBlogWordCount: 0,
    aiDocsWordCount: 0,
  };
}

function addCounts(base: WordCounts, delta: WordCounts): WordCounts {
  const docsBlogWordCount = Math.max(0, base.docsBlogWordCount + delta.docsBlogWordCount);
  const aiDocsWordCount = Math.max(0, base.aiDocsWordCount + delta.aiDocsWordCount);

  return {
    docsBlogWordCount,
    aiDocsWordCount,
    wordCount: docsBlogWordCount + aiDocsWordCount,
  };
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

function normalizeGitPath(filePath: string): string {
  return filePath
    .replace(/^"|"$/g, "")
    .replace(/\\/g, "/");
}

function parseDiffFilePath(line: string): string | null {
  const normalMatch = line.match(/^diff --git a\/(.+) b\/(.+)$/);
  if (normalMatch?.[2]) {
    return normalizeGitPath(normalMatch[2]);
  }

  const quotedMatch = line.match(/^diff --git "a\/(.+)" "b\/(.+)"$/);
  if (quotedMatch?.[2]) {
    return normalizeGitPath(quotedMatch[2]);
  }

  return null;
}

function getCountField(filePath: string): CountField | null {
  if (isExcluded(filePath)) {
    return null;
  }
  if (filePath.startsWith("docs/") || filePath.startsWith("blog/")) {
    return "docsBlogWordCount";
  }
  if (filePath.startsWith("ai-docs/")) {
    return "aiDocsWordCount";
  }
  return null;
}

function parseDiffAndCount(diff: string): WordCounts {
  const delta: WordCounts = createEmptyCounts();
  let countField: CountField | null = null;

  for (const line of diff.split("\n")) {
    if (line.startsWith("diff --git")) {
      const filePath = parseDiffFilePath(line);
      countField = filePath ? getCountField(filePath) : null;
      continue;
    }
    if (countField === null) continue;

    if (line.startsWith("+++") || line.startsWith("---") || line.startsWith("@@")) {
      continue;
    }

    if (line.startsWith("+")) {
      delta[countField] += countTextLength(line.slice(1));
    } else if (line.startsWith("-")) {
      delta[countField] -= countTextLength(line.slice(1));
    }
  }

  delta.wordCount = delta.docsBlogWordCount + delta.aiDocsWordCount;
  return delta;
}

function isRootCommit(commit: string): boolean {
  const out = execSync(`git rev-list --parents -n 1 ${commit}`, { encoding: "utf-8" }).trim();
  return out.split(/\s+/).length === 1;
}

function getMdDiff(commit: string): string {
  try {
    const isWin: boolean = process.platform === "win32";
    const shell: string = isWin ? "powershell.exe" : "/bin/bash";
    const markdownPathspec = `"*.md" "*.mdx"`;
    const git = "git -c core.quotePath=false";
    const cmd: string = commit === "WORKING_DIR"
      ? `${git} diff --unified=0 HEAD -- ${markdownPathspec}`
      : isRootCommit(commit)
        ? `${git} diff-tree --root -p --unified=0 --no-commit-id -r ${commit} -- ${markdownPathspec}`
        : `${git} diff --unified=0 ${commit}^ ${commit} -- ${markdownPathspec}`;
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

function hasCategoryCounts(item: RecordItem): boolean {
  return typeof item.docsBlogWordCount === "number"
    && typeof item.aiDocsWordCount === "number";
}

function isCacheCurrent(cache: Cache): boolean {
  return cache.schemaVersion === CACHE_SCHEMA_VERSION
    && typeof cache.lastDocsBlogWordCount === "number"
    && typeof cache.lastAiDocsWordCount === "number";
}

function getCountsFromCache(cache: Cache): WordCounts {
  const docsBlogWordCount = cache.lastDocsBlogWordCount ?? 0;
  const aiDocsWordCount = cache.lastAiDocsWordCount ?? 0;
  return {
    docsBlogWordCount,
    aiDocsWordCount,
    wordCount: docsBlogWordCount + aiDocsWordCount,
  };
}

function saveCountsToCache(cache: Cache, counts: WordCounts): void {
  cache.schemaVersion = CACHE_SCHEMA_VERSION;
  cache.lastWordCount = counts.wordCount;
  cache.lastDocsBlogWordCount = counts.docsBlogWordCount;
  cache.lastAiDocsWordCount = counts.aiDocsWordCount;
}

function writeTS(data: RecordItem[]): void {
  const header: string = `// 此文件由脚本自动生成, 包含每次提交及当前工作区的 Markdown 累计字数统计, 区分 docs/blog 与 ai-docs
export interface RecordItem { commit: string; date: string; wordCount: number; docsBlogWordCount: number; aiDocsWordCount: number; message: string; }
export const stats: RecordItem[] = `;
  const content: string = header + JSON.stringify(data, null, 2) + ";\n";
  fs.writeFileSync(OUTPUT_TS, content, "utf-8");
  console.log(`已写入 TS 文件: ${OUTPUT_TS}`);
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
  const needsFullRebuild = !isCacheCurrent(cache) || existingData.some(item => !hasCategoryCounts(item));
  const seedData = needsFullRebuild ? [] : existingData;
  const existingCommits: Set<string> = new Set(seedData.map(item => item.commit));

  if (needsFullRebuild) {
    console.log("检测到旧版字数统计缓存, 将重建历史分类统计。");
  }

  const startIdx: number = needsFullRebuild
    ? 0
    : cache.lastProcessed
      ? commits.indexOf(cache.lastProcessed) + 1
      : 0;
  let lastCounts: WordCounts = needsFullRebuild ? createEmptyCounts() : getCountsFromCache(cache);
  const result: RecordItem[] = [...seedData];

  for (const commit of commits.slice(startIdx)) {
    if (existingCommits.has(commit)) continue;

    const date: string = getCommitDate(commit);
    const diff: string = getMdDiff(commit);
    const delta: WordCounts = parseDiffAndCount(diff);
    lastCounts = addCounts(lastCounts, delta);
    const msg: string = getCommitMessage(commit);

    // 检查是否需要过滤
    const shouldIgnore = IGNORE_COMMIT_PREFIXES.some(prefix => msg.startsWith(prefix));

    if (!shouldIgnore) {
      result.push({ commit, date, ...lastCounts, message: msg });
      console.log(
        `提交 ${commit}: 累计 ${lastCounts.wordCount} (docs/blog ${lastCounts.docsBlogWordCount}, ai-docs ${lastCounts.aiDocsWordCount}, 增量 ${delta.wordCount}), 信息: ${msg}`
      );
    } else {
      console.log(
        `提交 ${commit}: 累计 ${lastCounts.wordCount} (docs/blog ${lastCounts.docsBlogWordCount}, ai-docs ${lastCounts.aiDocsWordCount}, 增量 ${delta.wordCount}), 信息: ${msg} [已过滤]`
      );
    }

    cache.lastProcessed = commit;
    saveCountsToCache(cache, lastCounts);
  }

  // 追加最新工作区状态, 使用用户输入的提交信息
  const workingDiff: string = getMdDiff("WORKING_DIR");
  const workingDelta: WordCounts = parseDiffAndCount(workingDiff);
  const workingCounts: WordCounts = addCounts(lastCounts, workingDelta);
  const workingDate: string = new Date().toISOString();

  // 也可以选择是否对工作区信息进行过滤, 通常工作区是用来预览的, 这里保留不过滤, 或者你可以加上同样的判断
  result.push({
    commit: "WORKING_DIR",
    date: workingDate,
    ...workingCounts,
    message: commitMessage,
  });

  console.log(
    `工作区: 累计 ${workingCounts.wordCount} (docs/blog ${workingCounts.docsBlogWordCount}, ai-docs ${workingCounts.aiDocsWordCount}, 相较 HEAD 差 ${workingDelta.wordCount}), 信息: ${commitMessage}`
  );

  writeTS(result);
  saveCache(cache);

  console.log("脚本完成。");
}

main();
