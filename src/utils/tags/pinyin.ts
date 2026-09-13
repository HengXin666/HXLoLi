import { pinyin } from 'pinyin-pro';

/**
 * 标签的拼音检索键 (中文友好)
 *
 * 为什么需要它:
 *   Docusaurus 原生 `listTagsByLetters` 用 `label[0]` 作为分组键 —— 对中文来说
 *   就是"只按第一个汉字分组", 于是「多模态大模型」「短视频」「电脑包」全挤进"多"那一组,
 *   索引完全失去意义。
 *
 * 这里把每个标签归一成三个检索/分组用的键:
 *   - groupKey: 索引分组用。拉丁字母开头取该字母, 汉字取**拼音首字母** (记忆 -> J),
 *     其它 (数字/符号) 归到 '#'。
 *   - full:     拼音全拼, 无声调 (记忆系统 -> "jiyixitong"), 供搜索用。
 *   - initials: 拼音首字母缩写 (记忆系统 -> "jyxt"), 供搜索用。
 *
 * 实现依赖 pinyin-pro 自带词典 (收录多音字常见读音, 如 现代 -> xian dai), 而不是
 * `Intl.Collator` 拼音比较: 后者只能比大小, 拿不到字母本身, 且拼音区间在标点处有空洞,
 * 边界汉字会误判。
 *
 * 这些键只在客户端算一次并用 Map 缓存; 标签总数是百量级, 成本可忽略。
 */

/** 非字母开头 (数字、符号) 的标签归到这一组, 排序时恒在最前 */
export const UNKNOWN_GROUP = '#';

export interface TagPinyinKeys {
  groupKey: string;
  /** 全拼, 无分隔且无声调; 非汉字部分原样保留 (现代C++ -> "xiandaic++") */
  full: string;
  /** 首字母缩写 (记忆系统 -> "jyxt"), 供快速输入用 */
  initials: string;
}

const cache = new Map<string, TagPinyinKeys>();

function isHan(char: string): boolean {
  return /[\u4e00-\u9fff]/.test(char);
}

/** 去掉拼音串里的分隔空格并小写 —— 检索键不需要保留词界 */
function squeeze(value: string): string {
  return value.replace(/\s+/g, '').toLowerCase();
}

function buildKeys(label: string): TagPinyinKeys {
  const trimmed = label.trim();
  if (!trimmed) {
    return { groupKey: UNKNOWN_GROUP, full: '', initials: '' };
  }

  // 纯拉丁/数字标签 (AI Agent / HXLibs / C++20): 不做音译, 检索键就是原文小写。
  // 不能按"首字符是不是字母"来分流 —— "CF过盾"以 C 开头但仍要音译出 guo dun。
  if (!/[\u4e00-\u9fff]/.test(trimmed)) {
    const flat = squeeze(trimmed);
    const head = trimmed.charAt(0);
    return {
      groupKey: /^[A-Za-z]$/.test(head) ? head.toUpperCase() : UNKNOWN_GROUP,
      full: flat,
      initials: flat,
    };
  }

  // 全拼: 每个汉字出完整拼音, 非汉字逐字符出 (CF过盾 -> CF guo dun), 去掉分隔符压成一串
  const full = squeeze(pinyin(trimmed, { toneType: 'none', nonZh: 'spaced' }));
  // 首字母: pattern 'first' 让汉字只出首字母 (CF过盾 -> C F g d), 非汉字保持原样
  const initials = squeeze(pinyin(trimmed, { toneType: 'none', nonZh: 'spaced', pattern: 'first' }));
  const groupKey = /^[a-z]/.test(initials) ? initials.charAt(0).toUpperCase() : UNKNOWN_GROUP;

  return { groupKey, full, initials };
}

/** 取标签的拼音检索键 (带缓存) */
export function getTagPinyinKeys(label: string): TagPinyinKeys {
  const cached = cache.get(label);
  if (cached) return cached;
  const keys = buildKeys(label);
  cache.set(label, keys);
  return keys;
}

/** 索引分组键: 记忆系统 -> 'J', AI Agent -> 'A', C++20 -> 'C', 003-编程语言 -> '#' */
export function getTagGroupKey(label: string): string {
  return getTagPinyinKeys(label).groupKey;
}

/** 标签排序: 中文按拼音, 英文按字母 (只比大小, 不解析读音) */
export function compareTagLabels(a: string, b: string): number {
  return a.localeCompare(b, 'zh-Hans-CN');
}

/** 分组键排序: '#' 恒在最前, 其后 A-Z */
export function compareGroupKeys(a: string, b: string): number {
  if (a === b) return 0;
  if (a === UNKNOWN_GROUP) return -1;
  if (b === UNKNOWN_GROUP) return 1;
  return a < b ? -1 : 1;
}

/**
 * 判断标签是否命中搜索词。
 *
 * 支持三种输入: 中文原文 (记忆)、拼音全拼 (jiyi / ji yi)、首字母缩写 (jyxt)。
 * 比对全拼时去掉空格, 因为用户不会按拼音词间空格来打字。
 */
export function matchesTagQuery(label: string, rawQuery: string): boolean {
  const query = rawQuery.trim().toLowerCase();
  if (!query) return true;

  const { full, initials } = getTagPinyinKeys(label);
  return (
    label.toLowerCase().includes(query) ||
    full.replace(/\s+/g, '').includes(query.replace(/\s+/g, '')) ||
    initials.includes(query.replace(/\s+/g, ''))
  );
}

export interface TagIndexEntry<T> {
  key: string;
  tags: T[];
}

/** 把标签按分组键聚合、组内按拼音排序、组间按 A-Z 排序 */
export function groupTagsByInitial<T>(
  tags: readonly T[],
  getLabel: (tag: T) => string = (tag) => (tag as { label: string }).label,
): TagIndexEntry<T>[] {
  const groups = new Map<string, T[]>();

  for (const tag of tags) {
    const key = getTagGroupKey(getLabel(tag));
    const bucket = groups.get(key);
    if (bucket) bucket.push(tag);
    else groups.set(key, [tag]);
  }

  return [...groups.entries()]
    .map(([key, bucket]) => ({
      key,
      tags: [...bucket].sort((a, b) => compareTagLabels(getLabel(a), getLabel(b))),
    }))
    .sort((a, b) => compareGroupKeys(a.key, b.key));
}

