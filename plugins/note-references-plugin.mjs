/**
 * Docusaurus plugin: 在每篇 ai-docs 笔记底部注入一个「引用关系」方框。
 *
 * 为什么需要它 (而不是在 md 里手写"参考来源"章节):
 *   1. 手写的参考来源是**线性文字**, 读者读完就忘, 也无法一眼看出方向
 *      (这篇引了谁 / 谁引了这篇)。
 *   2. 引用关系本来就是**图**, 该由机器从正文里抽出来, 而不是让人再抄一遍 ——
 *      抄一遍既容易漏, 又必然与正文里的链接不同步。
 *   3. 抽出来之后能区分三个方向, 这是手写做不到的:
 *        · 本文引用 (出边)  —— 正文里链到别的笔记
 *        · 本文被引用 (入边) —— 别的笔记链到本文
 *        · 外部来源 (出边, 站外) —— 引到站外的 URL
 *
 * 注入点: 每篇文档正文末尾 (postBuild 之外走 contentLoaded, 用 remark 插件在
 * mdast 层追加节点 —— 这样能复用 Docusaurus 的链接解析, 也不会被正文里的
 * `## 0x0A 参考来源` 这类手写章节干扰)。
 *
 * 数据 100% 由 ai-docs/ 内容重建, 不含任何状态。
 */

import fs from 'node:fs';
import path from 'node:path';

const CONTENT_DIR = 'ai-docs';
const HXID_RE = /hxid:(hx-[0-9a-f]+)/g;
/** 正文里的 md 链接: [文字](目标 "hxid:hx-xxx") 或 [文字](目标) */
const LINK_RE = /\[([^\]]+)\]\(([^)\s]+)(?:\s+"([^"]*)")?\)/g;

function toPosix(p) {
  return p.replace(/\\/g, '/');
}

/** 去掉路径段前缀数字, 与 Docusaurus DefaultNumberPrefixParser 保持一致 */
function stripNumberPrefix(segment) {
  return segment.replace(/^\d+\s*[-_.]+\s*/, '');
}

/**
 * 复刻 Docusaurus 的 doc slug 规则, 用来产出一条**可点**的站内链接。
 *
 * 与 plugins/tag-index-plugin.mjs 里的同名函数是同一套逻辑 —— 这里刻意复制而不是
 * import, 因为那个文件没有导出它。**改动时两边要一起改**, 否则引用方框的链接会 404。
 */
function computeSlug (relFile, frontMatterSlug) {
  const withoutExt = relFile.replace(/\.mdx?$/i, '');
  const segments = withoutExt.split('/');
  const baseID = stripNumberPrefix(segments[segments.length - 1]);
  const dirSegments = segments.slice(0, -1);

  if (typeof frontMatterSlug === 'string' && frontMatterSlug.trim()) {
    return frontMatterSlug.trim().replace(/^\/+|\/+$/g, '');
  }
  const dirs = dirSegments.map((seg) => stripNumberPrefix(seg));
  const isIndexLike = baseID === 'index' || baseID === 'readme';
  return (isIndexLike ? dirs : [...dirs, baseID]).join('/');
}

/** 递归收集 ai-docs 下所有 index.md 与 .hx-info.md 的路径 */
function walk(dir, out = []) {
  if (!fs.existsSync(dir)) return out;
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      walk(full, out);
    } else if (entry.name === 'index.md' || entry.name === '.hx-info.md') {
      out.push(full);
    }
  }
  return out;
}

/** 从 md 文本里抽 hxid */
function pickHxid(text) {
  const m = /^hxid:\s*"?([\w-]+)"?/m.exec(text);
  return m ? m[1] : null;
}

/** 从 md 文本里抽 title */
function pickTitle(text) {
  const m = /^title:\s*"([^"]+)"/m.exec(text);
  if (m) return m[1];
  const h = /^#\s+(.+)$/m.exec(text);
  return h ? h[1].trim() : null;
}

/* (see .agents/notes/implemented/architecture/2026-09-26-note-references-auto-rendered.md — 为什么不手写参考来源章节) */
export default function noteReferencesPlugin(context, options) {
  const siteDir = context.siteDir;
  const contentDir = path.join(siteDir, options?.contentDir ?? CONTENT_DIR);

  return {
    name: 'note-references-plugin',

    async contentLoaded({ actions }) {
      const files = walk(contentDir);
      if (files.length === 0) return;

      // --- 建索引: hxid -> {title, file}; 文件 -> 它的 hxid ---
      const byHxid = new Map();
      const fileHxid = new Map();
      for (const f of files) {
        const text = fs.readFileSync(f, 'utf8');
        const hxid = pickHxid(text);
        const title = pickTitle(text);
        if (hxid) {
          // 路由: 与 tag-index-plugin 用同一套 slug 规则; 只对 index.md 生成
          // (笔记对外的唯一入口就是它, .hx-info.md 不产页面)
          const rel = toPosix(path.relative(contentDir, f));
          const slug = path.basename(f) === 'index.md' ? computeSlug(rel) : null;
          byHxid.set(hxid, { title: title ?? hxid, file: f, slug });
          fileHxid.set(toPosix(f), hxid);
        }
      }

      // --- 建图: 每篇文档的出边 ---
      /** @type {Map<string, {out: Set<string>, in: Set<string>, ext: Set<string>}>} */
      const graph = new Map();
      const ensure = (hxid) => {
        if (!graph.has(hxid)) graph.set(hxid, { out: new Set(), in: new Set(), ext: new Set() });
        return graph.get(hxid);
      };

      for (const f of files) {
        const text = fs.readFileSync(f, 'utf8');
        const selfHxid = fileHxid.get(toPosix(f));
        if (!selfHxid) continue;
        const self = ensure(selfHxid);
        const dir = path.dirname(f);

        for (const m of text.matchAll(LINK_RE)) {
          const [, , target, hxidInTitle] = m;
          // 1) md 链接的 title 里带 hxid -> 站内跨笔记引用
          const hid = hxidInTitle ? /hxid:(hx-[0-9a-f]+)/.exec(hxidInTitle)?.[1] : null;
          if (hid && hid !== selfHxid && byHxid.has(hid)) {
            self.out.add(hid);
            ensure(hid).in.add(selfHxid);
            continue;
          }
          // 2) 站外链接
          if (/^https?:\/\//.test(target)) {
            self.ext.add(target);
            continue;
          }
          // 3) 相对路径 -> 用 hxid 反查 (兼容手写漏 title 的情况)
          if (/^[.][./]/.test(target) || target.endsWith('.md') || target.endsWith('/')) {
            const abs = path.resolve(dir, target.replace(/#.*$/, ''));
            for (const cand of [abs, path.join(abs, 'index.md'), abs + '.md']) {
              const hid2 = fileHxid.get(toPosix(cand));
              if (hid2 && hid2 !== selfHxid && byHxid.has(hid2)) {
                self.out.add(hid2);
                ensure(hid2).in.add(selfHxid);
                break;
              }
            }
          }
        }
      }

      // --- 落盘: 页面通过它取数据 (同 tag-index-plugin 的模式) ---
      // 写成 .ts 而不是 .json: 让 import 拿到类型, 页面无需运行时 fetch。
      const payload = {
        byHxid: Object.fromEntries([...byHxid].map(([k, v]) => [k, { title: v.title }])),
        graph: Object.fromEntries(
          [...graph].map(([k, v]) => [k, {
            out: [...v.out],
            in: [...v.in],
            ext: [...v.ext],
          }]),
        ),
      };
      const outFile = path.join(siteDir, 'data', 'noteReferences.ts');
      fs.mkdirSync(path.dirname(outFile), { recursive: true });
      fs.writeFileSync(
        outFile,
        `// Auto-generated by note-references-plugin — do not edit manually.\n` +
        `export interface NoteRefEntry { out: string[]; in: string[]; ext: string[] }\n` +
        `export const noteRefTitles: Record<string, string> = ${JSON.stringify(
          Object.fromEntries([...byHxid].map(([k, v]) => [k, v.title])), null, 0)};\n` +
        `export const noteRefPermalinks: Record<string, string> = ${JSON.stringify(
          Object.fromEntries([...byHxid].filter(([, v]) => v.slug).map(([k, v]) => [k, v.slug])), null, 0)};\n` +
        `export const noteRefGraph: Record<string, NoteRefEntry> = ${JSON.stringify(payload.graph, null, 0)};\n`,
        'utf8',
      );
      console.log(
        `[note-references] ${byHxid.size} 篇笔记, ` +
        `${[...graph.values()].reduce((n, g) => n + g.out.size, 0)} 条站内引用, ` +
        `${[...graph.values()].reduce((n, g) => n + g.ext.size, 0)} 条外部来源`,
      );
    },
  };
}
