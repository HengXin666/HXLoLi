/**
 * Docusaurus 引用关系插件  从正文抽出「本文引用 / 本文被引用 / 站外来源」三类边。
 *
 * 为什么需要它 (而不是在 md 里手写「参考来源」章节):
 *   手写的参考来源是**线性文字**, 读者读完就忘, 也看不出方向 (这篇引了谁 / 谁在依赖
 *   这篇的结论); 而且它必须人工与正文同步, 必然漂移。引用关系本来就是图, 该由机器从
 *   正文里抽出来, 而不是让人再抄一遍。
 *
 * 两个部分协作:
 *   · default 导出是 **Docusaurus 插件**: 在 allContentLoaded 里扫正文、并用
 *     Docusaurus 自己解析出的 source -> permalink / title 表把结果落盘成一份
 *     「每篇一份载荷」的 JSON。
 *   · noteReferencesRemark 是 **remark 插件**: 编译每篇 md 时按路径查那份 JSON, 把该篇
 *     载荷注入成组件 props。
 *
 * 为什么 slug 必须问 Docusaurus 要: 规则含数字前缀剥离、index 归一、frontmatter slug
 * 覆盖, 复刻必然漂移  而仓库里已经有这个函数的两份副本了 (docusaurus.config.ts 与
 * tag-index-plugin.mjs), 再加一份就是第三个会跑偏的真相源。
 *
 * 为什么载荷由 remark 注入 props, 而不是组件 import 一份全站表:
 *   边表是全站的, 实测 1000+ 篇共约 364KB (gzip 约 107KB)。组件 import 会让它进全站
 *   共享 chunk, 每一篇页面都替**所有**页面付这份钱; 注入 props 后数据随该页自己的 MDX
 *   chunk 走, 别的页面一字节都不付。
 *
 * 为什么扫描只在插件里做一次、remark 只查表:
 *   md 的编译可能发生在 webpack 的 worker 里。若 remark 自己扫内容目录, 每个编译单元都
 *   会重复读上千个文件 (935 篇 x 1010 次读取)。把扫描钉在插件这一次, remark 退化成一次
 *   文件读取 + 一次查表。
 *
 * (see 
 *  — 为什么引用关系由构建期抽取而不是手写章节)
 */

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

import matter from 'gray-matter';

/**
 * 引用关系方框改为全站注入, 站外来源按「域名@标题」显示
 * .agents/notes/implemented/architecture/2026-09-26-note-references-auto-rendered.md
 */
const PLUGIN_DIR = path.dirname(fileURLToPath(import.meta.url));
const SITE_DIR = path.resolve(PLUGIN_DIR, '..');
const BACKTICK = String.fromCharCode(96);
const FENCE = BACKTICK.repeat(3);

/** 三个内容区。顺序只影响扫描与日志, 不影响结果。 */
const SECTIONS = [
  { key: 'ai-docs', dir: 'ai-docs' },
  { key: 'docs', dir: 'docs' },
  { key: 'blog', dir: 'blog' },
];

/** 正文里的 md 链接: [文字](目标 "title") */
const LINK_RE = /\[([^\]]*)\]\(([^)\s]+)(?:\s+["']([^"']*)["'])?\)/g;
/** 裸 URL 自动链接 */
const AUTOLINK_RE = /<(https?:\/\/[^>\s]+)>/g;
/** 原生 HTML 锚点 */
const ANCHOR_RE = /<a\s[^>]*href=["'](https?:\/\/[^"']+)["'][^>]*>([\s\S]*?)<\/a>/gi;
/** 指向静态资源而不是文档的目标  这些不是引用 (图表 / PPT / 源码 / schema 等) */
const ASSET_RE =
  /\.(png|jpe?g|svg|gif|webp|bmp|ico|drawio|html?|tsx|jsx|ts|js|json|css|scss|txt|pdf|zip|gz|tar|mp4|mp3|webm|xml|ya?ml|toml|c|cc|cpp|cxx|h|hpp|py|java|go|rs|sh|sql|ipynb)([?#]|$)/i;

const LABEL_NOISE_RE = new RegExp('[' + BACKTICK + '*]', 'g');
const FENCE_RE = new RegExp(FENCE + '[\\s\\S]*?' + FENCE, 'g');
const TILDE_FENCE_RE = new RegExp('~~~[\\s\\S]*?~~~', 'g');

const PAYLOAD_FILE_NAME = 'noteReferences.json';
const ALIAS_PREFIX = '@site/';

function toPosix(p) {
  return p.replace(/\\/g, '/');
}

/** 去掉代码围栏内的内容, 避免把代码示例里的链接算成引用 */
function stripCodeBlocks(text) {
  return text.replace(FENCE_RE, '\n').replace(TILDE_FENCE_RE, '\n');
}

/** Docusaurus 以 dot:false 读取内容目录, 点开头与下划线开头的条目都不产页面 */
function isIgnoredEntry(name) {
  return name.startsWith('.') || name.startsWith('_');
}

function walkMarkdown(dir, out = []) {
  let entries;
  try {
    entries = fs.readdirSync(dir, { withFileTypes: true });
  } catch {
    return out;
  }
  for (const entry of entries) {
    if (isIgnoredEntry(entry.name)) continue;
    const full = path.join(dir, entry.name);
    let stat;
    try {
      stat = fs.statSync(full);
    } catch {
      continue;
    }
    if (stat.isDirectory()) walkMarkdown(full, out);
    else if (/\.mdx?$/i.test(entry.name)) out.push(path.resolve(full));
  }
  return out;
}

/**
 * 私有内容不进公开的引用框。
 *
 * 三条判据缺一不可, 因为它们在**不同机器上各有失效场景**:
 *
 * 1. **解析到站点目录之外**  覆盖符号链接 (docs/015-私密笔记、blog/2026/04/30、
 *    ai-docs/.hx-persona.md 都指向同级私有仓 HXLoLi-imouto)。别的机器没跑
 *    setup-private 时这些链接根本不存在, 所以这条只在本机生效  但它必须**同时判
 *    链接本身与解析结果**: 直接判 realpath 只能发现"已经断了"的链接, 已经挂上的私有
 *    仓内容 realpath 是通的, 会整批漏过 (实测漏 4 条)。
 * 2. **路径段以 `_` 开头**  Docusaurus 的 globby 以 dot:false 读取内容, 这是它自己
 *    的排除约定, 与私有性无关但同样不该出现在引用框里。
 * 3. **`hx_protected`**  加密流程写进 frontmatter 的标记, 是唯一一条跟着**内容**走的
 *    判据, 因此也是唯一在别人的机器上仍然有效的那条。
 *
 * 判据 1 用 `lstatSync` 拿链接自身、用 `realpathSync` 拿落点, 两者都检查:
 * 只要有一边出界就排除。这样"链接指向仓外"与"链接已断"都不会漏。
 */
function buildPrivacyFilter(siteReal) {
  const sitePrefix = siteReal + path.sep;
  return function isPrivate(abs, frontMatter) {
    let real;
    try {
      real = fs.realpathSync(abs);
    } catch {
      return true; // 断链: 连内容都读不到, 更不该进引用框
    }
    if (!real.startsWith(sitePrefix)) return true;
    // 链接自身的落点也要在站内  上面的 realpath 已经能覆盖"指向站外"的情况,
    // 这一条额外挡住"链接本身就在站外目录但仍解析进来"的形态。
    const lexical = path.resolve(abs);
    if (!lexical.startsWith(sitePrefix)) return true;
    if (toPosix(path.relative(siteReal, real)).split('/').some((seg) => seg.startsWith('_'))) return true;
    if (toPosix(path.relative(siteReal, lexical)).split('/').some((seg) => seg.startsWith('_'))) return true;
    if (frontMatter && frontMatter.hx_protected === true) return true;
    return false;
  };
}

function pickTitle(frontMatter, body, fallback) {
  const raw = frontMatter?.title;
  if (typeof raw === 'string' && raw.trim()) return raw.trim();
  const heading = /^#\s+(.+)$/m.exec(body);
  if (heading) return heading[1].trim();
  return fallback;
}

function pickHxid(frontMatter) {
  const raw = frontMatter?.hxid;
  return typeof raw === 'string' && raw.trim() ? raw.trim() : null;
}

function cleanLabel(text) {
  return String(text ?? '').replace(LABEL_NOISE_RE, '').replace(/\s+/g, ' ').trim();
}

/**
 * 扫描三个内容区, 建出「谁引用了谁」与「引了哪些站外 URL」。
 *
 * 站内边只认**同区**引用: Docusaurus 按插件解析 markdown 链接, 跨区相对路径不会被
 * resolve, 注入出去就是死链。
 *
 * 键用「相对站点根的 posix 路径」, 与 Docusaurus 的 @site/ 别名同构  这样才能和插件
 * 从 allContent 拿到的权威表直接对上。
 */
function scanContent(siteDir) {
  const siteReal = fs.realpathSync(siteDir);
  const isPrivate = buildPrivacyFilter(siteReal);

  const documents = new Map();
  const byHxid = new Map();
  const keyOf = new Map();

  for (const section of SECTIONS) {
    for (const abs of walkMarkdown(path.join(siteDir, section.dir))) {
      let raw;
      try {
        raw = fs.readFileSync(abs, 'utf8');
      } catch {
        continue;
      }
      let parsed;
      try {
        parsed = matter(raw);
      } catch {
        parsed = { data: {}, content: raw };
      }
      const frontMatter = parsed.data ?? {};
      if (isPrivate(abs, frontMatter)) continue;

      const key = toPosix(path.relative(siteDir, abs));
      const record = {
        abs,
        key,
        section: section.key,
        title: pickTitle(frontMatter, parsed.content ?? '', path.basename(path.dirname(abs))),
        hxid: pickHxid(frontMatter),
      };
      documents.set(key, record);
      keyOf.set(abs, key);
      if (record.hxid && !byHxid.has(record.hxid)) byHxid.set(record.hxid, key);
    }
  }

  const graph = new Map();
  const ensure = (key) => {
    if (!graph.has(key)) graph.set(key, { out: [], in: [], ext: [] });
    return graph.get(key);
  };
  for (const key of documents.keys()) ensure(key);

  const resolveTarget = (from, target, title) => {
    // 1) md 链接 title 里带 hxid  站内跨笔记引用的主路径 (目录改名也不断链)
    if (title) {
      const tagged = /hxid:(hx-[0-9a-f]+)/i.exec(title);
      if (tagged) return byHxid.get(tagged[1]) ?? null;
    }
    const clean = target.replace(/[#?].*$/, '');
    if (!clean || /[{}]/.test(clean)) return null;
    // 2) 相对路径  兼容手写时漏掉 title 的情况
    if (!clean.startsWith('/')) {
      const base = path.resolve(path.dirname(from.abs), clean);
      const candidates = [
        base,
        base + '.md',
        base + '.mdx',
        path.join(base, 'index.md'),
        path.join(base, 'index.mdx'),
      ];
      for (const candidate of candidates) {
        const key = keyOf.get(candidate);
        if (key) return key;
      }
    }
    return null;
  };

  for (const record of documents.values()) {
    const text = stripCodeBlocks(fs.readFileSync(record.abs, 'utf8'));
    const self = ensure(record.key);
    const seenInternal = new Set();
    const seenExternal = new Set();

    const addExternal = (url, label) => {
      if (seenExternal.has(url)) return;
      seenExternal.add(url);
      self.ext.push([url, cleanLabel(label)]);
    };

    for (const match of text.matchAll(LINK_RE)) {
      const label = match[1];
      const target = match[2].trim();
      if (/^https?:\/\//i.test(target)) {
        addExternal(target, label);
        continue;
      }
      if (target.startsWith('#') || /^mailto:/i.test(target) || ASSET_RE.test(target)) continue;

      const key = resolveTarget(record, target, match[3]);
      if (!key || key === record.key || !documents.has(key)) continue;
      // 跨区引用交给 Docusaurus 会 404, 直接不记
      if (documents.get(key).section !== record.section) continue;
      if (seenInternal.has(key)) continue;
      seenInternal.add(key);
      self.out.push(key);
      ensure(key).in.push(record.key);
    }

    for (const match of text.matchAll(AUTOLINK_RE)) addExternal(match[1], '');
    for (const match of text.matchAll(ANCHOR_RE)) {
      addExternal(match[1], match[2].replace(/<[^>]*>/g, ''));
    }
  }

  return { documents, graph };
}

/**
 * 扫描结果在本进程内缓存, 由 allContentLoaded 显式失效。
 *
 * 为什么必须显式失效: config 只被 require 一次, 开发服务器靠重跑 allContentLoaded 来
 * 响应内容改动。若缓存只认 siteDir, 改完一篇笔记的引用后框里会一直是旧数据。
 */
let scanCache = null;
function getScan(siteDir) {
  if (!scanCache || scanCache.siteDir !== siteDir) {
    scanCache = { siteDir, ...scanContent(siteDir) };
  }
  return scanCache;
}

/**
 * Docusaurus 插件: 扫正文 + 用 Docusaurus 的权威表解析出可点链接, 落盘每篇一份载荷。
 *
 * allContentLoaded 一定晚于所有插件的 contentLoaded, 而 md 要到更晚的 webpack 阶段才编译
 *  所以 remark 那边读这个文件时它必然已经写好了。
 */
export default function noteReferencesPlugin(context, options) {
  const siteDir = context.siteDir ?? SITE_DIR;
  const sections = options?.sections ?? SECTIONS.map((s) => s.dir);
  const outputFile = path.join(siteDir, 'data', PAYLOAD_FILE_NAME);

  return {
    name: 'note-references-plugin',

    getPathsToWatch() {
      return sections.map((dir) => path.join(siteDir, dir, '**/*.{md,mdx}'));
    },

    async allContentLoaded({ allContent }) {
      const permalinks = new Map();
      const titles = new Map();

      const record = (source, permalink, title) => {
        if (typeof source !== 'string' || !source.startsWith(ALIAS_PREFIX) || !permalink) return;
        const key = toPosix(source.slice(ALIAS_PREFIX.length));
        permalinks.set(key, permalink);
        titles.set(key, title ?? '');
      };

      for (const content of Object.values(allContent['docusaurus-plugin-content-docs'] ?? {})) {
        for (const version of content?.loadedVersions ?? []) {
          for (const doc of version.docs ?? []) record(doc.source, doc.permalink, doc.title);
        }
      }
      for (const content of Object.values(allContent['docusaurus-plugin-content-blog'] ?? {})) {
        for (const post of content?.blogPosts ?? []) {
          record(post?.metadata?.source, post?.metadata?.permalink, post?.metadata?.title);
        }
      }

      scanCache = null; // 内容改动后必须重扫, 见 getScan 的说明
      clearRawCache();
      const { documents, graph } = getScan(siteDir);

      const titleFor = (key) => titles.get(key) || documents.get(key)?.title || key;
      const toEdges = (keys) =>
        keys
          .map((key) => {
            const to = permalinks.get(key);
            // 目标没有页面 (草稿 / 被过滤) 时不注入死链
            return to ? [to, titleFor(key)] : null;
          })
          .filter(Boolean);

      /**
       * 键是**源文件相对路径**, 值里带 permalink。
       *
       * 为什么不用 permalink 当键: remark 编译时手上只有绝对文件路径, permalink 要到
       * 更晚才随 metadata 生成  用它当键就查不到。源路径两边都拿得到, 且与 Docusaurus
       * 的 @site/ 别名同构。
       *
       * 收录条件是**同时**通过扫描与页面表两道关:
       *   · 在 `documents` 里  说明它通过了隐私过滤 (符号链接出界 / `_` 前缀 /
       *     `hx_protected`)。只按页面表建会漏这条: 本地开发时私有内容以符号链接存在,
       *     Docusaurus 照样把它读成页面, 于是载荷里出现一批空壳, 私有页面反而渲染出空框。
       *   · 在 `permalinks` 里  说明 Docusaurus 确实为它产页面 (不是草稿、没被 include
       *     排除)。只按扫描建会漏这条: 那些文件注入出去就是指向 404 的框。
       *
       * 满足两条的每篇都有一条记录, 哪怕三条边全空  页面侧据此知道"这是真实页面",
       * 从而在没有引用时渲染空态, 而不是让方框整个消失。
       */
      const payload = {};
      for (const key of documents.keys()) {
        const permalink = permalinks.get(key);
        if (!permalink) continue;
        const entry = graph.get(key) ?? { out: [], in: [], ext: [] };
        payload[key] = {
          permalink,
          out: toEdges(entry.out),
          in: toEdges(entry.in),
          ext: entry.ext.map(([url, label]) => [url, label || '']),
        };
      }

      fs.mkdirSync(path.dirname(outputFile), { recursive: true });
      fs.writeFileSync(outputFile, JSON.stringify(payload), 'utf8');

      // 与 Docusaurus 的页面表对账: 差得太多说明某一边没跟上 (私有过滤或 include 规则
      // 变了), 构建日志里能一眼看出来。
      let out = 0;
      let incoming = 0;
      let ext = 0;
      for (const entry of graph.values()) {
        out += entry.out.length;
        incoming += entry.in.length;
        ext += entry.ext.length;
      }
      const unpublished = [...documents.keys()].filter((key) => !permalinks.has(key)).length;
      console.log(
        '[note-references] 载荷 ' + Object.keys(payload).length + ' 篇' +
        ' (页面表 ' + permalinks.size + ' 篇, 扫描到 ' + documents.size + ' 篇' +
        (unpublished > 0 ? ', 其中 ' + unpublished + ' 篇未出页面' : '') +
        ', 私有/未收录 ' + Math.max(0, permalinks.size - Object.keys(payload).length) + ' 篇)' +
        '; ' + out + ' 条站内引用, ' + incoming + ' 条被引用, ' + ext + ' 条站外来源',
      );
    },
  };
}

/* remark 侧: 每篇文档末尾注入引用框, 该篇载荷以 props 形式带下去 ───────────── */

let payloadCache = null;
function loadPayload(file) {
  try {
    const stat = fs.statSync(file);
    if (payloadCache && payloadCache.file === file && payloadCache.mtimeMs === stat.mtimeMs) {
      return payloadCache.data;
    }
    const data = JSON.parse(fs.readFileSync(file, 'utf8'));
    payloadCache = { file, mtimeMs: stat.mtimeMs, data };
    return data;
  } catch {
    return null;
  }
}

/**
 * 组件名必须在 MDX 组件表里注册 (src/theme/MDXComponents/index.tsx), 不能在这里注入
 * `import` 语句: MDX 3 只认处理前就在文档顶层的 ESM 节点, remark 阶段追加的 import 会被
 * 静默丢弃, 渲染期就是 "Expected component `NoteReferences` to be defined"。
 */
const COMPONENT_NAME = 'NoteReferences';
/** 博客列表用的截断标记, 与 blog 插件的 truncateMarker 默认值一致 */
const TRUNCATE_RE = /<!--\s*truncate\s*-->|\{\/\*\s*truncate\s*\*\/\}/;

/** 原始正文缓存  截断判定要读盘, 同一个文件会在两种变体与两端编译里反复出现 */
const rawCache = new Map();
function readRaw(filePath) {
  const cached = rawCache.get(filePath);
  if (cached) return cached;
  let raw = null;
  try {
    raw = fs.readFileSync(filePath, 'utf8');
  } catch {
    raw = '';
  }
  rawCache.set(filePath, raw);
  return raw;
}

/** allContentLoaded 里清掉, 让开发时的内容改动能反映到截断判定上 */
function clearRawCache() {
  rawCache.clear();
}

/**
 * remark 插件: 在每篇文档末尾注入「引用关系」方框。
 *
 * 常驻语义  只要这篇是 Docusaurus 认得的**页面**, 就一定注入, 哪怕三条边全空。空引用
 * 时框仍在, 显示的是空态。方框的缺席必须只意味着「插件没跑」, 不能意味着「这篇恰好没
 * 引用」, 否则读者分不清是没引用还是坏了。
 *
 * 四种情况下**应当**不注入, 它们都意味着"这不是一个该有框的页面":
 * 载荷里没有这个键 (草稿 / 被 include 排除 / 私有)、截断变体、重复编译、载荷尚未落盘。
 */
export function noteReferencesRemark(options = {}) {
  const siteDir = options.siteDir ?? SITE_DIR;
  const payloadFile = path.join(siteDir, 'data', PAYLOAD_FILE_NAME);

  return (tree, file) => {
    const filePath = typeof file?.path === 'string' ? path.resolve(file.path) : null;
    if (!filePath) return;

    // 同一个 md 会在两端 (server/client) 与多次热编译里反复出现, 防重复注入
    const alreadyInjected = tree.children.some(
      (node) => node.type === 'mdxJsxFlowElement' && node.name === COMPONENT_NAME,
    );
    if (alreadyInjected) return;

    /**
     * 截断变体不注入。
     *
     * 博客列表页用 file.md?truncated=true 编译**同一个文件**的另一份产物, 内容是原文在
     * 截断标记处切开的前半段。不挡住它, 引用框就会出现在列表里每一条摘要下面。
     *
     * 判据用「原文有标记、当前编译的内容里没有」 截断会把标记本身一起切掉, 所以两者
     * 不一致就说明这是截断版。比去问 loader 更省事, 且不依赖 Docusaurus 内部实现。
     */
    const raw = readRaw(filePath);
    if (TRUNCATE_RE.test(raw) && !TRUNCATE_RE.test(String(file?.value ?? ''))) return;

    const payload = loadPayload(payloadFile);
    if (!payload) return;

    // 页面身份由构建期那份表说了算: 表里没有 -> 这篇不产页面 -> 不该有框
    const key = toPosix(path.relative(siteDir, filePath));
    const entry = payload[key];
    if (!entry) return;

    /**
     * 载荷以 **JSON 字符串**作为属性值传下去, 由组件侧 JSON.parse。
     *
     * 为什么不传对象表达式 (`data={{...}}`): mdxJsxAttributeValueExpression 只有同时带上
     * `data.estree` 才会被 MDX 3 序列化成代码  只给 value 字段时它**静默输出空**,
     * 编译结果里就是 `data: ` 后面什么都没有。那需要一个 estree 构造器 (或 acorn) 参与,
     * 而这里传的是构建期自己生成的合法 JSON, 多这一层纯属自找。
     *
     * 字符串路线零依赖、零转义风险 (JSON 里的引号由 MDX 生成标准 JS 字符串转义, 中文原样
     * 保留), 组件侧一次 JSON.parse 的成本对约 367 字节的单页载荷可以忽略。
     */
    tree.children.push({
      type: 'mdxJsxFlowElement',
      name: COMPONENT_NAME,
      attributes: [
        { type: 'mdxJsxAttribute', name: 'data', value: JSON.stringify(entry) },
      ],
      children: [],
    });
  };
}
