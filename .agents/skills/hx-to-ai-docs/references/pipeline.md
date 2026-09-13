# 产出物清单与验证闸门

## 一次沉淀的产出物

以 `ai-docs/002-知识沉淀/001-现代C++/001-日常探索/001-HXLibs编写串行协程调度器/` 为例:

| 产出 | 必需 | 说明 |
|---|---|---|
| `index.md` | ✅ | 由 `makeDoc.py` 初始化, 再填充正文 |
| `.hx-mitemite.md` | 视情况 | 有待人类回答/拍板的问题时存在 (见 hx-docs-grill) |
| `tag.json` | 可选 | 自定义侧边栏标签与图标: `{ "tags": ["高性能"], "icon": "ISO_C++_Logo.svg" }`; 图标文件放 `static/icons/` |
| `xxx-ppt.html` | 可选 | `#ppt` 演示页侧车, 必须与 `index.md` 同目录 (见 hx-docs-ppt) |
| `xxx.html` (archify) | 可选 | 图表产物, 同样与 `index.md` 同目录 |
| 图片 (截图等) | 可选 | 与 `index.md` 同目录, 正文用 `![alt ##w80%##](x.jpg)` |

目录前缀约定: 分类与文章目录都用 `NNN-` 三位前缀. 新建**分类**目录 (如 `007-新分类`) 必须人类同意.

## 收尾验证闸门 (逐条确认过再交付)

1. `makeDoc.py` 真的执行过; 若没执行, 回复里写明跳过原因.
2. frontmatter 的 `hxid`/`title`/`created_at`/`model`/`skill`/`authors`/`tags` 齐备, `skill` 是 YAML 列表, 且**如实**记录了本次用到的技能. `hxid` 由 `makeDoc.py` 创建时生成; 存量笔记缺 ID 时跑 `hx_docs_id.py assign --write` (见 `hx-docs-organize`).
3. `format_cn_punct.py --check <index.md>` 通过 (先 `--diff` 看改动是否符合预期再原地归一化).
4. 正文不含过程痕迹: grep 一遍本地路径、脚本命令、`TODO`、`AI 辅助`、`review` 之类字样, 有则移走.
5. 新增/改名/移动目录后跑过 `node scripts/generateAiDocsSidebar.js`, 且 `sidebarsAiDocs.ts` 里能搜到新笔记的 id (如 `知识沉淀/现代C++/日常探索/<标题>/index`).
6. 跨文章引用用 `hxid:` 形式, 移动过目录后跑 `hx_docs_id.py resolve --write`, 再 `hx_docs_id.py check` 确认无陈旧路径; 同目录相对链接指向真实存在的文件.
7. 若加了 `#ppt` 链接: 侧车与 md 同目录、无外部 CDN、构建或起站后渲染正常.

回复里要写明: 执行过哪些命令、跳过了哪些、为什么.

## 本地开发联动

人类用 `run.sh` 启动站点时, 会同时起 `scripts/dev-edit-server.mjs` (localhost:3310, 仅本地). 它给 ai-docs 页面注入仅本地可见的 UI:

- **本地工具条**: 「在 VS Code 中打开」跳到该 `index.md` 的标题行 (KDE Wayland 走 KWin, X11 走 xdotool).
- **选中正文右键**: 「跳转到 VS Code」精确定位到源码行列; 无选区时放行浏览器原生菜单.
- **答题卡可折叠区** (`.hx-mitemite.md`, 每篇目录最多一个): 默认折叠, 展开后每问独立框体, 可点「编辑」切原始 textarea 并写回本地. **这是审核事项的唯一落盘处** (读者看不到).
- 组件运行时探测 `localhost:3310/health` 才渲染, 生产构建时为 null, 产物不含任何本地编辑 UI.

VS Code 改完 → Docusaurus HMR 即时重渲染. 交互路径: 审核要点走答题卡, 需要改源码时用「选中右键 → 跳转 VS Code」.
