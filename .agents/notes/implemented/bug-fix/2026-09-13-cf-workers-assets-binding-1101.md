# CF 部署线上大面积 Error 1101: [assets] 缺少 ASSETS 绑定

- 日期: 2026-09-13
- 状态: implemented
- 类别: bug-fix
- 影响: `wrangler.toml`(新增) / `worker.js`(从 workflow 内联挪出) /
  `.github/workflows/documentation.yaml` / `scripts/check-cf-worker-config.mjs`(新增) /
  `scripts/split-search-index.mjs` / `scripts/optimize-large-assets.mjs`

## Problem

线上 <https://km.woa.qzz.io> 部分 URL 返回 **Cloudflare Error 1101 (Worker threw exception)**,
但本地 `npm run start` 一切正常。实际受影响的是「需要 Worker 脚本兜底」的 URL:

| URL | 现象 |
| --- | --- |
| `/docs/`、`/docs` | 1101 (目录页需要脚本回落) |
| `/search-index.json` | 1101 (脚本要从 chunk 合并) |
| `/favicon.ico`、任意不存在路径 | 1101 (404 兜底要跑脚本) |
| `/docs/关于`、`/blog/`、`/anime/`、`/`、`/img/logo.png` | 200 正常 |

所以是**一半页面正常一半炸**, 不像"整站挂了"。

## Root cause

Workflow 里用 heredoc 现场生成 `wrangler.toml`, 写出来的 `[assets]` 段只有
`directory`, **没有 `binding`**:

```toml
[assets]
directory = "./build"     # ← 少了 binding = "ASSETS"
```

Wrangler 只有在 `[assets]` 里显式声明 `binding` 时才会往 `env` 注入这个绑定。
于是 `worker.js` 里的 `env.ASSETS` 是 `undefined`, 一执行到
`env.ASSETS.fetch()` 就抛:

```
TypeError: Cannot read properties of undefined (reading 'fetch')
```

**为什么本地正常**: 本地跑的是 `docusaurus start` (dev server), 根本不经过 Worker;
就算起 `wrangler dev`, 只要请求命中了 `build/` 里的真实文件, 静态资源路由会**直接返回文件、
完全不执行脚本**, 所以也看不到异常。只有"没有对应静态文件、必须由脚本兜底"的请求才会踩雷 ——
这正是上表里的目录页 / 404 / search-index。

`deploy` 本身不会因为缺 binding 报错, 所以这个坑一直静默存在。

### 复现 (已实测)

```bash
# 同一份 worker.js, 只改 wrangler.toml
# 无 binding:
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8791/search-index.json   # 500
#   日志: TypeError: Cannot read properties of undefined (reading 'fetch')
# 有 binding:
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8793/search-index.json   # 404 (逻辑正常走到查文件)
```

线上 `/search-index.json` 的 500 响应体也印证了这点: 响应 `content-length: 17`,
正好是 `error code: 1101` (Cloudflare 的通用异常文案), 而不是 Worker 自己返回的报错 —— 
说明异常是在脚本里抛的。

### 顺带发现的两个隐患

1. **heredoc 缩进污染**: workflow 里 `cat > worker.js << 'WORKEREOF'` 的收尾标记带缩进,
   shell 会把缩进原样写进文件 (实测每行多 10 个空格)。这份 worker.js 恰好语法上仍然合法,
   所以没炸; 但这是典型 "哪天改一下就突然挂" 的地雷。
   (对比之下 `[assets]` 段反而没被污染 —— 因为 `cat` 是同一个还原函数, 但
   `[assets]` 段在 YAML 里本来就是零缩进写的。)
2. **ffmpeg `-vsync` 已被移除**: `optimize-large-assets.mjs` 里 GIF→WebP 用的是
   `-vsync 0`, 新版 ffmpeg (本机 n9.0.1) 直接报 `Unrecognized option 'vsync'`。
   配合 workflow 的 `--remove-unsupported --remove-remaining`, 结果是
   **超大 GIF 全部被转换失败后直接删除**, 而 worker.js 里那条 GIF→WebP 回退逻辑
   因为没有 .webp 文件可回落, 也就一直没生效。

## Decision

1. **`wrangler.toml` 与 `worker.js` 提到仓库根目录维护**, 不再用 heredoc 生成。
   配置是基础设施, 就该能被 review、能被 lint、能被版本管理。
2. **`[assets]` 必须显式写 `binding = "ASSETS"`**, 并在文件里用注释说明原因。
3. **新增 `scripts/check-cf-worker-config.mjs`**: 校验 `main` / `directory` / `binding`
   三项, 缺任一项直接 `exit 1`。CI 在部署前跑, 让同类问题**在构建阶段就失败**,
   而不是等到线上 1101。
4. **worker.js 启动时显式取一次 `env.ASSETS`**, 缺失时返回可读的 500 文案
   而不是抛 `TypeError` —— 下次万一再配错, 一眼就能看出问题。
5. **worker.js 补 404 兜底**: 资源未命中时回落 `/404.html` (用同一份响应体改回 404 状态码),
   避免 Cloudflare 默认那个裸 404。
6. **修正 ffmpeg 参数**: `-vsync 0` → `-fps_mode passthrough` (ffmpeg 5.1+ 的正式替代)。
7. **`split-search-index.mjs` 默认不再删源文件**, 删除改成显式 `--delete-source`;
   CI 显式传该 flag。本地跑脚本不会再把自己的 `build/search-index.json` 删掉。

## Alternatives considered

- **只在 workflow 里给 `[assets]` 补一行 binding**: 最小改动, 但 heredoc 生成配置、
  缩进污染、无法 lint 这些结构性问题一个都没解决, 下次还会以别的形式炸。否决。
- **干脆不要 Worker 脚本, 纯静态资源部署**: 那 `/search-index.json` (28MB, 超过 25MiB
  单文件上限) 就没法服务, 站内搜索直接废掉 —— 而拆 chunk 合并本来就依赖脚本。否决。
- **用 `not_found_handling = "404-page"` 代替自己写 404 兜底**: 这个选项是有效的,
  但它只在**没有 `main` 脚本**时才接管; 有脚本时它不会替我们处理 GIF→WebP 回退那条逻辑链,
  两套兜底混着用更容易搞混。当前保留脚本兜底, 只在注释里说明。
- **不再拆 chunk, 改用压缩后的搜索索引**: 治本方向但属于另一个议题 (涉及搜索插件配置),
  本次先保证线上恢复。

## 跟进: 推送后 CI 挂在更前面的一步 (package.json 漏声明依赖)

推上去之后 workflow 确实跑起来了, 但**没走到 CF 那几步**就失败了:

```
[ERROR] Error: Docusaurus could not load module at path ".../docusaurus.config.ts"
Cause: Cannot find module 'feed'
Require stack: .../plugins/docs-rss-plugin.mjs
```

这是**另一个独立问题**, 跟本次 CF 修复无关, 但同样卡住了部署。

### 原因

`docusaurus.config.ts` 加载 `plugins/docs-rss-plugin.mjs`, 该插件
`import { Feed } from 'feed'`; `tag-index-plugin.mjs` / `src/utils/tags/pinyin.ts` 又分别用到
`gray-matter` / `pinyin-pro`, 但 **这三个包都没写进 `package.json`**:

```
lock-only (在 lockfile 里有, package.json 里没有): feed, gray-matter, pinyin-pro
```

历史上它们能装上, 是因为它们曾经是 root 依赖且 lockfile 里留着记录;
但 `package.json` 里没有 → CI 走 `npm ci` 时会按 `package.json` 重建依赖树,
把这三个从 node_modules 顶层剔掉, 只剩 `@docusaurus/plugin-content-blog` 自带的
`feed@4.2.2` (嵌套在子目录里, 根级 `require('feed')` 解析不到) → 构建报
`Cannot find module 'feed'`。

> 注: 本机一直没暴露这个问题, 是因为本地 `node_modules` 是增量长出来的,
> 里面还留着当年的 `feed@6.0.0` / `gray-matter@4.0.3` / `pinyin-pro@3.29.4` ——
> `npm ls` 里 `feed@6.0.0` 甚至被标成了 `extraneous`。典型的"本地能跑, CI 炸"。

### 修法

把三个包补进 `package.json` 的 `dependencies` (`feed ^6.0.0` / `gray-matter ^4.0.3` /
`pinyin-pro ^3.29.4`), 与 lockfile 的 root entry 对齐。对齐后
`pkg-only` 与 `lock-only` **双双为空**, `npm ci` 不需要改 lockfile 也能装齐:

```bash
npm ci --dry-run | grep '^add feed'
#   add feed 6.0.0     ← 根级 feed@6 回来了 (旧计划只有嵌套的 feed 4.2.2)
#   add feed 4.2.2
```

验证: 修好后本地 `npm run build` 成功 `[SUCCESS] Generated static files in "build"`。

## Evidence

- 线上实测 (2026-09-13): `/docs/关于` 200 / `/blog/` 200 / `/anime/` 200 / `/img/logo.png` 200;
  `/docs/` `/docs` `/search-index.json` `/favicon.ico` `/nonexistent-abc/` 全部 500 + 1101。
  `/search-index-manifest.json` 与 chunk 文件本身 200 (说明资源已上传, 只是脚本跑不了)。
- 本地复现: 同一 worker.js, `wrangler dev` 在无 binding 时报
  `TypeError: Cannot read properties of undefined (reading 'fetch')` (500);
  补上 `binding = "ASSETS"` 后同样请求变成正常的 404/200。
- 修复后逻辑回归 (fixture: 2 个 chunk + 404.html + 一张 webp):
  `/search-index.json` 200 `{"part":0}AB` (chunk 已按序合并);
  `/missing-page/` 404 + 自定义 404 页面 (而非 Cloudflare 裸 404);
  `/img/logo.gif` 200 `image/webp` (GIF→WebP 回退生效)。
- 配置校验器负向测试: 去掉 `binding` / 去掉 `directory` / 还原历史那份线上配置, 全部 `exit 1`。
