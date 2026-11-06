# Agent Note: kyara 角色配图: 目录约定 + 路由插件 + 导入接口

Status: implemented

Decision-ID: anime-kyara-gallery

## Problem
站长想给喜欢的角色配一组图, 页面 `/anime/kyara/{name}` 按顺序展示, 并且能从 pixiv / x.com 链接一键导入, 线上访客也能在自己浏览器里临时加图. Docusaurus 的 `src/pages` 不支持动态段, 角色又是随目录增减的, 不能一个角色手写一个页面文件. 图片要进仓库 (可 diff, 换机器不丢), 顺序和作者署名要能从文件本身看出来, 不另维护一份清单.

## Decision
- 目录即数据: `static/img/kyara/{name}/NNN-标题@作者.ext`, 正则 `^(\d{3,})-(.+?)(?:@([^@]+))?\.(png|jpe?g|webp|gif|avif)$`, 按最后一个 `@` 拆作者, 序号数值升序、同序号按文件名; 不合规文件跳过并 warn. 可选 `kyara.json` 给 displayName / work / bangumiCharacterId / accent / cover. 约定写在 `static/img/kyara/README.md`.
- 类型契约冻结在 `src/utils/kyara/types.ts` (计划 1.6 节, 也是该目录的代表文件; 同目录兄弟文件同属本决策), 页面 UI (路 4) 只读. 命名规则 `filename.ts`, 解析器 `resolvers.ts` (pixiv ajax + Referer / fxtwitter + `name=orig` / 直链), 本地模式 `localStore.ts` (IndexedDB `hx-kyara` + `useKyaraLocal`).
- `filename.ts` 与 `resolvers.ts` 只有 `import type`, 插件和 CLI 在 Node 里直接 import 这两个 `.ts` (Node 原生 type stripping, Docusaurus 加载配置时走 jiti), 前后端一套规则.
- 路由: `plugins/kyara-gallery-plugin.mjs` 在 `contentLoaded` 扫描目录, `createData` + `addRoute` 注册 `/anime/kyara` 与每个 `/anime/kyara/{encodeURIComponent(name)}`; `getPathsToWatch` 监听目录. 页面组件 (`src/components/anime/kyara/KyaraIndexPage|KyaraPage`) 不存在时不注册路由, 只 warn, 免得路 4 落盘前整站编译失败.
- 运行时清单: `postBuild` 写 `kyara/index.json` 与 `kyara/data/{name}.json`, dev 下同路径由中间件现算. 本地模式用它拿仓库最大序号, 本地条目的序号 = 仓库最大序号 + 本地位次, 不落库, 所以永远排在仓库图片之后, 导出 zip / 写进仓库时直接合规.
- dev-only 写入通道, 防跨站规则照搬 anime-overlay: 只认本机 Host + `X-HX-Kyara: 1` 头 (触发 CORS 预检), 只在 client 侧 development webpack 配置里经 `devServer.onListening` 挂载. `POST import` (服务端解析 + 下载, pixiv 只能走这里或 CLI), `PUT file/{filename}` (文件名必须合规, 内容按魔数校验是图片, 已存在 409), `POST reorder` (order 必须恰为全部文件; 先改临时名再改正式名). 同一角色的写操作串行, 写文件 tmp + rename.
- CLI `scripts/kyara-import.mjs <name> <url...>` 复用插件导出的 `importKyaraUrl`; `--selftest` 跑冻结的命名与链接匹配用例, 不联网.
- 本地 zip 用内置的 STORE 写法 (图片本身已压缩), 不引新依赖.

## Alternatives considered
- 什么都不做 / 复用现有: 最强理由是 `static/img/catgirl/manifest.json` 已有一套立绘清单, 照抄就不用写插件. 否决: 那套靠手写 manifest 维护顺序与作者, 用户明确要 "放进目录按文件名顺序渲染", 而且没有链接导入与动态路由.
- 每个角色一个 `src/pages/anime/kyara/{name}.tsx`: 最强理由是零插件. 否决: 加角色就要写代码, 中日文文件名做路由也别扭.
- 用 query 参数 `/anime/kyara?name=xxx` 单页客户端渲染: 最强理由是不用 addRoute. 否决: SSG 产不出每个角色的静态 HTML, 分享链接没有预渲染内容.
- 前端直接请求 pixiv: 否决, pixiv 原图校验 Referer 且无 CORS, 浏览器里拿不到; 改为 Node 侧 (dev 通道 / CLI) 下载, 本地模式里给出提示.
- 引 jszip / idb 依赖: 最强理由是少写代码. 否决: 只需要 STORE zip 和几个 IndexedDB 调用, 新依赖不值.
- `setupMiddlewares` 挂中间件: 同 anime-overlay 的结论, webpack-merge 会把栈拼两遍, 用 `onListening`.

## Consequences
- 验收: `node scripts/kyara-import.mjs --selftest` 退出码 0; 放一张 `static/img/kyara/<name>/001-x@y.png` 后 dev 下 `/anime/kyara/<name>` 出现 (需路 4 组件已落盘); 生产构建后 `grep -r "__hx/kyara\|X-HX-Kyara" build/` 应只命中 `localStore.ts` 打进去的 dev 分支常量以外为空 (写入通道服务端不进包).
- 依赖 Node ≥ 22.18 / 23.6 的原生 type stripping 跑 CLI; Docusaurus 侧由 jiti 处理 `.ts` import.
- 角色目录名会进 URL 与运行时清单文件名, 不许含 `/ \`、控制字符或以点开头.
- reorder 会重命名仓库文件, git 里表现为 rename; 序号宽度至少 3 位, 超过 999 张自动加宽.

## Code

- `plugins/kyara-gallery-plugin.mjs`
- `scripts/kyara-import.mjs`
- `src/utils/kyara/types.ts`
