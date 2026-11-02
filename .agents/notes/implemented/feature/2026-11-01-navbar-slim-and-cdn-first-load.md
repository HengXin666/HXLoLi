# Agent Note: CDN 选择器改为一行状态指示 + FloatingLayer 面板, 首屏测速与读数改为同一引擎

Status: implemented

Decision-ID: navbar-slim-and-cdn-first-load

## Problem

范围只有顶部栏的 CDN 选择器 (`custom-cdnSelector`), 导航栏结构、logo、其它 item 不动.

CDN 选择器与站点风格不符: 触发器是字符 `⚡` 加文字 `CDN` / `123ms`, 面板是整块内联样式, `🇨🇳` `🌐` `✓` `⏳` `🔄` 都是字符当图标, 面板自己 absolute 定位, 和仓库已有的 common/Tooltip 弹层不是一套.

Bug: 直接进站时按钮一直是灰色 `CDN`, 点开面板后才出现延迟. 根因是两个引擎各有一张延迟表 (`hx-cdn-forge` 的 `latencyResults` 是实例字段): `ensureInit()` 只在 mainEngine 上测速, `selectNodeAll()` 只同步节点 id; 按钮和面板却读 musicEngine 的表, 永远是空的. 打开面板时 `CDNNodePanel` 的 effect 发现没数据才在 musicEngine 上补测, 所以 "点一下才生效". localStorage 已有节点时 `shouldAutoTest = false`, 首屏完全不测, 不点就永远没有读数. 另外 `cf-worker` 只在子仓库引擎里, 选它之后 mainEngine 的 `selectNode` 静默失败.

不是 useEffect 依赖问题, 也不是 SSR 不一致: 按钮初始值在两端都是 `CDN`.

## Decision

触发器: 一行, `FaSignal` 图标 (颜色按当前节点延迟分档: 绿 / 黄 / 橙 / 红, 测速中闪烁) + 等宽小号延迟数字 (测速中显示 `···`). 无边框, 悬停只加浅底.

面板: 复用 `src/components/common/Tooltip/index.tsx` 导出的 `FloatingLayer` (与 `common/Tooltip/Select` 同一套 fixed 定位、上下翻转、视口夹取、淡入外壳), 下方优先、无箭头、可交互; 点外部 (pointerdown 捕获) 与 Esc 关闭, 同 Select 口径. 行的悬停 / 选中色与 `Select.module.css` 的选项一致. 字符图标换 SVG: `✓` → `FaCheck`, `🇨🇳` → `FaEarthAsia` (国内节点), `🌐` → `FaGlobe` (全球节点), `🔄` / `⏳` → `FaRotateRight` (测速中旋转). 内联样式移到 CSS module.

修复: 测速与读数统一用节点全集的 probeEngine (musicEngine, 回退 mainEngine). `linkJsDelivr.ts` 自己持有延迟表和一个不可变快照, 经 `onCdnReady` / `getCdnSnapshot` / `getCdnServerSnapshot` 给 `useSyncExternalStore`. 模块加载即在浏览器端初始化: 新访客全量测速并选最优; 已有持久化节点则不改选择, 只后台探测当前节点一次. 面板打开不再触发测速. `selectNodeAll` 对缺少该节点的引擎改选其自身测得最快的节点, 再退到默认节点.

## Alternatives considered

- 直接用 `common/Tooltip/Select`: 它的触发器是带边框和下拉箭头的 combobox, 放进导航栏太重, 也放不下 "重新测速"; 所以只复用它底下的 FloatingLayer.
- 测速后把 mainEngine 的延迟表拷进 musicEngine: 一行补丁. 否决是因为要写库的私有字段, 且 mainEngine 没有 cf-worker, 表仍不完整.
- 在 `CDNNavbarButton` 的 effect 里首屏主动调用 `runSpeedTest`: 能让读数出来. 否决是因为每个挂载点各测一次, 状态仍分散在组件里.

## Consequences

新访客首屏会并发发出每个节点一次 HEAD 探测 (与原 mainEngine 测速同口径); 回访只发一次. 水合首帧用固定 idle 快照, 不产生 hydration 警告. `CDNNodeSelector` 的默认导出从旧 `CDNNodePanel` 改为 `CDNNavbarButton`, 仓库内唯一引用方是 `src/theme/NavbarItem/ComponentTypes.tsx` (用的是具名导出, 未改). 新增 `src/components/CDNNodeSelector/styles.module.css` (非 AST 资源, 需人工核对). 无新增依赖.

## 2026-11-02 补充: 每次冷加载自动优选 + 定时重测 + 尊重手选

问题: 回访时 `ensureInit` 直接沿用 localStorage 里的节点, 只探测这一个, 网络变了也不换. 用户要求每次加载都测速并自动选当前最优, 站内跳转不重测, 定时再优选一轮.

改为: 冷加载先把持久化节点 (没有则默认节点) 应用到所有引擎, `ensureInit` 不等测速就完成, 资源照常加载; 同时后台全量测速, 全部节点测完才算最优. localStorage 新增 `hxloli-cdn-node-source` (`auto` / `manual`): 来源 auto 时测完切到最优; manual (面板手选) 时保留手选, 只把最优记入快照 `bestId`, 面板显示 "最优" 标记和 "当前节点较慢, 切换到 X" 提示. 面板 "重新测速" 视为用户要求重选, 测完切到最优并把来源改回 auto. 定时: 每 60 秒检查一次, 前台且距上次测速满 15 分钟就重测; `visibilitychange` 回到前台时同样检查; 后台标签页不测. 定时器与监听挂在 window 上, HMR 重跑模块时先清旧的, 不会叠加. SPA 跳转不重新执行模块, 不重测. 旧版本没存来源, 按 auto 处理: 少数以前手选过的用户会被自动优选覆盖一次, 之后再手选即可保留.

否决: 什么都不做 (最强理由: 回访首屏零额外请求) — 否决, 因为节点会长期停在已变慢的那个上; 复用现有 `retestAllNodes` 直接在回访时 await (最强理由: 一行改动) — 否决, 因为首屏资源会等最慢节点 5 秒超时, 且会覆盖手选. 代价: 回访首屏每个节点多一次 HEAD 探测, 每 15 分钟一轮.

## Code

- `src/components/CDNNodeSelector/index.tsx`
- `src/utils/cdn/linkJsDelivr.ts`
