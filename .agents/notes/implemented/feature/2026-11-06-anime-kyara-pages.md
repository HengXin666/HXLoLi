# Agent Note: kyara 角色画集页 UI (/anime/kyara, /anime/kyara/{name}, /anime/kyara/local), 产品页式, 与 susume 同一设计语言

Status: implemented

Decision-ID: anime-kyara-pages

## Code

- `src/components/anime/kyara/KyaraPage.tsx`
- `src/components/anime/kyara/dev/DevRepoTools.tsx`

## Problem

1. 番剧区需要一套按角色展示配图的页面: 图放进 `static/img/kyara/{name}/001-标题@作者.png` 就按序号渲染; 访客也能在浏览器本地给角色配图 (新建角色、拖图、粘贴、从 pixiv / x.com 链接导入、排序、导出 zip), 部署版与 dev 行为一致, 本机 dev 下额外能写进仓库.
2. 页面要像产品页: 有主视觉与轮换图片, 按番剧分组展示角色, 主题色随角色变化, UI 与 UX 连贯; 沿用 HX おすすめ Glass (`src/components/anime/ui/DESIGN.md`) 的令牌与动效, 不另起一套.
3. 画面上只描述当下, 不出现历史 / 变更类文案; 目录约定之类给站长看的说明不出现在线上.
4. 数据层 (扫描、路由、globalData、IndexedDB、dev 写入通道) 归 `src/utils/kyara/` 与插件 (另一篇 note `anime-kyara-gallery`), 本决策只做页面并按其契约引用.

## Decision

1. `src/components/anime/kyara/` (代表文件 `KyaraPage.tsx`, 同目录兄弟都是本决策的代码端):
   - `KyaraIndexPage.tsx`: 页头 (标题 + 本地模式开关) → `Spotlight` 主视觉 → 四格统计 (角色 / 配图 / 画师 / 作品) → 作品筛选 `HxPills` + 按番剧分组的角色卡 (分组锚点 `work-{subjectId}`, 详情页入口带 hash 进来时预选). 本地模式开启时并入本地角色并在末尾放「＋ 新角色」. 空态面向访客, 只引导开启本地模式.
   - `KyaraPage.tsx`: 路由组件 + `KyaraView` (与本地页共用). 首屏左 7 / 右 5: 左 `ImageStage` (封面打头的主图交叉淡入 + 胶片条, 点开灯箱), 右侧 kicker / 大标题 / 原名 / 「出自《作品》」(有条目 id 时链到番剧详情) / 三格规格 / 画师 chip (点了过滤画廊) / 操作 (看全部、Bangumi 角色页、本地模式开关); 下面依次是画廊、同作品角色横滑卡、上一位 / 下一位. dev 写盘后先用 dev 清单的最新内容, 热重扫后回到 props.
   - `KyaraLocalPage.tsx`: `/anime/kyara/local?name=`, 主体用 `BrowserOnly` 渲染 (静态构建里没有本地角色, 避免水合不一致); 同名仓库角色存在时 `Redirect`; 本机 dev 下刚建到仓库还没路由的角色先读 dev 清单渲染并轮询, 路由出现后跳过去.
   - `Spotlight.tsx`: `useRotation` (6s 自动轮换; 悬停 / 聚焦 / 标签页隐藏 / reduced-motion 暂停) + `Spotlight` (索引主视觉, 主题色随当前角色) + `ImageStage` (角色页主图).
   - `KyaraCard.tsx`: 3:4 角色卡, 悬停或聚焦时在前 4 张图之间轮换 (那时才加载后几张); 同文件导出 `LocalSwitch`.
   - `Gallery.tsx`: 等高行瀑布 (按序号左到右, w/h 预留比例) + 画师过滤; `Lightbox.tsx` 灯箱与小工具不变.
   - `LocalDrawer.tsx`: 本地模式操作台 = 吸底工具栏 (张数 / 添加 / 从链接导入 / 导出 zip / dev 写进仓库 / 管理) + 侧抽屉 (本地图排序删除、纯本地角色的资料与删除、dev 下「仓库」页签); 全局粘贴与拖放入库.
   - `ImportDialog.tsx`: 多行链接解析导入; pixiv 在浏览器拿不到时线上给出替代做法, dev 下交给服务端导入. `NewKyaraDialog.tsx`: 新建画集 (显示名 → 目录名默认取显示名去掉非法字符 → 作品), 线上建成本地角色, dev 可选直接建到仓库; 目录名撞车时同一 Bangumi 角色直接打开, 否则提示换名. 右键菜单「以此新建」复用它.
   - `devLoader.js` + `devLoader.d.ts` + `devChunks.ts`: dev 专属模块只经 `process.env.NODE_ENV === 'development'` 常量分支 `import()`, 生产构建判死分支; `useKyaraDev()` 只在本机 host 返回模块.
   - `Kyara.module.css`: 颜色只用 `--hx-*` 与 `--kyara-accent`, 派生用 `color-mix`; 断点 ≤996 / ≤640; reduced-motion 下全部瞬时, 不显示轮换进度线.
2. `src/components/anime/kyara/dev/` (代表文件 `DevRepoTools.tsx`): 只在 dev 包里. `devClient.ts` 封装写入通道 (import / putFile / deleteFile / reorder / putMeta / 读 dev 清单) 与「本地图写进仓库」; `DevRepoTools.tsx` 是仓库管理 (编辑 kyara.json、拖拽重排序号、设封面、两次确认删除); `KyaraGuide.tsx` 是目录约定说明, 只在 dev 下出现.
3. 目录名默认取中文名 (`name_cn`, 没有时原名): 读者看中文, 地址与目录一眼可读, 与文件名里的中文标题一致; 匹配靠 Bangumi 角色 id, 不依赖目录名.

## Alternatives considered

- 什么都不做: 最强理由是零风险. 否决: 部署版本地模式不可用 (开关与读库条件对不上, 本地角色刷新即消失), 页面观感也不满足产品页要求.
- 复用现有 `StageHero` 页头: 最强理由是与 susume 结构一致. 否决: 它是纯文字页头, 承载不了主视觉轮换与产品式两栏首屏; 面包屑、统计等语汇照搬它的样式.
- 首屏用 JS masonry 或第三方轮播 (Swiper 等): 最强理由是功能齐. 否决: 新增依赖, 且无法与灯箱共享 framer `layoutId`; framer `AnimatePresence` 已够用.
- 本地操作台常驻在首屏: 最强理由是一眼可见. 否决: 挤占产品式首屏, 访客视图与本地视图首屏不一致; 改为吸底工具栏 + 抽屉.

## Consequences

- 页面效果由用户在 1440 / 1024 / 390 三档宽度、亮暗各看一次; reduced-motion 下不自动轮换.
- `src/components/anime/kyara/Kyara.module.css` 是非 AST 资源, 改动需人工核对.
- 生产构建产物里 `grep -rE "__hx/kyara|X-HX-Kyara" build/` 为空; 本地模式代码 (IndexedDB) 在产物中, 这是预期.
