# Agent Note: 两侧猫娘贴合 + 博客列表页重做

Status: implemented

Decision-ID: blog-catgirl-sides

## Problem

站点已有两侧猫娘容器 `BlogWithCats` (cv 页、友链页在用), 但它把两张 200px 宽的 PNG (`static/default-img/neko_left.png` 440 KB / `neko_right.png` 435 KB) 用 `position: absolute; top: 100px; left: -190px` 挂在 960px 正文两侧: 随正文滚走, 上下是硬切的直线, 看起来像浮在空中; 用户要求两侧立绘"必须贴合, 不能浮空或有任何破绽", 并要用到博客列表页 (博客只有一页列表, 还是 Docusaurus 默认样式, 带一个与列表重复的"最近文章"侧栏), 之后推广到 /anime/share. 用户给的 34 张半身素材是不透明的满幅图 (深色砖墙 / 灰底), 直接贴必然有矩形边; 声称放进仓库的 `static/img/catgirl/` 经 fs 隧道读不到 (不存在). 约束: 不加依赖, CSS Modules (原组件是全局 css, 保持), 窄屏隐藏, 不压缩正文宽度, 不挡点击, 首屏不能变慢.

## Decision

- 立绘换成生成的两张透明底抠图 (用户在本轮明确允许生成): 左边银发白猫耳、身体朝右; 右边黑发黑猫耳、身体朝左, 都朝向正文. 生成时用纯绿底抠像得到 RGBA, 只做去绿溢色与 0.6px alpha 抗锯齿 (只向内收, 不外扩成光晕), 裁到人物外框, 不缩尺寸不降质: 无损 RGBA PNG, 左 753x1128 (约 800 KB)、右 693x1240 (约 830 KB), 显示最宽 240px, 即 3x 左右密度; 传输体积交给 HTTP 压缩与缓存 (用户 2026-11-02 22:15 明确: 清晰度优先, 不为体积降质). 放 `static/img/catgirl-neko-left.png` / `catgirl-neko-right.png` (fs 隧道不能建目录, `static/img/catgirl/` 不存在, 所以平放在 static/img). 早先的 480 宽有损 webp 版本作废, 待 git rm. 轮廓不加 mask 羽化, 只在底部 14% 叠渐隐, 因为半身图在腰部是硬切边.
- 贴合靠"图本身没有矩形": 透明底 + 底部渐隐, 四周像素 alpha 为 0, 明暗主题共用同一张, 不需要匹配底色; CSS 再叠一层 `mask-image: linear-gradient(to top, transparent, #000 14%)` 兜底, 防缩放后最后几行留边. 暗色主题 `brightness(0.9)`, 白衣不比正文亮.
- 位置: `position: fixed` 贴视口左右下角, 像从页面底部两侧长出来; 宽度 `clamp(160px, (100vw - 正文宽) / 2 - 24px, 240px)`; 视口放不下 "正文宽 + 2 x (160 + 24)" 时整组不渲染 (matchMedia, SSR 与首帧也不渲染, 所以窄屏不请求图片, 首屏不受影响). `pointer-events: none`, `z-index: 1` 在正文 (z 2) 之下; 页面里有立绘时页脚 `position: relative; z-index: 3`, 滚到底时立绘沉到页脚后面, 不会站在页脚文字上.
- `BlogWithCats` 保留原 API (style / children), 新增可选 `contentWidth`; 立绘拆成导出的 `SideCats`, 可单独挂在不用圆角正文块的页面 (/anime/share 用 1200).
- 博客列表页 (`src/theme/BlogListPage/index.tsx`) 不再走 BlogLayout 侧栏, 改为 BlogWithCats 包住的 880px 单栏: 页头 (BLOG 刊头 / 标题 / 简介 / 篇数 / 本页标签云带计数 / 全部标签) + 时间轴式条目 (左日期块 日 + 年.月, 右标题 / 摘要三行截断 / 阅读时长 / 标签) + 原分页器. 条目只用 metadata, 不渲染正文 MDX.

## Alternatives considered

- 什么都不做, 保留原来的 absolute 立绘: 最强理由是 cv / 友链两页一直这样用, 零风险. 否决: 用户明确说当前效果"浮空、有破绽", 而且博客页根本没有立绘.
- 复用现有方案, 只把图换成用户的 34 张素材, 再用 mask-image 四边羽化: 最强理由是素材是用户亲自挑的, 羽化也确实能消掉直角. 否决: 素材是满幅不透明图, 四边羽化后在亮色主题下是一团暗色雾斑, 暗色主题下砖墙纹理仍透出一块"方形区域", 两套主题都要各调一套遮罩且仍看得出是方图; 另外这些文件在仓库里读不到. 换成透明底立绘后不再需要按底色调.
- mix-blend-mode (lighten / multiply) 融进底色: 只对一种主题成立 (lighten 只在暗底、multiply 只在亮底), 而且会把角色本身的颜色一起改掉.
- position: sticky 跟着正文 (原 .neko-sticky 思路): 依赖正文高度, 正文短时立绘停在半空, 正是"浮空".
- 博客列表继续用 BlogPostItems 渲染全文截断: 与首页"最近更新"的卡片语言不一致, 长文列表很高, 也要加载每篇 MDX.

## Consequences

cv 页与友链页 (`src/pages/anime/cv.tsx`, `src/pages/friends/index.tsx`) 随组件一起换成新立绘与 fixed 定位, 正文块样式不变; 旧的 `.neko` / `.neko-left` / `.neko-right` / `.neko-sticky` 类删除, `static/default-img/neko_*.png` 不再被引用, 本轮未删. 立绘出现的最小视口: 博客列表 1248px, cv / 友链 1328px, /anime/share 1568px (1440 宽屏上分享页没有立绘, 因为它的正文本来就是 1200 宽, 硬塞会压缩正文). 贴合效果的验证截图 `.agents/research/catgirl-fit-dark.png` / `-light.png` 是按 CSS 规则用 PIL 合成的模拟图 (1440x900, 沙箱没有浏览器能打开本机站点), 立绘外框一圈像素与页面底色的最大差值为 0 到 1 (8 bit); 真实浏览器截图待本机复核. 用户的 34 张半身素材未使用. `src/components/BlogWithCats/BlogWithCats.css` 是非 AST 资源, 需人工核对.

## Code

- `src/components/BlogWithCats/index.tsx`
- `src/theme/BlogListPage/index.tsx`
- `src/pages/anime/share.tsx`
