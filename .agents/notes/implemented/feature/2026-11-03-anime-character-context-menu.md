# Agent Note: 番剧详情角色立绘右键菜单: 复制生图提示词 / 下载角色图

Status: implemented

Decision-ID: anime-character-context-menu

## Code

- `src/utils/anime/characterPrompt.ts`
- `src/components/anime/CharacterMenu/CharacterContextMenu.tsx`
- `src/pages/anime/details.tsx`

## Problem

番剧详情页的角色图是 `<a class=charImg>` 上的 background-image, 浏览器原生右键菜单里没有"另存图片", 访客拿不到角色图. 站长本人给两侧探头立绘生图时, 要手动拼番剧封面、番剧名、角色名、角色图 URL 与构图要求, 每次都重抄一遍, 左右两个朝向的方向词还容易写反. 要求: 自绘菜单 (不是原生菜单), 键盘可操作, 点外部关闭, 触屏长按可触发; 开发者用途的探头提示词只在本地模式出现, 网站模式只给纯角色那条与拿图; 提示词模板集中一处; 深浅两套主题; 不用原生 alert.

## Decision

- 模板: `src/utils/anime/characterPrompt.ts` 是纯函数模块 (不 import 任何东西), 导出 `buildPeekPrompt(input, side)`、`buildCharacterPrompt(input)`、`buildAllPrompts(input) -> { left, right, character }`, 以及 `PEEK_POSE` / `PEEK_NEGATIVE` / `CHARACTER_NEGATIVE` / `PROMPT_SEPARATOR`. 探头两份在正文里逐行写出番剧名、角色名、角色图片 URL、番剧封面 URL (参考图) 与姿势描述, 构图写明"从上往下俯探身子、半身出画、身体朝内、竖直裁切线在内侧", 左侧朝右 / 裁切线在右边缘, 右侧朝左 / 裁切线在左边缘. 末尾附负面清单, 抄自 `.agents/research/catgirl-genprompt.md` 第 6 节. 纯角色那份不含探头构图, 只要透明底全身立绘.
- 菜单: `src/components/anime/CharacterMenu/CharacterContextMenu.tsx` 导出 `CharacterMenuScope` (一个详情页一个菜单实例、一次本地模式判定) 与 `useCharacterMenuTrigger(target)` (挂在角色图上的 contextmenu / 长按 / click 拦截与类名). 定位复用 `common/Tooltip` 导出的 `FloatingLayer`, 指针位置用一个 0x0 的 fixed 点当锚点; 键盘菜单键 / Shift+F10 触发时贴着角色图下方. WAI-ARIA menu 模式: 焦点进第一项, 上下 / Home / End 移动, Enter / 空格执行, Esc 与 Tab 关闭并把焦点还给角色图; 点外部、滚动、resize 关闭. 触屏 500ms 长按 (移动超过 10px 取消), 长按后的那次 click 被拦下, 不会跳去 bgm.tv; `-webkit-touch-callout: none` 关掉 iOS 的系统长按菜单. 结果用页内 toast (role=status) 提示.
- 菜单项: 本地模式 = 复制探头立绘提示词 (左右两份合一次复制) / 复制角色独立立绘提示词 / 下载角色图 / 复制角色图片链接 / 打开 Bangumi 原图. 网站模式去掉第一项, 其余照旧.
- 本地模式判定 = `isLocalDev()` (`src/utils/cdn/linkJsDelivr.ts`) `||` `useDevService().ready` (`src/components/DevTools/useDevService.ts`), 两套都是仓里已有的; 不新增第三种. `isLocalDev()` 在 effect 里读, SSR 与首帧按网站模式.
- 下载: CDN 允许跨域, fetch 成 blob 后用 `<a download>` 存成 `<角色名>.jpg`; 失败时新开标签页打开图片.
- 视觉: 深色沿用 Tooltip 的半透明深色卡片, 选项尺寸 / 圆角 / 粉色 14% 高亮与 `common/Tooltip/Select` 一致; 亮色主题在本组件的 css module 里换成白底浅描边, 主题色文字压暗为 `#8c4b8c`.
- 校验: `src/components/anime/CharacterMenu/characterPrompt.check.mjs` 用仓库自带的 typescript 转译模板模块后逐条断言, `node src/components/anime/CharacterMenu/characterPrompt.check.mjs` 退出码 0 为通过, `--print` 打印三份全文.

## Alternatives considered

- 什么都不做 / 复用现有: 最强理由是 ZoomImage 与 Bangumi 链接已经能看到角色图, 提示词可以手写. 否决: 角色图是 background-image, 原生菜单拿不到图; 手拼提示词正是要消除的重复劳动, 方向词写反的问题也还在.
- 复用 DevTools 的 InlineEditor 右键菜单: 最强理由是仓里已有一套自绘右键菜单. 否决: 它绑死 dev-edit-server 与 Markdown 块选择, 生产站不渲染, 而网站模式也要有纯角色那条.
- 把 `<a>` 换成 `<img>` 让原生菜单能存图: 最强理由是零 JS. 否决: 仍拿不到提示词, 也不能区分本地 / 网站模式; 还会改动详情页角色卡布局 (另一篇 note 管).
- 引入 Radix ContextMenu: 可访问性现成. 否决: 新增依赖, 且仓里已有 FloatingLayer 定位层.
- 只用 `useDevService` 判定: 否决, `docusaurus start` 不一定同时起 dev-edit-server, 只在 localhost 开站时也要算本地模式.

## Consequences

- `src/components/anime/CharacterMenu/CharacterContextMenu.module.css` 是非 AST 资源, 需人工核对.
- 详情页每个页面多一次 `http://localhost:3310/health` 探测 (与 DevTools 已有探测相同), 生产站上必然失败, 结果是网站模式.
- 生产构建里"复制探头立绘提示词"的文案仍在产物中 (运行时过滤, 不是构建期裁切), 只是不显示; 它不含任何写入通道, 不需要 NODE_ENV 死分支.
- 详细说明、成品提示词与验证记录: `.agents/research/anime-character-context-menu.md`.
