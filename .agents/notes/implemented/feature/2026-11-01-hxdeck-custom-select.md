# Agent Note: hxdeck 原生下拉换成站点 Select

Status: implemented

Decision-ID: hxdeck-custom-select

## Problem
grep `<select` 在 src/hxdeck/ 命中 4 处: Dropdown.tsx 2、ThemeEditor.tsx 1、PptEmbed.tsx 1. 其中只有 ThemeEditor 的"融合"是真正的原生 select, 它展开后是系统菜单, 和站点深色风格不一致.

## Decision
- ThemeEditor.tsx 的融合模式改用 `src/components/common/Tooltip/Select.tsx`, 选项提成模块常量 BLEND_OPTIONS, 值类型直接取 `SkinSlot['blend']`.
- Dropdown.tsx 跳过: 它本身就是自绘下拉 (button + listbox), 两处 `<select` 都只在 JSDoc 注释里, 解释为什么不用原生.
- PptEmbed.tsx 跳过: 唯一命中是注释, 主题切换已经在用上面的 Dropdown.
- ui.css 删掉 `.hxed__mini select` 规则, 新增 `.hxed__mini .hxed__sel` 让触发按钮撑满格子.

## Alternatives considered
- 什么都不做: 最强理由与结论 —— ThemeEditor 的原生 select 能用, 系统菜单在功能上没问题. 否决: 用户点名全站下拉要统一到站点样式, 原生展开列表 CSS 碰不到, 不换就做不到.
- 复用现有 Dropdown 组件承载 ThemeEditor 的下拉: 最强理由与结论 —— 最强理由是仓库已有一个自绘下拉, 复用比新增组件省一套维护. 否决: Dropdown 的定位与 PptCard 工具栏耦合, 语义是"工具栏菜单"而非"表单取值", 套过来会把两处耦合一起带进来.
- 把 Dropdown 改成 Select 的薄包装: Dropdown 用 absolute 定位挂在 PptCard 工具栏里, 放大弹层也依赖它; 改了收益是统一一套, 代价是要回归 PptCard 的两个位置. 本轮按指示不动.
- 在 hxdeck 里复制一份 Select: 会出现两套组件, 否决.

## Consequences
- hxdeck 第一次从 `../components` 引入组件 (ThemeEditor 原本只引 hxdeck 内部). 如果以后要把 hxdeck 拆成独立包, 这条引用要跟着处理.
- 融合下拉现在有键盘导航、点外部关闭、portal 浮层, 不会被编辑器面板的 overflow 裁掉.

## Code

- `src/hxdeck/ThemeEditor.tsx`
