# 范例: 一份前端界面规格长什么样

这是 `reusable-spec.md` 五层结构的**填好的样子**。取值全部来自本仓已有的三个前端包
(`components/HX-VettaUI` / `components/HX-UI` / `components/HX-VettaMirror`), 不是虚构的范例。

**用法**: 写规格时照这个骨架填自己的值。**不要照抄这些值** —— 它们会变, 而骨架不会。

## 第一层: 触发场景

    这套规格给"单窗口应用外壳 + 若干设置/工具页"的桌面式 Web 界面用。
    判据: 页面有左侧栏与顶部工具条、有设置页、有长列表或代码区块。
    纯内容站 (博客 / 文档) 与移动端优先页面不适用.

## 第二层: 契约

| 契约 | 具体值 | 判据 (会失败) |
|---|---|---|
| 类型检查 | `tsc --noEmit` 必须零错误 | 退出码非 0 |
| UI 门禁 | `check-ui-rules.mjs` | 退出码非 0 |
| 格式检查 | prettier `--check` | 退出码非 0 |
| 颜色唯一来源 | 语义 token, 组件里禁 hex / rgb | 门禁规则 `no-hex-color` |
| 组件唯一来源 | `lucide-react` 图标 + 本库基础件, 禁第二套 UI 库 | 门禁规则 `no-other-ui-kit` |

**门禁脚本必须放在仓库里并被 `package.json` 的 `scripts` 引用**, 不能只写在文档里 ——
实测踩过: `components/HX-UI` 的 `package.json` 引用了 `check-ui-rules.mjs`, 而那个文件**不存在**,
于是 `npm run build` 一直是失败的, 而文字规范看起来完好无损。

## 第三层: 约定 (已选定)

### 标准页面的骨架

每个页面都用同一个外壳, 不允许各自为政:

```text [骨架-目录]
src/primitives/   button / input / dialog / select / switch ...
src/layout/       AppFrame / SidebarNav / SettingsSidebarView ...
src/index.ts      唯一导出面
```

```tsx [骨架-页面]
<AppFrame>                     固定全屏, flex-col, bg-background
  <SidebarNav items={...} active={id} />
  <main className="min-h-0 flex-1">
    <AppTopBar />            顶部工具条, 页面级操作
    <PageContent />          唯一可滚动区
  </main>
</AppFrame>
```

骨架的三段固定为: **左侧栏 (导航) + 顶部工具条 (页面级操作) + 内容区 (滚动)**。
空态与加载态也走同一骨架的内容区, 不许整页替换 —— 否则切页时会闪。

### UI 库 (只有一个来源)

| 层 | 用什么 | 说明 |
|---|---|---|
| 基础件 | `radix-ui` 无样式原语 | 弹层 / 下拉 / 选择器 / 开关的可访问性由它兜住 |
| 变体 | `class-variance-authority` | 每个组件的 variant 与尺寸**只有这一处定义** |
| 类名合并 | `tailwind-merge` | 只在组件内部用, 页面里不许出现 |
| 图标 | `lucide-react` | 一套, 不混 Iconify |
| 动效 | `motion` | 只在挂载/退场用, 不做持续动画 |

样式走 `Tailwind 4` + `class-variance-authority`, 不用 CSS Module, 不用内联 style (动态尺寸除外)。
**颜色只在 `tokens.css` 里定义一次**, 组件写 `bg-card / text-muted-foreground`。

### 目录与分层

    src/primitives/   无样式的可访问性封装 (button / input / dialog / select ...)
    src/layout/       版式件 (外壳 / 侧栏 / 设置页行式卡片 / 分段控件)
    src/index.ts      唯一导出面, 页面只从这里 import

**页面不许直接 import `radix-ui`。** 需要新原语时先在 `primitives/` 里包一层 —— 否则无障碍属性
与类名约定会在每个页面里被重新决定一遍。

### 状态放在哪

- 页面级状态 (当前页 / 筛选) 放路由, **不放组件 state** (见第四层的路由偏好);
- 跨页共享的服务端数据放一层薄 store, 不放 Context 树;
- 只在单组件内用的 UI 状态 (展开 / 悬停) 才放 `useState`。

## 第四层: 偏好 (写法习惯)

| 偏好 | 写成可检查的形式 |
|---|---|
| 组件必须显式类型 | `tsconfig.json` 开 `strict: true` + `noUnusedLocals`; 实测本仓 `components/HX-UI` 与 `components/HX-VettaMirror` 仍是 `strict: false` —— 这是**当前的不一致**, 新项目一律开 |
| props 必须有 interface | 导出组件的 props 必须具名 interface, 不许内联对象字面量类型 |
| 导出函数显式返回类型 | `JSX.Element` 之类也要写, 不靠推断 |
| 注释只写"为什么" | 写清某行类名/某个 hack 的**来源与原因**; 不写"设置状态"这种复述 |
| **每个页面有自己的 URL 路径** | 一个根组件用 `useState` 切页面 = 违反。判据: 路由表里能查到每个可见页面的条目; 分享链接能直达 |
| 路径一律绝对别名 | `@/` 开头, 禁止 `../`。判据: 门禁加一条正则拦相对回溯 |
| 格式化统一 | prettier `.prettierrc.json` 全文入仓 + `prettier --write` 命令 |

### 为什么"每个页面一个路径"要写进规格

它决定三个月后能不能改。**一个根组件用 state 切页面**的写法在两周内跑得最快,
但之后: 无法深链、无法在浏览器前进后退里工作、无法单独给某页加错误边界、
无法按页做代码分割、加第五个页面时那个组件会长到没人敢动。

判据必须机械: 起站后逐个访问每个页面的 URL, 直链能渲染 = 通过。

## 第五层: 可直接拷走的件

| 件 | 本仓的真实位置 | 怎么被复用 |
|---|---|---|
| 令牌 | `components/HX-UI` 的 `tokens.css` | 拷过去即用, 零依赖 |
| 桥接层 | `components/HX-UI` 的 `src/styles.css` | 要 `bg-primary/10` 这类写法时再叠 |
| 门禁脚本 | `check-ui-rules.mjs` | 拷到新项目 `scripts/` |
| 骨架组件 | `AppFrame` / `SidebarNav` | 直接 import, 不重写 |
| 复现件 | 单文件 `.html` 或 `.tsx` | 正文里内嵌, 读者点开就能看到 |

**复现件必须与笔记同目录。** 实测踩过: 复现件放在笔记仓库之外的独立包里, 正文只写一句路径,
读者点不进去, agent 也无法从笔记恢复它。

## 形态一字段的对照 (写规格时最容易漏的)

| 只写这段 | 读者会得到 |
|---|---|
| "用全套语义化变量, 每个组件都要有类型" | 能懂, 但搭不出来 |
| 上面这张表 + 骨架代码 + 门禁脚本 | 能照着搭出同样的东西 |

差别不在文字质量, 在**有没有把可执行件摆出来**。
