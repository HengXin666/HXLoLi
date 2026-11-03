# anime-sedai.egoist.dev 年份表格实测 (2026-11-02 22:35 CST)

来源: 线上 bundle `/assets/index-WKy_kSUx.js` (JSX className 原文) + `/assets/index-354HzvRz.css` (Tailwind v4.1.7). 不是凭印象.
用户贴的 "2006 / 死亡笔记 银魂 ..." 是 textContent 拍平后的文字, 不是版式.

## 结构
外框: `div.flex.flex-col.border.border-b-0.bg-white.w-fit.mx-auto` (有 1px 外框, 白底, 居中, 宽度=内容)
标题行: `border-b p-2 text-lg font-bold flex justify-between` (左标题 + 灰色站名, 右 "看过 x/y")
每个年份一行: `div.flex.border-b` (行与行之间 0 间距, 只有 1px 底线)
- 年份格: `bg-red-500 text-white font-bold flex items-center justify-center p-1 shrink-0 h-16 w-16 md:h-20 md:w-20`, 文字 `text-base` (16px)
  -> 红底 oklch(63.7% .237 25.331) 白字 80x80 (md 以上) 方块
- 番名: `div.flex.shrink-0` 里每部一个 `<button>`: `h-16 w-16 md:h-20 md:w-20 border-l text-center inline-flex items-center justify-center p-1 overflow-hidden text-sm` + 内层 `span.leading-tight.line-clamp-3`
  -> 每部一个 80x80 固定格, 14px, 行高 1.25, 最多 3 行截断, 居中
- 每年只取前 12 部 (`slice(0,12)`), 不足补 `bg-gray-50` 空格; 行尾 `w-0 border-r` 收边
- 选中: `bg-green-500` 整格绿底; 未选: `hover:bg-zinc-100`
- 分隔: 不是空格, 不是 margin/gap, 是格子之间的 1px `border-l`
- 换行: 不换行. 一行固定 13 格 (年份 + 12), 窄屏横向滚动 (`overflow-x-auto`)

## 对照题问
1. 年份: 16px / 700 / 白 / 红底 / margin 0 (格子垂直居中)
2. 番名分隔: 1px border-l 格线
3. 番名: 14px / leading-tight (1.25) / 默认文字色 / 选中 bg-green-500
4. 每部一个块 (button), 居中, line-clamp-3
5. 年份顶格, 是行首第一格
6. 年份间距 0px, 只有 1px border-b

结论: 参考站是固定格子网格, 和 "无外框 / 年份非反色 / 密排文字流" 的判据正相反.
