# 3 figures: 画结构图

只做一件事: 给需要看结构的节配图. 产物是笔记同目录的 `fig-<名字>.svg`, 正文用 `<Figure src={require('./fig-x.svg').default} alt=... caption=... />` 引用

## 什么时候画

- 谁和谁之间有数据流, 而且有一条路绕过了另一条: 结构图
- 两个参与方之间按时间来回: 泳道时序图
- 有阈值的时间或数量: 横向色带加刻度
- 一行代码里的位置关系: 逐字定位的代码行
- 只是左右对照或三步顺序: 不画图, 用 Compare 或 Flow

## 怎么画

用 `scripts/svgfig.py`. 在 `ai-docs/.hx-staging/<主题>/figs.py` 里 import 它, 写坐标, 运行后把 SVG 复制进笔记目录

```python
import sys; sys.path.insert(0, '.agents/skills/hx-ai-docs/scripts')
from svgfig import Fig
f = Fig(520, 250, '谁在改文件, 谁在回答查询')
f.zone(6, 30, 150, 212); f.text(14, 20, '谁在写文件', 'h')
f.box(20, 188, 122, 36, 'bash, sed', 'pill pk', 'pk')
f.box(220, 184, 140, 40, '磁盘上的文件')
f.line([(142, 206), (220, 206)], 'pk')
f.save('fig-copy.svg')
```

类名的含义 (样式在 `HXLoLi/src/components/KBNote/kb-note.css`)

- `box` 节点, `pill` 圆角胶囊, `pk` 强调色, `ghost` 虚线框, `zone` 分组底色
- `ln` 连线, `dash` 虚线, `sep` 泳道生命线, `band` 色带
- 文字 `h` 小标题, `s` 小字, `b` 加粗, `pk` 强调色

## 过关条件

- 每个节点和每条边都对应来源里真实存在的东西
- 一张图只用一处强调色, 标的是这一节要说的那条路
- 第 4 步的截图里没有字压线, 没有出框
