---
hxid: "hx-32d080de"
title: "手机选购指标: 直板和折叠屏怎么打分"
created_at: "2026-11-23"
model: "hark"
skill: ["hx-ai-docs"]
authors: "Heng_Xin"
tags: ["选购指标", "手机", "折叠屏"]
mdx:
  format: mdx
---

# 手机选购指标: 直板和折叠屏怎么打分

<Lead
why="参数表上每台都写着旗舰芯片, 大电池, IP68, 到手后有的撑一天, 有的半小时就烫手掉帧"claim="参数表写的是峰值和容量, 日常过的是持续表现. 先按门槛淘汰, 再在最在意的两项里比实测, 最后才看实付价"use="看到任何一台新机, 知道该去哪查哪个数, 多少算及格, 不被发布会话术带着走"/>

方法是通用的, 例子取自 2026-11-23 前后的中国大陆在售机型. 数字来自极客湾 2026-10-27 芯片横评 ([转引页](https://www.vgover.com/news/242935)), Notebookcheck, 官方参数页和电商报道

文中 "及格, 好" 的划线是按这批机型的分布划的, 不是行业标准

## 参数表写的是峰值, 日常过的是持续

先看一个常见误判: 两台机都写着最新旗舰芯片和 6000mAh 以上的电池, 于是以为续航和游戏差不多

真正决定体验的是同一个量: <Red>同样帧率下芯片要烧多少瓦</Red>. 功耗低, 电就用得慢, 机身也热得慢. 热得慢, 系统就不用降频, 二十分钟后的帧率还在

<Figure src={require('./fig-sustain.svg').default} alt="电池容量直接影响续航, 跑分只说明上限, 能效同时决定续航和机身温度, 温度再决定持续帧率" caption="跑分和 mAh 是参数表上的数, 续航, 温度, 持续帧率才是日常. 中间那根线是能效" />

所以下面每一节都是同一个套路: 参数表上那个数为什么会骗人, 该换成哪个实测, 去哪查, 多少算好

## 续航: 别比 mAh, 比同一把尺子量出来的小时数

场景: 一台 7000mAh 的安卓和一台不到 5000mAh 的 iPhone 摆在一起, 看容量会觉得前者稳赢

朴素做法是直接比 mAh. 坏在两处: 硅碳负极让同体积电池容量涨了一截, 不同代不可比; 更关键的是能效, 同一局游戏 A20 Pro 烧 3.1W, 第六代骁龙 8 至尊版比上一代还费

该看的是同一家媒体, 同一套测试量出来的时间. 不同测试之间的小时数不能混着比

<Stats items={[
{ value: "10h37m", label: "iPhone 18 Pro Max, 极客湾续航 5.0", source: "极客湾续航榜转引, 2026-11" }, { value: "9h56m", label: "Mate 90 Pro Max, 同一测试", source: "极客湾续航榜转引, 2026-11" }, { value: "7h50m", label: "iPhone 18 Pro, 同一测试", source: "极客湾续航榜转引, 2026-11" }, { value: "81%", label: "荣耀 Power2 安兔兔 6 小时后剩余电量", source: "安兔兔续航测试报道, 2026-11", verified: false }, ]} />

怎么打分, 按直板和极客湾续航 5.0 口径

- 及格: 7 小时以上, 重度一天要补电
- 好: 9 小时以上, 正常一天用不完
- 加分: 有线 80W 以上, 或 30 分钟能充到七八成

## 游戏: 看二十分钟后的帧率和机身温度, 不看跑分

场景: 新机首发评测原神满帧, 自己玩到第十五分钟开始卡, 背面烫手

朴素做法是看安兔兔或 Geekbench. 跑分只跑几分钟, 测的是峰值. 温度撞上限后系统降频, 帧率才往下掉, 跑分根本走不到那一步

<Compare caption="同样是满帧开局" left={{
title: "只看开头", items: ["一加 16 星穹铁道约 10 分钟后锁到 50 帧左右", "机身 59.5℃ (摘要)", "Mate 90 Pro 原神能稳 60, 但机身 57.3℃"], }} right={{title: "看持续", items: ["一加 Ace 7 Pro 开风扇, 和平精英 185 帧跑一小时", "机身 32℃ (摘要)", "iPhone 18 Pro Max 散热被评为游戏手机级别"], }} />

该看的三个数: 30 分钟以上的平均帧率和 1% Low, 机身最高温度, 3DMark 压力测试稳定度. 稳定度是最后一轮和最好一轮的比值, 掉得越少越好

怎么打分

- 及格: 主流游戏 30 分钟平均帧率不低于目标帧的九成, 机身 45℃ 以下
- 好: 1% Low 不低于目标帧的八成, 3DMark 稳定度 80% 以上
- 要注意: 鸿蒙上没有原生版的游戏走转译, 同一颗芯片会更费电

## 系统: 先看手上已有的设备, 再看更新, 广告和后台

场景: 换了系统才发现手表连不上, 电脑传文件要绕一圈, 某个常用 App 没有原生版

朴素做法是比哪家动画更顺. 流畅度这几年各家差距很小, <Gold>真正换不掉的是已有设备和常用 App</Gold>

<Fork from="定系统" caption="按顺序问, 前一条有答案就停" options={[
{ name: "已有 Mac, iPad, Apple Watch", desc: "iOS 互联最强, 和安卓, Windows 互通弱", tone: "good" }, { name: "已有华为平板, 手表, 电脑", desc: "鸿蒙互联最强, 但不能直接装 APK, 长尾 App 走卓易通", tone: "ok" }, { name: "都没有, 或用 Windows", desc: "安卓选择最多, 价格降得最快", tone: "last" }, ]} />

定了系统再比三件事, 每件都要书面证据

- 更新年限: 看厂商书面承诺. 苹果在英国法规下承诺至少 5 年安全更新; ColorOS 写的是 "6 年流畅维护", 不是大版本次数
- 广告和预装: 看官方参数页的预装清单. 例如 iQOO 16 列出约 26 个第三方预装
- 鸿蒙原生应用: 应用市场约 45 万应用和服务, 其中原生口径约 10 万. 先查自己常用的几个 App 有没有原生版

## 影像: 看长焦焦段和传感器, 评分只当参考

场景: 宣传图里人像和夜景都很好看, 到了演唱会和动物园才发现拉不近, 一放大就糊

朴素做法是看主摄像素或总分. 像素数和画质关系不大, 总分把视频, 人像, 长焦平均到一起, 看不出你在意的那项

该看的

- 有没有 3 倍以上的光学长焦. 很多折叠屏和便宜机型没有, 例如 iPhone Duo 只有主摄和超广角, 10 倍数码变焦画质明显下降
- 主摄传感器尺寸, 越大弱光越从容, 参数页一般会写
- 视频看专门的视频评测. DXOMARK 的视频分 iPhone 18 Pro 全榜第一
- 自己在意的场景去翻样张, 不要看总分

DXOMARK 能参考的是同一时期的相对位置: iPhone 18 Pro 172 分全球第三, 榜首仍是上一代 Pura 80 Ultra 175 分. 今年的安卓和鸿蒙新机大多还没出分

## 防水: 看测试条件, 不只看 IP68 这几个字

场景: 手机标着 IP68, 掉进温泉或者被高压水枪冲了, 结果进水, 而且不保修

IP 后面两位分别是防尘和防水. 第一位 6 是完全防尘, 5 是有限防尘, 4 只挡 1mm 以上的颗粒. 第二位 8 是浸水, 深度和时长由厂商自己定; 9 是高温高压水柱

所以要看括号里的条件. 同样是 IP68, 所有 iPhone 是 6 米 30 分钟, Pura 90 系列只测到 2 米. <Red>任何等级都不代表进水能保修</Red>

怎么打分

- 直板: IP68 加 IP69 是 2026 年安卓和华为旗舰的常态, iPhone 只有 IP68 但深度最深. IP65 不能泡水, 例如 nova 16 Pro
- 折叠屏另算, 见后面的折叠屏一节

## 耐用: 玻璃名字是厂商口径, 第三方跌落和维修价才算数

场景: 发布会说新玻璃抗摔提升多少倍, 第一次从桌上掉下来正面就裂了

各家的玻璃名字 (昆仑玻璃, 龙晶, 超瓷晶面板, 晶盾) 和 "提升 N 倍" 都是自己和自己比, 不能横向比. 能横向比的只有两类: 第三方整机认证, 媒体做的跌落和划伤测试

能查到的例子: OPPO Find X10 Pro Max 和荣耀 Power2 有 SGS 五星整机抗跌认证. iPhone 18 Pro 的跌落测试里背板没碎, 但正面屏碎了

真实成本要算维修价, 摔坏一次的钱可能抵得上降价省下的

怎么打分

- 及格: 有官方公布的玻璃和中框材质, 重量在 230g 以内
- 好: 有 SGS 或 TÜV 这类第三方整机跌落认证
- 买前查一次官方屏幕维修价, 当成持有成本的一部分

## 价格: 看实付价走势, 不看首发价

场景: 首发价买入, 两个月后同款便宜两千

国产旗舰发布后几个月, 电商实付价普遍会降. 每次发布新一代之前, 旧款往往还会再降一轮. 下面是官方价到近期电商价的几个例子

<Chart kind="range" unit="元" source="官方价; 电商价为 2026-11 比价站和媒体转引" caption="条的右端是官方价, 左端是近期电商价. iPhone Air 降得最多, 约四成" data={[
{ name: "iPhone Air", from: 8799, to: 4989, label: "-43%" }, { name: "REDMI K100 Pro Max", from: 4499, to: 2968, label: "-34%" }, { name: "vivo X500 Pro", from: 6499, to: 4967, label: "-24%" }, { name: "X Fold6 12+512", from: 8999, to: 6835, label: "-24%" }, { name: "Find N6 12+256", from: 9999, to: 7604, label: "-24%" }, { name: "Mate X7 12+256", from: 12999, to: 9988, label: "-23%" }, { name: "Pura 90 Pro Max", from: 6499, to: 5109, label: "-21%" }, { name: "Find X10 Pro Max", from: 6799, to: 5849, label: "-14%" }, ]} />

查价用比价站看最低价, 再到结算页看叠完券的到手价. 国补只补成交价不超过 6000 元的手机, 补 15%, 封顶 500 元

要注意: 拼多多这类百亿补贴的低价一般要当面激活, 激活后不能无理由退货. 小米 18 Pro 这类基本不直降的, 首发买和半年后买差得不多

## 可玩性: 想 root, 先看厂商的门还开不开, 再看你的系统版本

场景: 买来想装 LSPosed 模块或者刷国际版, 才发现解不了 BL. 或者能解, 一次系统更新之后就再也解不了

朴素做法是搜 "某某机型能 root 吗". 坏在答案按系统版本变: 官方通道一年比一年窄, 漏洞会被补, 有的更新会熔断防回滚, 降级都回不去

2026 年末国行的大致情况

- 官方关门: 小米 2026-01-01 关停了社区的解锁申请通道. 华为 2018 年起停止提供解锁码, 荣耀也没有官方解锁. vivo 和 iQOO 从未提供
- 官方关门: 三星从 One UI 8 起在所有地区取消 BL 解锁, 这是社区确认, 三星没有发声明
- 申请制: 一加国行出厂 ColorOS 16 的机型要申请深度测试. 2026-09 起要实名加人脸, 账号满 60 天, 每月 1 日放名额, 一年最多 4 台 (摘要)
- 只剩漏洞: 小米 17 系列等第五代骁龙 8 至尊版机型, 2 月补丁之前的系统有漏洞可解 (摘要). 部分天玑机型有第三方提权工具 (摘要)
- iOS: iOS 27 没有公开越狱工具. Calif 演示过 iPhone 18 Pro 和 iPhone Duo 的私有越狱, 但说不做面向用户的工具 (摘要)

<Fork from="想要 root" caption="按顺序看, 越往下越不稳" options={[
{ name: "官方通道", desc: "一加国行深度测试, 解锁记录官方认账, 但要排队审核", tone: "good" }, { name: "版本漏洞", desc: "只对没升级的系统有效, 升级可能熔断, 不能降级", tone: "ok" }, { name: "免解锁临时 root", desc: "靠内核提权, 重启就没了, 版本墙很窄", tone: "last" }, ]} />

代价也要算进去: 解锁会清空全部数据. 银行, 支付这类 App 可能检测到并拒绝运行. 三星的 Knox 一旦触发是永久的

怎么打分

- 不在乎 root: 这一项不计分
- 想 root: 及格是买到手时厂商还有官方申请通道; 拿到后先申请, 系统别急着升
- 以上状态变化很快, 下单前去查最新的社区汇总

## 折叠屏: 用重量, 厚度, 防水和耐用换一块大屏

场景: 想要一台能看文档和漫画的大屏手机, 买回来发现每天揣着的是一块 250g 的砖, 常用 App 展开后两边留黑条

折叠屏在上面每一项之外, 还多出几个直板没有的维度. 先看它整体拿什么换了什么

<Compare caption="同一年的旗舰" left={{
title: "直板旗舰", items: ["IP68 加 IP69 很常见", "旗舰基本都有 3 倍以上长焦", "同代电池更大", "只有一块屏要护"], }} right={{title: "大折叠", items: ["多为 IP58, IP48, 防尘弱", "部分没有长焦, 如 iPhone Duo", "展开后是 7.6 到 8 英寸的大屏", "iPhone Duo 内屏保外维修 7998"], }} />

折叠屏多看的几项

- 重量和折叠厚度: 合上后就是一台直板机, 这两项决定能不能天天揣着. 大折叠目前 214g 到 254g, 折叠厚度 8.75mm 到 11.3mm; 三折叠 Mate XT 2 是 299g
- 铰链和折痕: 认证次数要看是谁认证的. OPPO Find N6 有 TÜV 莱茵 60 万次认证; 好几家只有厂商口径, 不同报道数字互相矛盾
- 内屏比例: 接近方形 (约 1.1:1) 适合分屏和多窗; 接近 √2 或 4:3 的阔折叠看视频, 读漫画黑边少
- 外屏比例: 20:9 左右和直板手感一样, 合上也能正常用
- App 适配: 何同学测了 iPhone Duo 上 25 个常用 App, 约八成没有为折叠屏适配
- 防水: 大折叠里 IP58 加 IP59 已经算好, 荣耀 Magic V6 做到 IP68 加 IP69; 三星 Z Fold8 Ultra 是 IP48

小折叠是另一种取舍: 主打便携, 电池常在 4300 到 4400mAh, 通常没有长焦, 防水经常缺. 看外屏能不能直接跑完整 App

## 直板参照: 芯片强弱去看极客湾的能效曲线, 单场游戏的帧率只代表那台手机

芯片怎么比: 打开能效曲线, 在你常用的功耗段比同功耗下的分数. 玩游戏看 3W 到 7W 那段, 不看曲线最右端的峰值

现成的曲线在 SOCPK, 是极客湾的芯片数据库: [CPU 单核](https://socpk.com/chart/mobile-soc-spec26), [CPU 多核](https://socpk.com/chart/mobile-soc-efficiency-gb7), [GPU](https://socpk.com/chart/mobile-soc-efficiency-snl). 新芯片出来先查这里

极客湾 2026-10-27 的横评结论: 单核 A20 Pro 第一, 玄戒 O3 第二; 多核和 GPU 都是玄戒 O3 第一, 靠 10 核 CPU 和 16 核 GPU 堆出来. 骁龙 8 Elite Extreme Gen6 能效倒退

游戏帧率测的是整台手机. 玄戒 O3 只能在小米 18 Fold 上测, 终末地 47.9fps. 这个数里有折叠机身的散热限制, 不能直接当成芯片的成绩

另外两点 (摘要): 18 Fold 只有 16+1T 是 LPDDR6 内存, LPDDR5X 版超大核没跑满; 小米的调度在游戏里刻意不调用两颗 Ultra 超大核

## 直板参照: 续航去查同一张榜

续航只比同一套测试. 极客湾的 [手机续航排行榜](https://socpk.com/chart/battery-life-5-0) 是现成的全量榜单

安兔兔 6 小时综合测试也是同一把尺子. 下面几台是能查到的这一代机型

<Chart kind="bar" unit="%" max={100} source="安兔兔 6 小时综合续航, 2026 年各机型评测转引" caption="跑完 6 小时还剩多少电, 剩得越多越耐用. 虚条只见于搜索摘要" data={[
{ name: "荣耀 Power2", value: 81 }, { name: "一加 16", value: 69 }, { name: "iQOO 16", value: 68 }, { name: "一加 Ace 7 Pro", value: 65 }, { name: "vivo X500 Pro Max", value: 50, muted: true }, ]} />

## 折叠参照: 合上之后, 重量差了 85g

大折叠合上就是一台直板机, 揣不揣得住看重量和厚度. 最轻一档 214g 到 225g, 折叠厚度 9mm 以内; iPhone Duo 254g, 三折叠 Mate XT 2 是 299g

<Chart kind="bar" unit="g" min={150} max={300} source="各品牌官方参数页" caption="条上标的是重量和折叠后厚度. Magic V6 是白色版, 其他配色 224g, 9mm" data={[
{ name: "三星 Z Fold8 Ultra", value: 214, label: "214g · 8.9mm" }, { name: "荣耀 Magic V6", value: 219, label: "219g · 8.75mm" }, { name: "小米 18 Fold", value: 219, label: "219g · 10.68mm" }, { name: "OPPO Find N6", value: 225, label: "225g · 8.93mm" }, { name: "vivo X Fold6", value: 228, label: "228g · 9.4mm" }, { name: "华为 Pura X Max", value: 229, label: "229g · 11.2mm" }, { name: "华为 Mate X7", value: 235, label: "235g · 9.5mm" }, { name: "iPhone Duo", value: 254, label: "254g · 11.3mm" }, { name: "华为 Mate XT 2 三折", value: 299, label: "299g · 12.3mm" }, ]} />

## 折叠参照: 同一套测试里, 续航差了十个小时

Notebookcheck 用同一套 Wi-Fi 网页测试量了六台大折叠. iPhone Duo 和华为折叠屏没有这套测试的数, 不放进来混比

<Chart kind="bar" unit="h" source="Notebookcheck Wi-Fi 网页续航" caption="荣耀 Magic V6 测的是 6660mAh 全球版, 国行是 6850 或 7150mAh" data={[
{ name: "小米 18 Fold", value: 26.77, label: "26h46m" }, { name: "vivo X Fold6", value: 20.2, label: "20.2h" }, { name: "moto razr fold", value: 18.37, label: "18h22m" }, { name: "OPPO Find N6", value: 18, label: "约 18h" }, { name: "三星 Z Fold8 Ultra", value: 17.18, label: "17h11m" }, { name: "荣耀 Magic V6", value: 16.18, label: "16h11m" }, ]} />

## 折叠参照: 长时间负载下, 多数折叠屏守不住

折叠机身薄, 散热空间小. 3DMark 压力测试里六台有四台在 70% 以下. X Fold6 稳定度最高, 但 Notebookcheck 说它高负载非常热

<Chart kind="bar" unit="%" max={100} source="Notebookcheck 3DMark 压力测试; iPhone Duo 为 Tom's Guide" caption="稳定度是最差一轮除以最好一轮. 两组负载不同, 只在组内比" data={[
{ name: "vivo X Fold6", value: 84.9, group: "Wild Life" }, { name: "小米 18 Fold", value: 66.6, group: "Wild Life" }, { name: "moto razr fold", value: 82.1, group: "Wild Life Extreme" }, { name: "iPhone Duo", value: 67.9, group: "Wild Life Extreme" }, { name: "荣耀 Magic V6", value: 55.4, group: "Wild Life Extreme" }, { name: "OPPO Find N6", value: 47.5, group: "Wild Life Extreme" }, ]} />

Mate X7 在 Phonandroid 的压力测试里不超过 50%. 防水: Magic V6 是 IP68 加 IP69; 华为三台是 IP58 加 IP59; Z Fold8 Ultra 是 IP48

华为 Mate X8 定在 2026-11-24 14:30 发布, 目前只有爆料, 没放进来

## 放到我自己的选购里

- 先过门槛: 系统 (已有设备), 防水是否满足用法, 重量能不能接受. 一项不过直接排除, 不进入比较
- 再挑两项最在意的维度, 只在这两项上比实测数, 其余及格就行
- 最后看实付价走势: 新一代发布前后, 或双 11, 618 这种节点再下单
- 折叠屏多加三问: 常用 App 有没有适配, 内屏维修价多少, 合上后的重量和厚度能不能天天揣
- 想 root 的, 把厂商有没有官方解锁通道放进门槛, 和系统一起先过

## 什么时候不成立

<Boundary items={[
"划线按 2026 年末这批机型的分布划的, 一两代之后会整体上移", "极客湾续航榜页面是 JS 渲染, 本文只拿到转引的几个数, 其余机型缺这一项", "很多电商价来自媒体和比价站报道, 不是京东原页面, 每天都在变", "部分测试是海外版: 荣耀 Magic V6 测的是 6660mAh 全球版, 三星 Z Flip8 测的是 Exynos 版, 国行会不同", "跌落, 划伤和铰链寿命的第三方数据很少, 大多只有厂商口径", "带 (摘要) 的数字只在搜索摘要里见过, 没有打开原页面核对", "解锁和 root 的情况按月在变, 本文是 2026-11 的状态", ]} />

## 留着查

去哪查哪个数

- 芯片能效和游戏帧率: 极客湾 B 站横评, 能效曲线数据库 [socpk](https://socpk.com/)
- 折叠屏续航和稳定度: Notebookcheck, 例如 [小米 18 Fold 评测](https://www.notebookcheck.net/Xiaomi-18-Fold-review-Big-display-big-performance-big-compromises.1428903.0.html)
- 影像: DXOMARK 的分项分 (视频, 长焦, 人像), 不看总分
- 防水, 重量, 厚度, 电池: 官方参数页, 例如 [iPhone Duo 技术规格](https://support.apple.com/zh-cn/148588), [Mate X7 参数](https://consumer.huawei.com/cn/phones/mate-x7/specs/)
- 解锁和 root: 社区维护的 [各品牌 BL 解锁现状汇总](https://github.com/AdaUnlocked/2026-All-Brands-Bootloader-Unlock-Status), 以及 [Droid Rooter 的品牌状态表](https://www.droidrooter.com/zh-hans/blog/unlock-bootloader-android-guide)
- 实付价: 比价站看最低价走势, 例如 [Mate X7 比价](https://compare.fenxianglife.com/p/huawei-mate-x7-th2r7ehol2.html), 再到结算页确认

3DMark 稳定度的测试项目不全相同. 小米 18 Fold 和 X Fold6 来自 Wild Life

Magic V6, Find N6, razr fold 和 iPhone Duo 来自 Wild Life Extreme

iPhone Duo 那组是 Tom's Guide 测的, 其余来自 Notebookcheck

Wild Life 负载更轻, 两组之间只能粗看
