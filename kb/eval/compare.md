# 逐题对照

判分盲评, 字母与来源对应见 grade/key.json; 下表已解盲. 答对 = 正确 / 正确拒答.
合计: 卡库 27/30, B 16/30, OLD 6/30


## bag

### bag-01 
**问**: 我有一台 16 寸 MacBook Pro (M5 Pro), 下个月坐首都航空经济舱出差, 只想带一个电脑包上飞机不托运. 电脑本身塞进他家经济舱随身行李的尺寸和重量限制够不够? 包该挑多大?

**标准答案**: 首都航空运输总条件规定经济舱随身一件, 每件不超过 7 kg, 体积不超过 20 × 30 × 40 cm, 且须能放进前排座位下或封闭行李架. 16 寸 MacBook Pro 本体为 35.57 × 24.81 × 1.68 cm, 2.14/2.15 kg, 本体放得进这个尺寸 (长边余约 4.4 cm, 短边余约 5.2 cm), 但余量不大; 包的外形整体 (连同电脑与其他物品) 都不能超过 20 × 30 × 40 cm, 总重不超 7 kg. 注意不能套用航旅指南的 20 × 40 × 55 cm, 首都航空经济舱比它小.

- **kb** [正确] 一件/7 kg/20×30×40 cm/前排座位下、机身尺寸与 2.14 kg、勿按 20×40×55 买, 三个 keypoint 全中.
  - 答: 够. 首都航空经济舱随身行李: 只能带一件, 每件 7 公斤为限, 体积不超过 20 × 30 × 40 厘米, 且须能放进前排座位下或封闭行李架. 16 寸 MacBook Pro (M5 Pro) 宽 35.57 cm, 深 24.81 cm, 高 1.68 cm, 重 2.14 kg, 三个尺寸都在 40 × 30 × 20 cm 之内, 重量也远低于 7 公斤. 挑包: 包装满后的外部尺寸不超过 20 × 30 × 40 厘米 (这是唯一一件随身行李), 电脑仓内尺寸要能容下 35.57 × 24.81 × 1.68 cm; 包连电脑和其他东西总重不超过 7 公斤, 即除去电脑约 2.14 kg 后, 包身加其他物品约 4.86 公斤以内; 装满后仍要能塞进前排座位下或封闭行李架. 注意不要按网上常见的 20 × 40 × 55 厘米买, 首都航空经济舱比这个小. 出行前再核对承运人运输总条件是否有更新.
  - 依据: life/bag/capital-airlines-economy, life/bag/capital-airlines-under-seat, life/bag/macbook-pro-16-dimensions, life/bag/pitfall-domestic-size-not-uniform
- **B** [部分] 尺寸上限与机身尺寸都对, 但没提醒不能套用 20×40×55, 且称 M5 Pro 重量查不到 (gold 给 2.14/2.15 kg), 无错误断言.
  - 答: 尺寸够: 资料给的 MacBook Pro 16 机身为 355.7 × 248.1 × 16.8 mm, 首都航空经济舱随身行李上限 20 × 30 × 40 cm、7 kg. 放进 400 × 300 mm 时两侧合计余量 44.3 mm 和 51.9 mm, 厚度 16.8 mm 远小于 200 mm. 选包: 包外廓 (机身 + 两侧衬垫和面料) 必须 ≤ 20 × 30 × 40 cm, 即单侧约 22 mm (宽向) / 26 mm (深向) 以内的壁厚, 用电脑仓内尺寸对机身规格页毫米数, 不看 '16 寸'. 重量: 资料只有 M5 Max 版 2.15 kg (占 7 kg 的 31%), M5 Pro 版的重量查不到, 所以电脑本身是否够重量部分无法确切判断 (按 M5 Max 的数, 电脑之外还剩约 4.85 kg). 另外资料注明首都航空条款取自携程转载, 未在航司官网核对版本日期, 需去首都航空运输总条件确认. 资料里的 MacBook Pro 16 外形尺寸是否就是 M5 Pro 款也查不到.
  - 依据: B.md 尺寸: 别信英寸, 比毫米 (行 40-48), B.md 尺寸上限: 包被两头夹住 (行 50-83)
- **OLD** [查不到] 声明首都航空限制和机身尺寸都查不到, 未答出任何 keypoint; 另给的 '外部宽度≥电脑宽+3.5 cm' 不在 gold 内但不冲突.
  - 答: 首都航空经济舱随身行李的尺寸和重量限制: 查不到; 16 寸 MacBook Pro (M5 Pro) 的长宽和重量: 查不到, 所以没法判断够不够. 资料能给的挑包方法: 别只看包标的英寸数, 直接量电脑长宽, 重点看宽度; 先找商品页有没有给电脑仓内部最大尺寸; 如果只给外部尺寸, 外部宽度至少要有 电脑宽度 + 约 3.5 cm (实测内部宽度一般比外部少约 2.5-3.5 cm). 包的外形尺寸能不能过航司限制: 查不到.
  - 依据: OLD.md 0x01 第2条 (L36), OLD.md 2.3 (L75-86)

### bag-02 
**问**: 网上很多攻略说国内航班随身行李统一是 20 × 40 × 55 厘米, 我按这个尺寸买包, 坐任何一家国内航司都没问题吗?

**标准答案**: 不一定. 20 × 40 × 55 cm 出自 1996 年国内运输规则, 该规则已被 2021-09-01 起施行的《公共航空运输旅客服务管理规定》废止, 现在随身行李的尺寸、重量、数量由各承运人在运输总条件中自定. 民航局航旅指南仍写每件不超过 20 × 40 × 55 cm, 但个别航司更严, 例如首都航空经济舱只允许 20 × 30 × 40 cm. 应以所乘航司的运输总条件为准.

- **kb** [部分] 答到新规废止、承运人自定、首都航空反例, 但未点明 20×40×55 出自 1996 规则第三十七条; '航旅指南总重不超过 5 公斤' 为 gold 外细节, 不冲突.
  - 答: 不是. 民航局航旅指南现行写法确实是国内航班随身行李总重不超过 5 公斤, 每件不超过 20 × 40 × 55 厘米, 但 2021-09-01 起施行的《公共航空运输旅客服务管理规定》废止了 1996 年规则, 改为由各承运人在运输总条件中自定非托运行李的尺寸、重量和数量, 所以 20 × 40 × 55 已不是全国统一规则. 例如首都航空经济舱上限是 20 × 30 × 40 厘米 (7 公斤, 一件), 按 20 × 40 × 55 买的包坐首都航空经济舱会超. 应去查所乘承运人、舱等的运输总条件. 其他各家国内航司的具体上限资料里查不到.
  - 依据: life/bag/caac-guide-domestic, life/bag/caac-2021-carrier-sets-limits, life/bag/pitfall-domestic-size-not-uniform, life/bag/capital-airlines-economy
- **B** [部分] 答到 1996 规则被 2021-09-01 新规废止、承运人自定、首都航空反例, 但没指出 20×40×55 出自 1996 规则第三十七条; 所写 '第六十五条废止' '第三十七条改为承运人自定' 为 gold 外条文号, 无法核实.
  - 答: 不能保证. '国内全国统一 20 × 40 × 55' 已不成立: 2021-09-01 施行的《公共航空运输旅客服务管理规定》第六十五条废止了写这组数的 1996 年规则, 第三十七条改为由承运人在运输总条件里自定尺寸和重量. 例如首都航空经济舱是 20 × 30 × 40 cm、7 kg, 比 20 × 40 × 55 小. 坐哪家就去翻哪家的运输总条件.
  - 依据: B.md 尺寸上限: 包被两头夹住 (行 63-83)
- **OLD** [查不到] 声明资料没写国内随身行李尺寸.
  - 答: 查不到. 资料没写国内航班随身行李尺寸, 也没写各航司规定.
  - 依据: 

### bag-03 
**问**: 包的吊牌写着符合 IATA Cabin OK 尺寸 (55 × 35 × 20 cm), 是不是就能保证所有航司都让我带进客舱?

**标准答案**: 不能保证. IATA 明确说 Cabin OK 的 55 × 35 × 20 cm 是建议尺寸, 不是最大尺寸, 最大尺寸由各航司自定; 随身行李上限因航司、舱等、机型而异. IATA 给的通用参考是许多航司最大 56 × 45 × 25 cm (含轮子与把手), 且部分航司另有重量限制 (通常从 5 kg 起). 以具体航司规定为准.

- **kb** [正确] 建议非最大、航司/舱等/机型而异、56×45×25 cm、5 kg 起重量限制、不等于一定能带, 全中.
  - 答: 不能保证. IATA 明说 Cabin OK 的 55 × 35 × 20 cm 是 '建议尺寸, 不是最大尺寸', 最大尺寸由各航司自定, 且随身行李上限因航司、舱等、机型而异 (IATA 的通用参考是许多航司最大 56 × 45 × 25 cm, 含轮子与把手). 有的航司比它小, 例如首都航空经济舱上限 20 × 30 × 40 厘米, 55 × 35 × 20 的包就超了. 另外部分航司还有重量限制, 通常从 5 kg 起. 所以要以所乘航司公布的上限为准.
  - 依据: life/bag/iata-cabin-ok-not-max, life/bag/pitfall-iata-size-not-guarantee, life/bag/iata-carryon-reference-size, life/bag/capital-airlines-economy, life/bag/carryon-weight-from-5kg
- **B** [部分] 答到 Cabin OK 是建议尺寸、上限由航司自定, 但缺 IATA 通用参考 56×45×25 cm 和重量限制 (通常 5 kg 起).
  - 答: 不能保证. IATA Cabin OK 的 55 × 35 × 20 cm 只是建议尺寸, 上限由各航司自定. 比如首都航空经济舱上限是 20 × 30 × 40 cm, 55 cm 长的包就超了. 要以所乘航司运输总条件为准.
  - 依据: B.md 尺寸上限: 包被两头夹住 (行 63-72, 83)
- **OLD** [查不到] 声明资料没提 IATA Cabin OK.
  - 答: 查不到. 资料没提 IATA Cabin OK, 也没提航司客舱行李规定.
  - 依据: 

### bag-04 
**问**: 我想买个标着 TSA checkpoint friendly 的电脑包, 去美国出差过安检时就不用把电脑拿出来了吧? 我没有 PreCheck.

**标准答案**: 不能这么指望. TSA 现行页面要求把笔记本电脑从包里取出单独放一个筐过 X 光, 只有 TSA PreCheck 旅客不需要取出; 你没有 PreCheck, 就要取出. checkpoint friendly 是 TSA 2008 年的设计准则 (专门只放电脑、能完全展开平放的隔层, 隔层内外上下无金属按扣/拉链/扣具, 无口袋, 只放电脑), 准则本身就保留了 TSA 对任何包重检的权利, 不保证省掉取电脑这一步.

- **kb** [正确] 现行取出要求、PreCheck 例外、2008 准则四项内容、保留重检权、不保证省步骤, 全中.
  - 答: 不能这么指望. TSA 现行页面要求把笔记本电脑从包里取出, 单独放一个筐过 X 光, 只有 TSA PreCheck 旅客不需要取出; 你没有 PreCheck, 应按取出电脑准备. 'checkpoint friendly' 是 TSA 2008 年的设计准则 (有只放电脑的专用隔层且能完全展开平放在传送带上; 隔层内、下、上没有金属按扣、拉链、扣具; 隔层内外没有口袋, 里面只放电脑), 但该准则本身就保留重检权, 不保证省掉取电脑这一步. 建议把电脑放在能快速取出的位置.
  - 依据: life/bag/tsa-remove-laptop, life/bag/pitfall-checkpoint-friendly-still-remove, life/bag/checkpoint-friendly-compartment, life/bag/checkpoint-friendly-no-metal, life/bag/checkpoint-friendly-no-pockets
- **B** [部分] 答到需取出电脑/PreCheck 例外、TSA 保留重检权, 但未讲 2008 准则的独立平放隔层、无金属件等要点.
  - 答: 不行. 据 TSA 2026-07 页面, 普通旅客过安检要取出电脑, PreCheck 旅客不用. 你没有 PreCheck, 所以仍要取出. checkpoint friendly 的代价 (电脑仓里不能放充电器和线材、隔层内外无口袋等) 每天都在付, 收益只对 PreCheck 旅客稳定兑现; 且 TSA 保留重检任何包的权利.
  - 依据: B.md 安检: checkpoint friendly 的账 (行 100-116)
- **OLD** [查不到] 声明资料没提 TSA/PreCheck.
  - 答: 查不到. 资料没提 TSA checkpoint friendly, 没提美国安检, 也没提 PreCheck.
  - 依据: 

### bag-05 
**问**: 两款通勤背包都标 25L, 其中一款外侧有两个大网兜水壶袋. 按背包容量的标准测法, 这两个网兜算不算进这 25L? 两款的 25L 能直接对比吗?

**标准答案**: 按 ASTM F2153, 没有被面料完全包围的区域不计入容量, 网兜、水壶袋、压缩袋口袋都不算. 该标准还区分 '未扩展容量' 与 '扩展容量' 两个值, 目的是让容积可以用一致方法比较. 所以只有两家都按 ASTM F2153 测、且说明是哪一个值时才能直接比; 商品页只写 '25L' 不说测法时无法确认 (另外该标准不适用于小于 4 L 的容量).

- **kb** [正确] 不计入区域 (含压缩袋口袋)、两个容量值、一致比较前提, 全中.
  - 答: 按 ASTM F2153 测法, 网兜、水壶袋、压缩袋口袋这类没有被面料完全包围的区域不计入容量, 所以两个网兜水壶袋不算在 25L 里 (前提是该款按 ASTM F2153 测). 能否直接对比: 只有两款都按 ASTM F2153 测定时才可比; 还要确认两款标的都是 '未扩展容量' 或都是 '扩展容量' (该标准区分这两个值). 这两款实际用的什么测法、标的是哪个值, 资料里查不到, 需向商家确认.
  - 依据: life/bag/capacity-excludes-open-pockets, life/bag/capacity-compare-same-method, life/bag/capacity-two-values
- **B** [正确] 网兜水壶袋不计入、区分未扩展/扩展容量、只有同按 F2153 且同口径才可比, keypoint 全中.
  - 答: 按 ASTM F2153 测法, 网兜、水壶袋这类没被面料完全包围的地方不计入容量, 所以两个网兜不算进 25L. 两款能否直接对比: 只有都声明按 ASTM F2153 测、且说明是未扩展还是扩展容量时才可比; 只写 '25L' 没配测法的, 按 '未声明' 处理, 不能直接比. 至于这两款各自实际是怎么测的, 查不到.
  - 依据: B.md 先认一条规矩 (行 15-27), B.md 容量: 升数先问是哪种升 (行 29-36)
- **OLD** [查不到] 声明标准测法查不到, 只给定性建议, 未答出 keypoint.
  - 答: 背包容量的标准测法、外侧网兜算不算进标称升数: 查不到. 资料能说的是: 标称升数只能当参考, 不等于实际好装; 骨架软硬、分区多少、开合方式 (翻盖无拉链装满会合不上)、有没有扩容结构都会影响能用的空间, 所以两款都标 25L 不能直接对比, 要拿自己真实要装的东西去比.
  - 依据: OLD.md 0x01 第3条 (L37), OLD.md 2.2 (L60-73)

### bag-06 
**问**: 同样标 16 寸电脑仓, 我的 Lenovo Legion Pro 7i 16 寸 (第 9 代) 和同事的 16 寸 MacBook Pro 实际外形差多少? 买包时该看什么数而不是看 '16 寸'?

**标准答案**: '16 寸' 是屏幕对角线 (Apple 规格页屏幕尺寸按标准矩形的对角线测量; Legion 屏幕对角线 16.0 英寸), 不是机身尺寸. Legion Pro 7i (16'', 9) 机身宽 363.5 mm, 深 262.1 mm, 厚 21.95 到 25.9 mm; 16 寸 MacBook Pro 宽 35.57 cm, 深 24.81 cm, 高 1.68 cm. Legion 宽约多 0.8 cm, 深约多 1.4 cm, 厚约多 0.5 到 0.9 cm. 买包应拿电脑仓的内部长宽厚与机身实测尺寸比, 不看英寸标称.

- **kb** [正确] 对角线、两机机身尺寸、差值 (宽 7.8/深 14.0/厚 5.2–9.1 mm) 全中, 与 gold 一致.
  - 答: '16 寸' 是屏幕对角线, 不是机身尺寸. Lenovo Legion Pro 7i (屏幕 16.0 英寸): 宽 363.5 mm, 深 262.1 mm, 厚 21.95–25.9 mm. 16 寸 MacBook Pro (M5 Pro/M5 Max, Apple 标 16.2 英寸): 宽 355.7 mm, 深 248.1 mm, 高 16.8 mm, 重 2.14/2.15 kg. 差值: Legion 宽多约 7.8 mm, 深多约 14.0 mm, 厚多约 5.2–9.1 mm. 买包时看电脑仓内尺寸的宽、深、厚, 对照电脑规格页的机身宽、深、高 (厚度按最厚处 25.9 mm 算). 注: 资料里的 Legion 卡片未写明是第 9 代, 也查不到 Legion 的重量.
  - 依据: life/bag/screen-size-is-diagonal, life/bag/legion-pro-7i-16-dimensions, life/bag/macbook-pro-16-dimensions
- **B** [正确] 对角线概念、Legion 363.5×262.1×21.95–25.9 mm 与 16.0 英寸、MacBook 355.7×248.1×16.8 mm 及差值, 全中.
  - 答: Legion Pro 7i 16 (第 9 代): 16.0 英寸, 363.5 × 262.1 × 21.95 到 25.9 mm; MacBook Pro 16: 16.2 英寸, 355.7 × 248.1 × 16.8 mm. 差值: 宽 7.8 mm, 深 14.0 mm, 最厚处 9.1 mm. 屏幕英寸只是显示区对角线, 约束不了机身外廓; 买包要拿电脑仓的内尺寸 (毫米数) 去对自己机器规格页的宽 × 深 × 厚.
  - 依据: B.md 尺寸: 别信英寸, 比毫米 (行 38-48)
- **OLD** [查不到] 声明两机具体尺寸查不到, 未答出 keypoint; '+3.5 cm' 规则为 gold 外但不冲突.
  - 答: Lenovo Legion Pro 7i 16 寸 (第 9 代) 和 16 寸 MacBook Pro 的具体外形尺寸差多少: 查不到. 资料只说同样 16 英寸, 轻薄本和游戏本的实际宽度可能不同. 买包该看的数: 实际量出来的电脑长宽 (尤其宽度); 电脑仓的内部最大尺寸; 只给外部尺寸的话, 外部宽度至少要有 电脑宽度 + 约 3.5 cm. 大屁股游戏本最好再看实测或买家反馈.
  - 依据: OLD.md 2.3 (L75-86)

### bag-07 
**问**: 一款包写 1680D 弹道尼龙, 另一款写 1000D Cordura. 是不是 D 数越大面料越耐磨抗撕? 要比结实程度该看什么?

**标准答案**: 不是. 旦尼尔只是纱线线密度 (每 9000 m 纱线的克数, den = 9 × tex), 不直接代表面料强度; 同一个旦数也不代表同一块布, 有经验资料显示标称 1000 d Cordura 的面料实测克重从 325 到 560 gsm 不等. 结实程度要看另测的强度指标, 例如按 ASTM D5034 抓样法测的断裂强力; 抓样法结果与条样法之间没有简单换算关系.

- **kb** [正确] 旦尼尔定义、325–560 gsm 线索、D5034 抓样法及不可与条样法混比, 全中.
  - 答: 不是. 旦尼尔 (D) 的定义只是每 9000 m 纱线的克数 (den = 9 × tex), 是纱线粗细/重量指标, 不直接代表耐磨抗撕; 同一个标号也不代表同一块布, 有线索 (T3 来源) 称标称 '1000 d Cordura' 的面料实测克重从 325 到 560 gsm 不等. 比结实程度应看面料克重和断裂强力, 断裂强力找按 ASTM D5034 抓样法 (grab test) 测的数值; 抓样法与条样法结果没有简单换算关系, 不能混着比. 耐磨、抗撕裂的具体测试方法资料里查不到; 1680D 弹道尼龙的具体数据也查不到.
  - 依据: life/bag/denier-definition, life/bag/pitfall-denier-not-same-fabric, life/bag/fabric-grab-test, life/bag/grab-vs-strip-no-simple-relation
- **B** [正确] 旦尼尔定义 den=9×tex、1000D Cordura 325–560 gsm 线索、D5034 抓样法且与条样法无简单换算, 全中.
  - 答: 不是. 旦尼尔只是线密度 (每 9000 米纱线的克数, den = 9 × tex, ASTM D2260), 不含强度、织法和涂层, 所以光看 D 数推不出 '1680D 比 1000D 结实'. 比结实程度要看织物强力测试, 如 ASTM D5034 抓样法的结果 (如多少 N); 且抓样法和条样法之间没有简单换算, 要用同一测法比. 另有一条仅作线索的说法: 标称 1000D Cordura 的布实测克重 325 到 560 gsm 不等 (未经复核). 两款面料具体的耐磨/抗撕数据查不到.
  - 依据: B.md 材质: 旦尼尔只是线密度 (行 118-125), B.md 先认一条规矩表格 (行 24)
- **OLD** [查不到] 只说 D 是纤度、量化指标与测试标准查不到, 未给定义公式与 D5034, 基本没答出 keypoint.
  - 答: D 数是面料纤度, 数字越小纤维越细, 越大越粗; 外侧耐磨区域更可能用高 D 数材料. 资料没说 D 数越大就一定越耐磨抗撕, 弹道尼龙和 Cordura 的具体区别也查不到. 比结实程度资料能给的: 光写「织物」看不出配方; 同类面料也会因工艺 (比如防泼水涂层) 表现不同; 多看买家秀和细节图, 到手不满意及时退换. 耐磨/抗撕的量化指标或测试标准: 查不到.
  - 依据: OLD.md 2.7 (L133-151)

### bag-08 
**问**: 我把 Peak Design Everyday Backpack 官网描述存成文本, 想用目录里的 claim_lint.py 查它的宣传话术. 它会报哪几条? 退出码是多少?

**标准答案**: 会报 2 条, 退出码 2: (1) 防水: 'weatherproof' 出现 1 次, 但没有 IP 代码/IEC 60529/静水压 mm 或 kPa 之类的测法; (2) 容量: '20L' '30L' 出现 2 次, 但没有 ASTM F2153. 不会报军规、适配尺寸、面料强度 (文本里没有这些声明).

- **kb** [部分] 答出 2 条与退出码 2 及退出码含义, 但未说明防水需 IP/IEC 60529/静水压、容量需 ASTM F2153, 也未写 20L/30L.
  - 答: 按知识库记录 (2026-11-09 本地 Python 3.12 实测): claim_lint.py 对 peakdesign_everyday_backpack.txt 报两条: 防水 (如 'two weatherproof UltraZips' 没带测法) 和容量; 退出码 2 (退出码 = 未带测法的声明条数). 用法: python3 kb/raw/2026-11-08-bag/claim_lint.py < 商品页文本.txt. 注: 脚本本身不在可用资料目录里, 此结果来自卡片的实测记录, 未能重新运行; 若你存的文本与记录时不同, 结果可能不同. 每条报告的完整输出原文查不到.
  - 依据: life/bag/merchant-claim-lint
- **B** [部分] 答出 2 条、退出码 2、weatherproof UltraZips 与 20L/30L, 但容量条未指出缺 ASTM F2153; 另称脚本名为 bag-claim-lint 属 gold 外说法.
  - 答: 资料里的脚本标为 bag-claim-lint (不是 claim_lint.py, 目录里 claim_lint.py 是否就是它、你存的文本是否与作者用的原文一致, 查不到). 作者用 Peak Design Everyday Backpack 官方描述原文跑的结果: 报 2 条 —— '防水: 声明出现 1 次, 未找到测法' ('two weatherproof UltraZips' 没给 IP 代码) 和 '容量: 声明出现 2 次, 未找到测法' ('20L / 30L' 没说按什么测); 退出码 2 (退出码等于报出条数). 若你存的文本与该原文不同, 结果可能不同.
  - 依据: B.md 把检查写成脚本 (行 136-176)
- **OLD** [查不到] 声明资料没提 claim_lint.py 和 Peak Design.
  - 答: 查不到. 资料没提 claim_lint.py, 也没提 Peak Design Everyday Backpack, 不知道它会报哪几条、退出码是多少.
  - 依据: 

### bag-09 (原件未覆盖)
**问**: 我是体重 75 kg 的成年人, 每天背电脑包通勤, 包连电脑总重最多压到多少公斤比较合适?

**标准答案**: 原件未覆盖: 原件只有儿童与青少年的背包负重比例 (AAP 10% 到 20%, AAOS 15%, NSC 转述 5% 到 10%), 没有针对成人的 T1/T2 负重占体重比例建议值. 正确表现是说明查不到成人标准, 不应把儿童比例套用成成人的具体公斤数.

- **kb** [正确拒答] 明确查不到, 列出儿童比例并说明不适用成人, 未给公斤数.
  - 答: 查不到. 资料里的背负比例只针对儿童与青少年 (AAP 写不超过体重 10%–20%, AAOS 写 15%, NSC 转述 AAP 为 5%–10%, 三者互不一致), 不适用于成人, 不能用来算 75 kg 成人的上限. 资料里能用的只有背法建议: 选宽而有衬垫的双肩带, 两条肩带都用, 电脑等重物靠近背部中央.
  - 依据: life/bag/pitfall-load-ratio-inconsistent, life/bag/kids-backpack-load-ratio, life/bag/backpack-straps-heavy-center
- **B** [正确拒答] 明确成人建议查不到, 附带儿童比例并说明不适用, 未给公斤数.
  - 答: 查不到. 资料明确说没找到成人通勤背电脑的负重建议数值, 只有儿童/青少年语境的数据 (AAP: 体重的 10% 到 20%; AAOS: 不超过 15%), 不适用于成人, 不能据此给出 75 kg 成人的上限. 资料中可用的通用做法仅有: 宽而有衬垫的双肩带, 两条都背上, 重物贴近背部中央.
  - 依据: B.md 背负: 只有儿童数据 (行 127-134)
- **OLD** [正确拒答] 明确没给按体重算的上限, 未给数字.
  - 答: 查不到. 资料没给按体重算的背包负重上限. 只说负重一上来, 肩带和背垫设计就更重要.
  - 依据: 

### bag-10 (原件未覆盖)
**问**: 电脑仓的衬垫至少要多厚、底部悬空 (false bottom) 离地要多高, 包掉地上时电脑才不会摔坏?

**标准答案**: 原件未覆盖: 没有任何关于电脑仓衬垫厚度、悬空底离地高度与跌落保护关系的标准或可复核测试数据; MIL-STD-810H 也只读了 Foreword, 未读 Method 516 (冲击) 细节. 正确表现是说明没有依据, 不给具体毫米数.

- **kb** [正确拒答] 明确资料里没有衬垫厚度/悬空高度数据.
  - 答: 查不到. 资料里没有电脑仓衬垫厚度或 false bottom 离地高度的标准或数据, 也没有跌落防护与这两项关系的内容.
  - 依据: 
- **B** [正确拒答] 明确没有标准或可复核数据, 指出需 Method 516 类报告, 未给数值.
  - 答: 查不到. 资料标注 [不确定]: 悬空底和衬垫厚度能把跌落冲击降多少, 没找到标准或可复核的测试数据; 要确认需 MIL-STD-810H Method 516 之类的剪裁报告或厂商公开的跌落测试. 资料未给出任何衬垫厚度或悬空高度的数值.
  - 依据: B.md 防护: 军规与防水 TIP (行 97-98)
- **OLD** [正确拒答] 明确没有具体数, 仅附定性描述, 未给毫米数 (定性称悬空底 '更安全' 略越界但非具体断言).
  - 答: 衬垫最少多厚、架空底部离地多高: 都查不到具体数. 资料只是定性地说: 厚实有弹性的材料、独立电脑仓、架空底部 (电脑仓底部留出缓冲区) 能让包垂直落地时更安全, 代价是更重更厚. 怎样才能保证电脑摔不坏: 查不到.
  - 依据: OLD.md 2.4 (L88-98), OLD.md 0x01 第5条 (L39)

## crawler

### crawler-01 
**问**: 我用 httpx.AsyncClient 把 500 个 URL 一次性 asyncio.gather 出去, 目标站没挂, 但后面一大半请求报 PoolTimeout. 原因是什么, 代码该怎么改?

**标准答案**: httpx 连接池默认 max_connections=100, 且默认对所有操作有 5 秒超时, 其中 pool 超时是从池里拿连接的最长等待. 一次 gather 几百个请求, 超出 100 个的请求在池外排队等连接, 等超过 pool 超时就抛 PoolTimeout (实测 700 个打 1 秒延迟服务: 198 成功、502 PoolTimeout). 改法: 外面套 asyncio.Semaphore 把并发限在池大小内 (Semaphore(100) 后 700 个全成功, 7.8s); 也可像骨架脚本那样用 httpx.Timeout(10.0, pool=None) 让排队请求不因等连接超时, 并让 Limits(max_connections=...) 与信号量一致.

- **kb** [正确] 4 个要点全答到 (100/20 池上限, 5 秒及 pool 超时, 502 PoolTimeout 与 Semaphore(100) 全成功, 骨架 Timeout(10.0,pool=None)+Limits), 无冲突.
  - 答: 原因: httpx 连接池默认 max_connections=100 (max_keepalive_connections=20, keepalive_expiry=5), 并且 httpx 有单独的 pool 超时 (从连接池拿连接的最长等待, 默认对网络操作强制约 5 秒超时). 一次性 gather 500 个, 只有约 100 个拿到连接, 其余在池外排队等连接, 等超过 pool 超时就抛 PoolTimeout, 不是目标站的问题. 资料实测: AsyncClient 默认配置 gather 700 个打 1 秒延迟服务, 198 个成功, 502 个 PoolTimeout (个数只当量级参考); 套 Semaphore(100) 后 700 个全成功. (对比: aiohttp 超出 limit 的请求会排队而不抛池超时, 所以从 aiohttp 换到 httpx 时尤其要补信号量.)
改法: 1) 外面套 asyncio.Semaphore, 信号量不超过 max_connections; 2) limits = httpx.Limits(max_connections=concurrency) 与信号量对齐; 3) timeout = httpx.Timeout(10.0, pool=None), 让在信号量后排队的请求不会因等连接超时; 4) gather 用 return_exceptions=True 并逐个检查结果里的异常. 资料里的骨架 (httpx + Semaphore + Timeout(10.0, pool=None) + 抖动延迟) concurrency=100 跑 700 个全 ok. 也可以用 httpx.Limits 调大 max_connections.
  - 依据: cs/crawler/httpx-gather-pooltimeout, cs/crawler/httpx-pool-limits-default, cs/crawler/httpx-four-timeouts, cs/crawler/httpx-default-timeout-5s, cs/crawler/httpx-pool-none-behind-semaphore, cs/crawler/httpx-crawl-skeleton, cs/crawler/aiohttp-gather-queues
- **B** [正确] 4 个要点全答到; 附加的 delay=0 跑 9.0 秒、速率公式无法对照 gold 核实, 但与 gold 不冲突.
  - 答: 原因: httpx 默认连接上限总 100 (保活 20), 默认超时是 5 秒无活动且包含 pool 等待. 一次性 gather 时所有请求同时抢 100 个连接, 排在后面几波的请求等连接超过 5 秒就被抛 PoolTimeout (文中实测 700 个请求 198 成功、502 个 PoolTimeout). 改法: 并发交给 asyncio.Semaphore 控制, 并让排队不计超时: timeout = httpx.Timeout(10.0, pool=None); limits = httpx.Limits(max_connections=concurrency); 用 async with httpx.AsyncClient(timeout=timeout, limits=limits, follow_redirects=True) 建 client, 每个请求在 async with sem: 里发出 (可加 delay+抖动), 最后 asyncio.gather(..., return_exceptions=True). 文中实测默认配置 + Semaphore(100) 跑 700 个全成功 (7.8 秒); concurrency=100, delay=0 也全成功 (9.0 秒). 整体速率约为 concurrency / (delay + 响应时间).
  - 依据: B.md L42 表格 httpx 行, B.md L59-70 超载时各自怎么坏, B.md L72-108 骨架与会炸的写法, B.md L110
- **OLD** [查不到] 声明资料未讲 PoolTimeout 原因与改法, 未答出任何要点.
  - 答: 查不到. 资料只说 httpx 支持同步/异步 API, 下载层负责连接池、超时、异步并发等 (2.1 表格), 没有讲 PoolTimeout 的原因, 也没有给出连接池上限或并发控制的改法.
  - 依据: OLD.md L60, OLD.md L72

### crawler-02 
**问**: 我的 requests 爬虫跑在定时任务里, 偶尔某个请求卡住十几分钟不动, 也不报错. 为什么? timeout 应该怎么写?

**标准答案**: requests 默认不设超时 (timeout 默认 None), 不显式传 timeout 时请求可以挂几分钟甚至更久. 要显式传 timeout: 单个数值同时作用于 connect 与 read, 也可以传 (connect, read) 元组分开设. 注意 connect 超时按每个 IP 地址计, 域名有多个地址 (如 IPv4 + IPv6) 时总等待会成倍放大, 且 connect/read 超时都不是整个请求的挂钟时间.

- **kb** [正确] 默认 None、单值/元组写法、按 IP 计且非挂钟时间全答到.
  - 答: 原因: requests 默认不设超时 (实测 requests 2.34.2 timeout 默认 None), 不显式传 timeout 时请求可以挂几分钟甚至更久, 也不会报错.
写法: 每个 requests 调用都显式传 timeout. 单值 (如 timeout=10) 同时作用于 connect 和 read; 也可传元组分开设, 如 timeout=(connect秒数, read秒数).
注意: connect 超时按每个 IP 地址计, 域名有多个地址 (如 IPv4+IPv6) 时实际总等待会成倍放大; connect/read 超时都不是挂钟时间, 估算最坏等待要按 IP 个数放大. 如何给整个请求设硬性的总挂钟时限: 资料查不到.
  - 依据: cs/crawler/requests-default-no-timeout, cs/crawler/requests-defaults-measured, cs/crawler/requests-connect-timeout-per-ip
- **B** [部分] 答到默认 None 与按 IP 计, 但声明查不到元组写法, 也没提非挂钟时间; 无错误断言.
  - 答: 原因: requests 默认 timeout=None, 也不重试; 不传 timeout 会无限期挂住, 文档原话是挂几分钟或更久. 写法: 每个请求都要显式传 timeout 参数. 注意 connect 超时按每个 IP 分别算, 域名有 IPv4 和 IPv6 两个地址且都不响应时实际等待会翻倍. 需要重试要自己挂 HTTPAdapter(max_retries=urllib3.util.Retry(total=..., backoff_factor=..., status_forcelist=[...])). 查不到: timeout 参数的具体写法 (如 connect/read 分开的形式) 和推荐取值, 资料未给出.
  - 依据: B.md L41 表格 requests 行, B.md L46-48, B.md L55 TIP
- **OLD** [查不到] 声明没讲卡住原因与 timeout 写法, 未答出要点.
  - 答: 查不到. 资料只说 requests 适合同步脚本, 超时属于下载层职责 (2.1 表格), 没有讲请求为什么会无限卡住, 也没有讲 timeout 参数怎么写.
  - 依据: OLD.md L60, OLD.md L70

### crawler-03 
**问**: 我开了 32 个线程共用一个 requests.Session 爬同一个站, 日志一直刷 'Connection pool is full, discarding connection'. 这会让请求失败吗? 怎么消掉?

**标准答案**: 不会让请求失败, 但说明连接没被复用. requests 默认 DEFAULT_POOLSIZE = 10, 线程数超过池大小时多出的连接用完被丢弃. 实测 32 线程共用默认 Session, 请求全部成功但出现 22 次该警告; 挂一个 HTTPAdapter(pool_maxsize=32) (与线程数一致) 后警告为 0 次.

- **kb** [正确] DEFAULT_POOLSIZE=10、22 次警告、pool_maxsize=32 后 0 次全答到.
  - 答: 不会让请求失败. 资料实测: 32 线程共用一个默认 Session 打同一主机, 请求都成功, 但出现 22 次 'Connection pool is full, discarding connection'. 原因是 requests 默认连接池大小 DEFAULT_POOLSIZE = 10, 小于线程数, 多出来的连接用完后被丢弃 (不放回池复用).
消掉: 给 Session 挂一个 HTTPAdapter(pool_maxsize=32) (调到线程数), 例如 s.mount('https://', HTTPAdapter(pool_maxsize=32)); 实测调到 32 后警告为 0 次. 改完再看日志确认无该警告.
  - 依据: cs/crawler/requests-pool-full-threads, cs/crawler/requests-defaults-measured, cs/crawler/requests-session-reuse
- **B** [部分] 答到池大小 10 与 22 次警告且请求成功, 但声明查不到消除方法, 漏 pool_maxsize=32 后 0 次.
  - 答: 不会让请求失败. requests 默认每主机连接池 10; 文中实测 32 线程共用默认 Session 发 320 个请求, 全部 200, 只出现 22 次 pool is full 警告. 超载时 requests 照常成功, 只是超出池大小的连接用完即丢 (discard). 怎么消掉该警告: 查不到, 资料没有给出调整连接池大小或消除警告的方法.
  - 依据: B.md L41 表格 requests 行, B.md L61-68 超载表与结论
- **OLD** [查不到] 声明资料没讲该警告与池大小调整.
  - 答: 查不到. 资料没有讲 requests.Session 多线程共用、'Connection pool is full' 警告的含义, 以及调整连接池大小的方法.
  - 依据: 

### crawler-04 
**问**: 我用 scrapy startproject 新建项目爬一个站, 发现大概一秒才出一个请求, 远低于我以为的默认并发 16. 是哪些设置在起作用? 调快之前要注意什么?

**标准答案**: startproject 生成的 settings 里 CONCURRENT_REQUESTS_PER_DOMAIN = 1 (全局 fallback 才是 8), DOWNLOAD_DELAY = 1 秒 (fallback 0), 再叠加 DOWNLOAD_DELAY_JITTER 默认 0.5, 即每次延迟在 ±50% 内随机; 响应时间小于 DOWNLOAD_DELAY 时该域实际并发就是 1. CONCURRENT_REQUESTS = 16 是全局上限, 对单个域不起主导作用. 另外 startproject 默认 ROBOTSTXT_OBEY = True. 调快时注意礼貌与被封风险 (官方避免被封建议包括 DOWNLOAD_DELAY 2 秒或以上).

- **kb** [正确] PER_DOMAIN=1/fallback 8、DOWNLOAD_DELAY=1+JITTER 0.5、实际并发 1、CONCURRENT_REQUESTS=16 全答到, 礼貌/被封注意点与 gold 一致.
  - 答: 起作用的设置 (Scrapy 2.20): CONCURRENT_REQUESTS 全局默认确实是 16, 但 startproject 生成的 settings 里 CONCURRENT_REQUESTS_PER_DOMAIN = 1 (全局 fallback 是 8), DOWNLOAD_DELAY = 1 秒 (fallback 0), 再叠加 DOWNLOAD_DELAY_JITTER 默认 0.5, 即延迟在 ±50% 内随机. 响应时间小于 DOWNLOAD_DELAY 时该域实际并发就是 1, 所以单站大约一秒一个请求. 另外 startproject 里 ROBOTSTXT_OBEY = True (fallback False), 会遵守 robots.txt.
调快前注意: 1) 设了 DOWNLOAD_DELAY 时, 只调高 CONCURRENT_REQUESTS_PER_DOMAIN 不能提速, 要一起看 DOWNLOAD_DELAY; 2) 对目标站的压力与被封风险: 资料建议避免被封时首选 USER_AGENT 表明身份, DOWNLOAD_DELAY 2 秒或以上等; 结果时有时无多是目标服务器过载或封禁; 3) 同进程跑多个 crawler 时并发与礼貌设置按 crawler 各算一份, 会成倍放大负载; 4) 若用 scrapy-playwright, PLAYWRIGHT_MAX_PAGES_PER_CONTEXT 默认等于 CONCURRENT_REQUESTS, 调并发要一起看浏览器页数. 建议在项目 settings 里显式写这些值.
  - 依据: cs/crawler/scrapy-concurrent-requests, cs/crawler/scrapy-per-domain-startproject, cs/crawler/scrapy-download-delay, cs/crawler/scrapy-delay-caps-concurrency, cs/crawler/scrapy-robotstxt-obey, cs/crawler/scrapy-avoid-getting-banned, cs/crawler/scrapy-intermittent-is-server, cs/crawler/scrapy-multi-crawler-load, cs/crawler/scrapy-playwright-settings
- **B** [正确] 三个要点全答到, 附加注意点 (多 crawler 叠加、2 秒以上降速) 与 gold 不冲突.
  - 答: startproject 模板生成的 settings.py 很保守: CONCURRENT_REQUESTS=16 (总并发), 但 CONCURRENT_REQUESTS_PER_DOMAIN=1 (全局 fallback 是 8), DOWNLOAD_DELAY=1 (fallback 0, 每域两次请求最小间隔秒数), DOWNLOAD_DELAY_JITTER=0.5 (实际延迟在 0.5~1.5 倍间随机), 另有 ROBOTSTXT_OBEY=True、DOWNLOAD_TIMEOUT=180. 只抓一个站时就是约每秒 1 个请求; 响应时间短于 DOWNLOAD_DELAY 时该域实际并发就是 1, CONCURRENT_REQUESTS=16 只有多域名时才用得满 (单站 10 万页约 27.8 小时). 调快前注意: (1) 同进程跑多个 crawler 时并发、延迟、AutoThrottle 按 crawler 各算一份, 同一 spider 跑两份就是两倍压力, 官方建议要更快就在单个 crawler 上调高 CONCURRENT_REQUESTS; (2) 若用 scrapy-playwright, PLAYWRIGHT_MAX_PAGES_PER_CONTEXT 默认等于 CONCURRENT_REQUESTS, 会跟着变; (3) 速率与 IP 是反爬识别的第一层, Scrapy 文档避免被封的建议是先用 USER_AGENT 表明身份, 其次降速 (2 秒以上)、换 IP.
  - 依据: B.md L44 表格 Scrapy 行, B.md L115-127 Scrapy 的模板很保守, B.md L135-136 TIP, B.md L174, B.md L189-196
- **OLD** [查不到] 只有 Scrapy 职责泛述, 明确说查不到具体设置, 未答出任何要点.
  - 答: 能答的部分: 资料说 Scrapy 属于调度层, 负责 URL 队列、去重、限速、失败重试等, 有 Settings 组件 (0x01 表格, 2.1). 调快之前: 资料在下载层一节提到, 遇到访问限制时应先确认目标站点的授权、服务条款、robots 约束和采集频率边界 (原文是讲 curl_cffi/访问限制的语境). 查不到: 具体是哪些设置让请求变成约一秒一个 (设置名、默认值、startproject 模板里的配置), 以及怎么调.
  - 依据: OLD.md L39, OLD.md L62, OLD.md L76

### crawler-05 
**问**: 我给 httpx.Client 加了 http2=True, 怎么确认请求真的走了 HTTP/2? 需要额外装什么吗?

**标准答案**: 需要 pip install httpx[http2] 并设 http2=True; 只有服务端也支持 HTTP/2 时才会真走, 否则回落 HTTP/1.1. 用 response.http_version 查实际协议. httpx 默认关闭 HTTP/2, 原因是官方认为 HTTP/1.1 实现更成熟.

- **kb** [正确] httpx[http2]+http2=True、服务端不支持回落、用 response.http_version 确认全答到.
  - 答: 需要额外装: pip install httpx[http2] (HTTP/2 默认关闭, 原因是 HTTP/1.1 实现更成熟). 开了 http2=True 不等于真走 HTTP/2, 服务端不支持就回落 HTTP/1.1. 确认方法: 看 response.http_version 得到实际使用的协议 (如 'HTTP/2' 或 'HTTP/1.1').
  - 依据: cs/crawler/httpx-http2-not-guaranteed
- **B** [正确] 三个要点全答到.
  - 答: 需要额外装: 先 pip install httpx[http2], 再在 client 上设 http2=True (httpx 的 HTTP/2 默认关闭). 服务端不支持时会自动回落到 HTTP/1.1, 所以要用 response.http_version 确认实际走的是哪个协议.
  - 依据: B.md L113 TIP
- **OLD** [查不到] 声明资料没讲如何确认与额外安装.
  - 答: 查不到. 资料只确认 httpx 支持 HTTP/1.1 和 HTTP/2, 没有讲怎么确认请求真的走了 HTTP/2, 也没有讲需要额外安装什么.
  - 依据: OLD.md L72

### crawler-06 
**问**: 服务器上设了 https_proxy 环境变量, 同一台机器上 requests 脚本能走代理, 换成 aiohttp 写的爬虫却直连了. 怎么让 aiohttp 也走环境变量里的代理?

**标准答案**: aiohttp 默认不读环境变量代理, 要在 ClientSession 上设 trust_env=True. requests 则在未被覆盖时会读 http_proxy / https_proxy / no_proxy, 所以两者表现不同.

- **kb** [正确] aiohttp 需 trust_env=True、requests 读 http_proxy/https_proxy/no_proxy 均答到.
  - 答: aiohttp 默认不读环境变量里的代理 (requests 则默认读 http_proxy / https_proxy / no_proxy, 所以 requests 脚本能走代理). 让 aiohttp 也走: 创建 ClientSession 时传 trust_env=True, 例如 aiohttp.ClientSession(trust_env=True). 注意: 开启后若也要访问本地服务, 资料提到有环境变量代理时 trust_env=True 的客户端 (资料实测的是 httpx/requests) 会把 127.0.0.1 请求也送去代理, 可设 NO_PROXY=127.0.0.1.
  - 依据: cs/crawler/aiohttp-trust-env-proxy, cs/crawler/requests-proxies-env-override, cs/crawler/httpx-trust-env-localhost
- **B** [正确] 答到 aiohttp 需 trust_env=True 及 requests 默认读环境代理, 与 gold 一致.
  - 答: aiohttp 默认不读环境变量里的代理, 需要设置 trust_env=True 才会读 (requests 和 httpx 默认都读, httpx 默认 trust_env=True). 资料未给出更多代码细节.
  - 依据: B.md L39-43 表格 读环境变量代理 列, B.md L50-52
- **OLD** [查不到] 声明资料没讲 aiohttp 是否读环境变量代理.
  - 答: 查不到. 资料只把代理列为下载层职责, 没有讲 aiohttp 是否读取 https_proxy 环境变量, 也没有讲怎么配置.
  - 依据: OLD.md L60

### crawler-07 
**问**: 目标页的数据是 JS 渲染出来的, Scrapy 抓到的 HTML 里没有. 我打算在 parse 回调里直接起一个 playwright-python 浏览器去渲染, 这样做行吗?

**标准答案**: 不推荐. Scrapy 官方建议先找数据源: 用浏览器 Network 面板找到真正返回数据的请求并复现 (可用 Request.from_curl()), 实在难复现且不在乎速度时才用无头浏览器. 在回调里直接用 playwright-python 会绕过大部分 Scrapy 组件 (中间件、去重等), 官方推荐用 scrapy-playwright: 它是一个下载处理器, 只处理 meta 里带 playwright 键的请求, 其他请求仍走普通下载器, 需要 asyncio reactor.

- **kb** [正确] 先找数据源 (Request.from_curl)、回调直接用会绕过中间件/去重、scrapy-playwright 只处理 meta playwright 键且需 asyncio reactor 全答到.
  - 答: 不推荐. 在 Scrapy 回调里直接用 playwright-python 会绕过大部分 Scrapy 组件 (中间件、去重等), 官方推荐用 scrapy-playwright: 它是一个下载处理器, 只处理 meta 里带 playwright 键的请求, 其余仍走普通下载器; 需要 asyncio reactor (Windows 上 Playwright 必须跑在另一个线程的 ProactorEventLoop). 可用 PLAYWRIGHT_ABORT_REQUEST 丢弃不需要的资源, PLAYWRIGHT_MAX_PAGES_PER_CONTEXT 默认等于 CONCURRENT_REQUESTS.
另外, 官方更推荐先找数据源: 在浏览器 Network 面板找到数据请求并复现 (可用 Request.from_curl()), 实在难复现且不在乎速度时才用无头浏览器. 每个浏览器安装还要占几百 MB 磁盘.
  - 依据: cs/crawler/scrapy-no-direct-playwright, cs/crawler/scrapy-playwright-meta-key, cs/crawler/scrapy-playwright-windows, cs/crawler/scrapy-playwright-settings, cs/crawler/scrapy-dynamic-find-data-source, cs/crawler/playwright-browser-disk
- **B** [正确] 三个要点全答到 (含 meta playwright 与 asyncio reactor 配置), "不行"与 gold"不推荐"方向一致.
  - 答: 不行. Scrapy 官方文档说在回调里直接调 playwright-python 会绕过中间件、去重等大部分组件. 应走 scrapy-playwright: settings.py 里设 DOWNLOAD_HANDLERS = {"https": "scrapy_playwright.handler.ScrapyPlaywrightDownloadHandler"}, TWISTED_REACTOR = "twisted.internet.asyncioreactor.AsyncioSelectorReactor", 可用 PLAYWRIGHT_ABORT_REQUEST 屏蔽图片等资源; spider 里只有 meta={"playwright": True} 的请求走浏览器, 其余仍走普通下载器. 另外上浏览器前建议先去 Network 面板找 XHR/fetch 数据源直接复现 (可用 Request.from_curl()); 即使用浏览器, 也优先用 page.expect_response 截接口响应, 最后才解析渲染后的 DOM.
  - 依据: B.md L17-35 先看数据在哪, B.md L140-174 动态页
- **OLD** [部分] 答到先复现接口请求、浏览器作最后手段 (要点1), 但声明查不到在回调里直接起浏览器的问题与 scrapy-playwright 集成方式; 无错误断言.
  - 答: 能答的部分: 资料建议先看页面真实数据是否来自 XHR/Fetch API, 如果是, 优先复现接口请求, 不要立刻上浏览器自动化; 浏览器层成本最高, 最容易引入稳定性和资源占用问题, 应当作最后手段. 如果确认必须用真实浏览器, Playwright 是新项目的优先评估项, Python API 同时支持 sync 和 async. 另外 Crawlee for Python 覆盖 HTTP-only 和 browser-based 爬取, 可作为希望两者共用框架时的调研选项. 查不到: 在 Scrapy 的 parse 回调里直接起 playwright-python 浏览器是否可行、会有什么问题、该用什么集成方式.
  - 依据: OLD.md 2.1 L66, OLD.md 2.5 L98, OLD.md L94, OLD.md 0x03 L144

### crawler-08 
**问**: 我把 requests 换成 curl_cffi, 用了 impersonate="chrome124" 模仿浏览器指纹, 还是被一个用 Akamai 的站拦. 是我版本写错了还是这条路本身不够?

**标准答案**: 主要是这条路本身不够: curl_cffi 模仿的是浏览器的 TLS/JA3 与 HTTP/2 指纹, 只解决这一层; curl_cffi README 自己 (赞助商段落, 广告性质) 也写着对 Akamai / DataDome / Kasada / Incapsula 这类现代反爬还需要另外的 token. 版本方面: 'chrome124' 是固定版本写法, impersonate="chrome" 会跟随库更新到最新版本, 也可用 curl-cffi update 在不升级库的情况下拉新指纹, 但这不能替代反爬 token.

- **kb** [正确] 只解决 TLS/HTTP2 层且 Akamai 等需另外 token (标为线索)、chrome 跟随最新/chrome124 固定、curl-cffi update 全答到.
  - 答: 主要是这条路本身不够, 不是版本写法错. impersonate="chrome124" 是合法写法, 表示固定版本 (可复现指纹); impersonate="chrome" 则跟随库更新到最新版本; 也可用 `curl-cffi update` 在不升级库的情况下拉取新指纹. 但指纹模仿只解决 TLS/HTTP2 这一层, 对 Akamai / DataDome / Kasada / Incapsula 这类现代反爬, 还需要另外的 token, 只换指纹库不指望能过 (注: 资料对这条标为 B 级, 来源带广告性质, 仅作线索). 具体如何拿到 Akamai 所需的 token: 资料查不到.
  - 依据: cs/crawler/curl-cffi-fingerprint-not-enough, cs/crawler/curl-cffi-impersonate-pin, cs/crawler/curl-cffi-update-fingerprints, cs/crawler/curl-cffi-tls-impersonate
- **B** [正确] 三个要点全答到并标明 README 广告性质, 无冲突断言.
  - 答: 资料倾向于是这条路本身不够, 但证据较弱: curl_cffi 的指纹模仿只管 TLS/JA3 和 HTTP/2 这一层; curl_cffi README 赞助商段落写着对 Akamai / DataDome / Kasada / Incapsula 只做 TLS 指纹不够 (作者注明这是广告, 只当反例线索). 下一层是 JS 挑战与 token, 需要浏览器或第三方服务. 关于版本: 写 "chrome124" 是固定版本指纹, 不写版本号 (impersonate="chrome") 则跟随库更新到最新指纹; 指纹会旧, 可用 curl-cffi update 不升级库拉新指纹. 查不到: 固定版本指纹是否会被目标站拒掉, 资料明说没找到一手数据 [不确定]; 你这个站被拦的具体原因也无法从资料判断.
  - 依据: B.md L185-222 被拦了就一层一层换
- **OLD** [查不到] 只有"不应作默认选项"等泛述, 声明查不到版本写法与对 Akamai 的效果, 未答出要点.
  - 答: 能答的部分: 资料说 curl_cffi 是较特殊的下载层工具, 常用于需要 libcurl/curl-impersonate 能力的场景, 不应当成为默认选项; 遇到访问限制时, 应先确认目标站点的授权、服务条款、robots 约束和采集频率边界. 查不到: impersonate="chrome124" 这个写法是否正确、可用的版本名, 以及对 Akamai 的效果和这条路是否足够.
  - 依据: OLD.md 2.2 L76

### crawler-09 (原件未覆盖)
**问**: aiohttp 怎么开启 HTTP/2? 我想让 aiohttp 爬虫像 httpx 那样走 h2.

**标准答案**: 原件未覆盖: 没有找到 aiohttp 官方关于 HTTP/2 的陈述; '不支持 HTTP/2' 只出现在 curl_cffi README 的对比表 (第三方说法), 原件正文未采用. 正确表现是说查不到 aiohttp 官方说法, 不给具体开启方法或断言.

- **kb** [正确拒答] 明确说查不到 aiohttp 开启 HTTP/2 的方法, 只附 httpx/curl_cffi 相关事实, 未越界.
  - 答: 查不到. 资料中没有 aiohttp 开启 HTTP/2 的方法或 aiohttp 是否支持 HTTP/2 的信息 (资料里只写了 httpx 用 pip install httpx[http2] + http2=True, 以及 curl_cffi 支持 HTTP/2).
  - 依据: 
- **B** [正确拒答] 明确说查不到, 只附 httpx 的开启方法, 未越界.
  - 答: 查不到. 资料只介绍了 httpx 开 HTTP/2 的方法 (pip install httpx[http2] + http2=True), 没有任何关于 aiohttp 开启 HTTP/2 的内容.
  - 依据: B.md 全文 (仅 L113 提到 httpx 的 HTTP/2)
- **OLD** [正确拒答] 明确说没讲 aiohttp 能否或怎么开启 HTTP/2, 未给具体断言.
  - 答: 查不到. 资料只说 aiohttp 是 asyncio 生态的成熟 HTTP client/server, 明确写支持 HTTP/2 的是 httpx; 没有讲 aiohttp 能否或怎么开启 HTTP/2.
  - 依据: OLD.md L72, OLD.md L74

### crawler-10 (原件未覆盖)
**问**: 用 curl_cffi impersonate="chrome" 能不能直接过 Cloudflare 的 5 秒盾 / Turnstile 验证?

**标准答案**: 原件未覆盖: 原件没有对任何真实带反爬的站点做测试, 没有 '某指纹被某厂商拦截/放行' 的一手数据, 也没提 Cloudflare. 只有 README 广告段说仅 TLS 指纹对现代反爬不够. 正确表现是说不确定, 不给能过/不能过的具体断言.

- **kb** [正确拒答] 明确说资料未提 Cloudflare/Turnstile、能否通过查不到, 附带的指纹只管一层属资料内事实.
  - 答: Cloudflare 5 秒盾 / Turnstile 能否用 curl_cffi 通过: 资料查不到 (资料没有提到 Cloudflare 或 Turnstile). 资料能给的相关信息: curl_cffi 可模仿浏览器的 TLS/JA3 与 HTTP/2 指纹, impersonate="chrome" 跟随库更新到最新版本; 但指纹模仿只解决 TLS/HTTP2 这一层, 资料举例 Akamai / DataDome / Kasada / Incapsula 这类反爬还需要另外的 token (B 级线索). 能否推广到 Cloudflare: 查不到.
  - 依据: cs/crawler/curl-cffi-tls-impersonate, cs/crawler/curl-cffi-impersonate-pin, cs/crawler/curl-cffi-fingerprint-not-enough
- **B** [正确拒答] 明确说查不到能否通过, 只给分层的一般框架, 未断言能过或不能过.
  - 答: 资料没有提到 Cloudflare、5 秒盾或 Turnstile, 针对它们能否通过: 查不到. 资料只给出一般框架: curl_cffi impersonate 只模仿 TLS/JA3 和 HTTP/2 指纹这一层; JS 挑战与 token 属于更后一层, 要靠浏览器或第三方服务处理. 排查时一次只换一层, 哪层开始稳定拿到数据就停.
  - 依据: B.md L187-222 被拦了就一层一层换
- **OLD** [正确拒答] 明确说资料没讲对 Cloudflare 的效果, 未给能过/不能过断言.
  - 答: 查不到. 资料没有讲 curl_cffi 对 Cloudflare 5 秒盾或 Turnstile 的效果; 只说 curl_cffi 不应作为默认选项, 遇到访问限制应先确认授权、服务条款、robots 和频率边界.
  - 依据: OLD.md L76

## skill

### skill-01 
**问**: 我想把新 skill 的 name 写成 Claude-PDF-Helper, 放在 skills/pdf/ 目录下. 这个名字有什么问题, 该怎么改?

**标准答案**: 有多处问题: name 只能含小写字母、数字与连字符 (1 到 64 字符, 不以连字符开头结尾, 无连续连字符), 大写不行; 不能含保留词 claude / anthropic; 必须与父目录名一致 (目录是 pdf, name 却不是); 命名建议动名词形式, 避免 helper / utils 这类空名. 可改为如 processing-pdfs 并把目录也命名为 processing-pdfs.

- **kb** [正确] 小写字符规则, 保留词 claude/anthropic, 须与目录一致, 动名词命名避免 helper 均答到, 改法与 gold 一致.
  - 答: 问题有三: (1) name 只能含小写字母、数字和连字符, Claude-PDF-Helper 含大写字母; (2) name 不能含保留词 anthropic 或 claude, 这里含 Claude; (3) name 必须与父目录名一致, 而目录是 skills/pdf/, 名字和目录不一致. 另外命名建议用动名词形式, 避免 helper、utils 这类空名. 改法: 改成动名词形式的小写连字符名, 如 processing-pdfs, 并把目录同步改名为 skills/processing-pdfs/ (或者名字与目录取同一个合规名). 其他约束: 1 到 64 字符, 连字符不在首尾、不连续.
  - 依据: cs/skill/name-format, cs/skill/name-reserved-words, cs/skill/naming-gerund
- **B** [部分] 只答到 name 须与目录名一致这一点, 大小写字符规则与保留词 claude 均称查不到, 未提动名词命名; 无错误断言.
  - 答: 资料能确认的问题: 规范要求 SKILL.md 的 name 与所在目录名一致, name 写 Claude-PDF-Helper 而目录叫 pdf, 不一致 (skill_lint.py 会报 [E] name-dir). 改法: 让两者一致, 例如 name 改成 pdf, 或把目录改名成与 name 相同. name 的大小写/字符规则、是否禁止含 'Claude' 等保留词: 查不到.
  - 依据: B.md 第157-158行 体检输出 name-dir, B.md 第172行
- **OLD** [部分] 答到小写/连字符/64 字符与目录一致, 保留词称查不到, 命名建议写成'动词开头短语'而非动名词且未提避免 helper; '不做校验的宿主会静默失败'为 gold 外说法但不冲突.
  - 答: 问题: (1) 规范只允许小写字母、数字、连字符, Claude-PDF-Helper 含大写字母, 不合法; (2) name 必须与父目录名完全一致, 目录是 pdf 而 name 是 Claude-PDF-Helper, 不一致, 在不做校验的宿主里会直接失败, 且不报错(静默失败). 另需满足: 不超过 64 字符、不以连字符开头或结尾、不能有连续连字符. 改法: name 与目录名保持一致, 例如 name 写 pdf; 或者把目录和 name 一起改成同一个合法名字(全小写+连字符). 命名风格上官方建议优先用动词开头的短语描述动作, 需要时用工具名做前缀(如 gh-address-comments). 至于名字里含 Claude 这类词是否有额外限制: 查不到.
  - 依据: OLD.md 五、命名 (L183-193)

### skill-02 
**问**: 我在 Claude Code 里给一个部署 skill 加了 disable-model-invocation: true, 但 description 里仍写了一堆 'Use this skill when...' 的触发词. 这些触发词还起作用吗? 这个 skill 以后要上传到 claude.ai 会有问题吗?

**标准答案**: 不起作用: disable-model-invocation: true 时 description 不进上下文, 只能由用户用 /name 调用 (反过来 user-invocable: false 是只有 Claude 能调). 上传 claude.ai / Skills API 或用 package_skill.py 打包只接受规范内 6 个字段 (name, description, license, compatibility, metadata, allowed-tools), disable-model-invocation 是 Claude Code 专有字段, 多出的字段会硬报错 'Unexpected key(s) in SKILL.md frontmatter', 上传前要去掉.

- **kb** [正确] 三个要点全部答到, 包括 6 个字段名单和 Unexpected key(s) 报错.
  - 答: 不起作用. 在 Claude Code 里 disable-model-invocation: true 时 description 不进上下文, 这个 skill 只能由用户用 /name 调用, 所以 description 里的 'Use this skill when...' 触发词对自动触发没有作用. 上传 claude.ai 会有问题: disable-model-invocation 是规范外字段, Claude Code 接受它, 但上传 claude.ai (以及 Skills API、package_skill.py 打包) 只接受规范内 6 个字段 (name、description、license、compatibility、metadata、allowed-tools), 多出的字段会硬报错 Unexpected key(s) in SKILL.md frontmatter. 上传前要删掉该字段 (删掉后它在那边就不再有'禁止模型调用'的效果).
  - 依据: cs/skill/claude-code-invocation-control, cs/skill/nonspec-fields-hard-error, cs/skill/frontmatter-fields
- **B** [部分] 答到 description 不进上下文/只能手动调用, 以及上传只接受 6 个规范字段多出报错, 但 6 个字段是哪些称查不到, 未给出 Unexpected key(s) 报错原文.
  - 答: 不起作用: 设了 disable-model-invocation: true 后, Claude Code 不把 description 放进上下文 (只能手动调用), 所以那些 'Use this skill when...' 触发词对自动触发没有作用. 上传 claude.ai 会有问题: disable-model-invocation 是规范外字段, claude.ai 上传 / Skills API 打包会直接报错; 想上传 claude.ai, frontmatter 只能留规范里的 6 个字段, 多一个就报错, 需要删掉该字段. (规范 6 个字段具体是哪些: 查不到.)
  - 依据: B.md 第164-165行 体检输出 non-spec-keys / manual-only, B.md 第179-180行 表格, B.md 第184行
- **OLD** [查不到] 该字段的效果和上传 claude.ai 的问题都称查不到, 没有答到任何要点.
  - 答: 资料能给的部分: 资料说触发完全由 name 与 description 决定, 模型只看这两个字段判断是否加载, 所以一般情况下描述里的 'Use this skill when...' 是真正的触发器. 但 disable-model-invocation: true 这个字段在 Claude Code 中的效果(是否让描述里的触发词失效): 查不到. 上传到 claude.ai 是否会有问题(该字段是否被支持/是否报错): 查不到. 资料只提到主流宿主遇到不合规条目时常见行为是丢弃并只写一条日志, 模型侧静默, 但未说明该字段是否属于不合规.
  - 依据: OLD.md 二、触发工程 (L74-128), 0x01 核心结论 第3条 (L35)

### skill-03 
**问**: 我的 SKILL.md 已经写到 800 多行了, 里面有财务、销售、法务三块规则. 该怎么拆?

**标准答案**: SKILL.md 正文建议少于 500 行, 接近时拆文件. 按领域拆参考文件 (如 reference/finance.md, reference/sales.md, reference/legal.md), 让无关领域不占上下文; 引用保持离 SKILL.md 只深一层, 不要嵌套引用 (否则 Claude 可能用 head -100 预览而读不全); 超过 100 行的参考文件顶部放目录. 参考文件在被读之前不占上下文.

- **kb** [正确] 四个要点全部答到.
  - 答: SKILL.md 正文应 < 500 行 (正文建议 < 5000 token), 800 多行需要拆. 按领域拆成参考文件, 如 reference/finance.md、reference/sales.md、reference/legal.md, 一个领域一份, 这样无关领域在被读之前不占上下文 (零占用). SKILL.md 正文只留通用流程、重要指令 (放靠前) 和指向各参考文件的链接. 引用保持离 SKILL.md 只深一层 (所有参考文件都由 SKILL.md 直接链接, 不要文件引文件, 否则 Claude 可能只用 head -100 预览而读不全). 超过 100 行的参考文件在顶部放目录. 能交给脚本的步骤放进 scripts/, 只有输出占 token.
  - 依据: cs/skill/body-under-500-lines, cs/skill/split-references-by-domain, cs/skill/references-one-level-deep, cs/skill/reference-toc-over-100, cs/skill/references-no-cost-until-read, cs/skill/three-tier-loading, cs/skill/claude-code-instructions-upfront-hooks
- **B** [正确] <500 行, 按领域拆, 只深一层且嵌套时 head -100 预览, >100 行放目录, 全部答到.
  - 答: SKILL.md 正文应少于 500 行 (Anthropic、agentskills.io、Claude Code 同一个数), 只放导航 + 硬规则. 把财务、销售、法务按领域拆成独立参考文件, 如 reference/finance.md、reference/sales.md、reference/legal.md, 用到才读, 无关领域零占用. 所有参考文件都从 SKILL.md 直接链接, 引用只深一层; 二级引用的文件会被 head -100 式预览, 读不全. 单个参考文件超过 100 行就在顶部放目录. 能机械执行/易错的部分可写成 scripts/ 下的脚本 (只运行, 源码不进上下文).
  - 依据: B.md '正文和附属文件怎么分' 第66-77行, B.md '每一层都是保头丢尾' 第56-60行, B.md 第83-85行
- **OLD** [部分] 答到 <500 行, 按领域拆, 只深一层, >100 行放目录, 但没提嵌套引用会被 head -100 预览而读不全; 另把 500 行写成'硬约束', 比 gold 的'建议'说得更死.
  - 答: 800 多行已超过正文硬约束(少于 500 行且少于 5000 token), 而且触发时整篇正文都会进上下文. 正确做法是分层而不是压缩: 按'这段内容是不是每次触发都需要'判断, 是就留在 SKILL.md 正文, 否就外移到 references/. 财务、销售、法务若是互斥场景, 拆成 references/ 下各自独立的文件(如 references/finance.md、references/sales.md、references/legal.md)能真正降低单次消耗, 因为模型只加载实际用到的那份; 若每次都要一起读, 拆分只增加跳转成本. 要求: 正文只留核心流程与选择指引, 细节、schema、长示例移到参考文件; 信息只存一份, 正文与 references/ 不重复; 每个拆出的文件都必须在正文中被引用并说明何时读; 引用保持从 SKILL.md 出发一层深, 不链式嵌套; 超过约 100 行的参考文件开头放目录, 超过一万词的给检索模式(如 grep 关键词). 需要确定性执行的脆弱步骤可固化为 scripts/. 拆完跑校验(行数、引用是否齐全等).
  - 依据: OLD.md 一、上下文预算 (L51-72), 四、正文怎么写 第2-5条 (L163-177), 八、完整工作流 (L250-266)

### skill-04 
**问**: 在 Claude Code 长会话里, 前面调用过的 skill 在自动压缩 (auto-compact) 之后好像不再遵守其中后半部分的规则了. 为什么? 怎么写能缓解?

**标准答案**: 自动压缩后, 每个 skill 只重新附上最近一次调用的前 5000 token, 所有 skill 合计不超过 25000 token, 从最近调用的开始填, 旧 skill 可能整个被丢. 所以靠后的内容可能丢失. 缓解: 重要指令放在 SKILL.md 靠前处; 每次都必须成立的规则移进 hook. 另外 skill 内容是跨轮常驻的消息, 应写成持续生效的指令而非一次性步骤.

- **kb** [正确] 三个要点全部答到, 包括按最近调用填充, 旧 skill 可能整个被丢.
  - 答: 原因: Claude Code 自动压缩后, 每个 skill 只重新附上最近一次调用的前 5000 token, 所有 skill 合计 25000 token, 从最近调用的开始填, 旧 skill 可能整个被丢. 所以长 skill 的后半部分在压缩后就不在上下文里了. 缓解: 把关键/重要指令放在 SKILL.md 靠前处 (落在前 5000 token 内); 每次都必须成立的规则移进 hook; 正文写成持续生效的指令而非一次性步骤, 删掉不需要常驻的行 (被调用的 skill 作为一条消息跨轮常驻, 每行都是逐轮成本); 压缩后某 skill 整个失效时, 检查是否因超出合计 25000 token 被丢.
  - 依据: cs/skill/claude-code-compaction-reattach, cs/skill/claude-code-instructions-upfront-hooks, cs/skill/claude-code-skill-persists
- **B** [部分] 答到每个 skill 5000 / 合计 25000, 要点放前面并移进 hook, 持续生效的写法, 但没提从最近调用的开始填以及旧 skill 可能整个被丢.
  - 答: 原因: Claude Code 自动压缩后, 每个 skill 只保留正文前 5000 token, 所有 skill 合计 25000 token, 截断保留开头丢尾部, 所以后半部分规则被丢了. (另: skill 被调用后正文作为一条消息留在对话里, 后续轮次不会重读文件.) 缓解: 把每次都要遵守的规则放在 SKILL.md 正文最前面; 控制正文长度 (建议 < 5000 token、< 500 行), 细节拆到按需读的参考文件; 指令写成整段任务都生效的那种; 每次都必须成立的规则可以不放 skill, 改放常驻上下文 (如 AGENTS.md)、脚本或 hook.
  - 依据: B.md 第17-26行 加载顺序图及注, B.md 第56-62行 保头丢尾表, B.md 第105-107行
- **OLD** [查不到] 声明资料没有 auto-compact 相关内容, 没有答到要点.
  - 答: 查不到. 资料没有涉及 Claude Code 长会话、auto-compact 自动压缩后 skill 内容如何保留, 也没有给出针对压缩的缓解写法. 资料只说触发时正文会整篇读入上下文, 但未讨论压缩之后的行为.
  - 依据: 

### skill-05 
**问**: 我要给自己 skill 的 description 做触发评测: 准备多少条查询, 正负例怎么配, 每条跑几次, 触发率多少算触发?

**标准答案**: 约 20 条查询, 其中 8 到 10 条应触发、8 到 10 条不应触发, 负例要选与 skill 共享关键词的近似请求. 每条跑 3 次算触发率, 默认阈值 0.5 (20 × 3 = 60 次调用). 改描述时按约 60% 训练 / 40% 验证划分, 只用训练集失败来改, 按验证集通过率选最佳版本, 通常 5 轮足够.

- **kb** [正确] 三个要点全部答到 (含按验证集选最佳); 回归检查条数和 claude -p 判定是 gold 外的补充, 与 gold 不冲突.
  - 答: 约 20 条查询: 8 到 10 条应触发, 8 到 10 条不应触发; 负例要选共享关键词的近似请求. 每条跑 3 次算触发率, 触发率 ≥ 0.5 (默认阈值) 记为触发; 20 条 x 3 次 = 60 次调用. 调优时按约 60% 训练集 / 40% 验证集划分, 只用训练集失败改描述, 按验证集通过率选最佳版本, 通常 5 轮足够; 不要把失败查询的具体关键词塞进描述 (过拟合). 正例里别放一步就能完成的请求. 只做回归检查的话 10 到 20 条即可, 至少一条 should_trigger=false 的负例. Claude Code 中可用 claude -p ... --output-format json 看有无 Skill 工具调用来判定是否触发.
  - 依据: cs/skill/trigger-eval-query-set, cs/skill/trigger-eval-3-runs, cs/skill/trigger-train-val-split, cs/skill/small-eval-with-negative, cs/skill/one-step-tasks-not-triggered, cs/skill/trigger-no-keyword-stuffing, cs/skill/claude-p-trigger-check
- **B** [部分] 答到 20 条, 正负各 8 到 10 条, 近似负例, 3 次, 0.5, 60 次, 60/40, 只用训练集失败改, 约 5 轮, 但没提按验证集通过率选最佳版本; '另加 5 到 10 条新查询复核'在 gold 之外, 无法核实.
  - 答: 按 agentskills.io 建议: 约 20 条查询, 正例 (should_trigger=true) 和负例 (should_trigger=false) 各 8 到 10 条; 正例用真实用户会打的话 (带文件路径、口语、不点名 skill), 负例用共享关键词但需求不同的近似请求. 每条重复 3 次, 触发率阈值 0.5 (达到即算触发). 每轮 60 次运行, 约 5 轮, 训练 60% / 验证 40%, 合计约 300 次, 另加 5 到 10 条新查询各跑 3 次复核. 只用训练集失败去改描述, 不要把失败查询关键词原样塞进描述 (过拟合); 每次运行从干净上下文开始.
  - 依据: B.md '触发评测与输出评测' 第114-140行
- **OLD** [查不到] 声明资料没有触发评测的数字, 没有答到要点.
  - 答: 查不到. 资料没有给出描述触发评测的查询条数、正负例配比、每条运行次数或触发率阈值. 资料的验证部分(六)只讲 forward-test 的污染控制: 让被测 agent 不知道自己在被测、用新鲜会话、传原始产物、不给预期答案等, 不含上述数字.
  - 依据: 

### skill-06 
**问**: 我在 Claude Code 里装了一百多个 skill, 发现有些 skill 明明描述写得很清楚却从不自动触发. 会不会是描述被截掉或压根没进列表? 具体限制是多少?

**标准答案**: 可能. Claude Code 列表里每个 skill 的 description 与 when_to_use 合计截断于 1536 字符 (可用 skillListingMaxDescChars 修改); skill 列表整体预算为上下文窗口的 1%, 超出时先丢最少被调用的 skill 的描述, 名字始终保留. 所以描述写长或 skill 太多时, 冷门 skill 可能只剩名字而无法按描述匹配. 关键用例宜放在描述前面.

- **kb** [正确] 三个要点全部答到, 包括 skillListingMaxDescChars.
  - 答: 有可能. Claude Code 的 skill 列表整体预算为上下文窗口的 1%, 超出时先丢最少被调用的 skill 的描述 (名字始终保留), 所以装一百多个时少用 skill 的描述可能根本没进列表; 另外单个 skill 的 description 与 when_to_use 合计在列表中截断于 1536 字符, 可用 skillListingMaxDescChars 修改. 对策: 卸掉不用的 skill 给预算腾地方, 关键触发信息写在描述前部且控制长度. 另两个可排查点: frontmatter YAML 解析失败时正文仍加载但元数据为空, /name 能用而不会按 description 自动匹配; 一步就能完成的请求即使描述匹配也可能不触发.
  - 依据: cs/skill/claude-code-listing-budget, cs/skill/claude-code-desc-1536, cs/skill/yaml-parse-fail-empty-metadata, cs/skill/one-step-tasks-not-triggered
- **B** [部分] 答到 1536 字符, 1% 预算, 先丢最少调用的描述且名字保留, 关键词前置, 但没提可用 skillListingMaxDescChars 修改.
  - 答: 会. Claude Code 的 skill 列表预算是上下文窗口的 1%, 单条 description + when_to_use 共上限 1536 字符 (超出被截到 1536); 列表溢出时先丢最少被调用的 skill 的描述, 名字保留 (此时只有名字没描述, 难以自动触发). 另外若该 skill 设了 disable-model-invocation: true, description 根本不进上下文. 对策: 关键用途和触发词写在 description 最前面, 先把触发词挪到第一句而不是把描述写长. 另注: 规范给 description 的硬上限是 1024 字符.
  - 依据: B.md 第43-50行 宿主表, B.md 第58行, B.md 第165行, B.md 第188行
- **OLD** [查不到] 1536 截断和 1% 预算等具体限制都称查不到, 只给了 1024 上限和静默丢弃等 gold 外信息, 没有答到要点.
  - 答: 资料能答的部分: 规范限制 description 在 1024 字符内; 第一级(name+description)预算约 100 词, 每个已安装 skill 都在启动时预载, 成本是乘性的. 另外主流宿主加载器遇到不合规条目(名字不合法、描述缺失、布尔值写错等)常见行为是丢弃并只写一条日志, skill 从目录中静默消失, 模型看不到错误. 所以'压根没进列表'是可能的原因之一, 建议用校验脚本检查. 查不到的部分: Claude Code 对 skill 列表的总量/条数限制、描述在列表中被截断的具体长度、装一百多个 skill 时的具体行为.
  - 依据: OLD.md 一、上下文预算 表格 (L55-61), 二、推论三 (L126-128), 七、把规则变成代码 (L227-231)

### skill-07 
**问**: 我写了个 PDF 处理 skill, 描述里明明写了 'read PDF', 可用户说 '帮我读一下这个 PDF' 时它不触发. 是描述写坏了吗? 描述应该怎么写?

**标准答案**: 不一定是描述的问题: agent 通常只为自己独立做不好的任务去查 skill, 一步就能完成的简单请求 (如 'read this PDF') 即使描述完全匹配也可能不触发. 描述写法: 用第三人称 (它被注入系统提示, 人称不一致会导致发现问题), 用 'Use this skill when...' 祈使句、聚焦用户意图, 可以写得 pushy, 把用户没点名领域的情形也列进去. 不要把失败查询里的具体关键词直接塞进描述, 那是过拟合.

- **kb** [正确] 三个要点全部答到, 并提醒不要把失败查询关键词塞进描述.
  - 答: 不一定是描述写坏了. agent 通常只为自己独立做不好的任务查 skill, 像 'read this PDF' 这种一步就能完成的请求, 即使描述完全匹配也可能不触发. 描述写法: 写清做什么与什么时候用 (1 到 1024 字符, 非空); 用 'Use this skill when…' 祈使句, 聚焦用户意图, 可以写得 pushy, 把用户没点名领域的情形也列进去; 人称保持一致 (资料写为第三人称, 因为描述被注入系统提示); 关键用例与触发词放在前面; 不要把失败查询里的具体关键词塞进描述 (过拟合). 也应让 skill 针对的是一步做不好的任务, 而不是简单读取; 触发评测里也别拿这种一步请求当正例.
  - 依据: cs/skill/one-step-tasks-not-triggered, cs/skill/description-imperative-pushy, cs/skill/description-third-person, cs/skill/description-length, cs/skill/trigger-no-keyword-stuffing, cs/skill/codex-frontload-triggers
- **B** [部分] 答到一步就能完成的请求不触发, pushy 写法, 第三人称, 但没说明第三人称的原因 (会被注入系统提示), 也没讲聚焦用户意图的 'Use this skill when...' 祈使句.
  - 答: 不一定是描述写坏了: description 写得再准也不保证触发, agentskills.io 举的例子正是 'read this PDF' —— 一步就能做完的请求 agent 自己就处理了, 不会去查 PDF skill. 描述写法: 前半句用第三人称写能力, 后半句用 Use when 写时机, 例如 'Extracts text and tables from PDF files, fills forms, and merges documents. Use when working with PDF files or when the user mentions PDFs, forms, or document extraction.'; 关键用途和触发词放最前面; 可写得 pushy 一点, 把用户没点名领域的情形也列进去; 不超过 1024 字符. 若必须稳定使用, 可在 AGENTS.md 里明确要求调用 (Vercel 评测中触发率升到 95% 以上), 或把必须成立的知识放常驻上下文/脚本/hook.
  - 依据: B.md 第33-52行 'description 决定会不会被用', B.md 第91-107行
- **OLD** [错误] '很可能是描述写得不够'和'应把用户的真实说法 (如读一下这个 PDF) 写进描述'与 gold 冲突: gold 说一步就能完成的请求即使描述匹配也可能不触发, 不一定是描述问题, 且塞失败查询关键词属过拟合.
  - 答: 很可能是描述写得不够: 模型只看 name 和 description 做语义匹配决定是否加载, 描述里包含用户实际会用的词至关重要, '用户可能说的原话'才是真正的触发器. 只写 'read PDF' 这类简短能力描述属于反模式(太短、缺'什么时候用'), 会静默不触发. 写法公式: [做什么: 具体的、含关键名词的能力清单] + [什么时候用: 触发场景 + 用户可能说的原话]. 资料的好例: 'Extracts text and tables from PDF files, fills PDF forms, and merges multiple PDFs. Use when working with PDF documents or when the user mentions PDFs, forms, or document extraction.' 据此应把用户的真实说法(如'读一下这个 PDF'这类表述)写进描述; 不要把触发条件写在正文的 When to Use 段(正文触发前不可见). 描述上限 1024 字符, 常见问题是太短而非太长. 另需排除 name 不合法/与目录名不一致等导致 skill 被静默丢弃的情况.
  - 依据: OLD.md 二、触发工程 (L74-128)

### skill-08 
**问**: 我从 GitHub 拿了一个社区 skill: 目录叫 AI-Animation-Skill, SKILL.md 里 name 写的是 science-content-ppt, 正文还把默认输出路径写成 /home/mt/桌面/AI_Animation.html. 用 check/skill_lint.py 体检会报什么? 该怎么修?

**标准答案**: skill_lint.py 会报一条 E 级 name-dir (name 'science-content-ppt' 与目录名 'AI-Animation-Skill' 不一致, 规范要求一致) 和一条 W 级 abs-path (正文写死绝对路径 /home/mt/桌面/AI_Animation.html, 换机器就失效); 因有 E 级, 退出码为 1. 其余为 I 级信息 (description 190 字符, 正文 236 行 < 500, 直接引用 0 个存在的文件). 修法: 把目录改名为 science-content-ppt (目录名也须是合法 name, AI-Animation-Skill 含大写不合格), 并把输出路径改为相对路径或由用户指定.

- **kb** [部分] 答到 name 与目录不一致且目录名含大写不合规, 但 lint 报什么 (E/W 级别, 退出码) 和绝对路径问题都称查不到; 无错误断言.
  - 答: check/skill_lint.py 这个工具具体会报什么: 查不到 (知识库里没有它的资料). 按资料里的规范能指出的问题: (1) name 必须与父目录名一致, 目录 AI-Animation-Skill 与 name science-content-ppt 不一致; (2) 目录名 AI-Animation-Skill 含大写字母, 不是合规的 name (只允许小写字母、数字、连字符). 修法: 把目录改名为与 name 一致的合规名 (如 science-content-ppt), 或两者统一成同一个小写连字符名 (命名建议用动名词形式). 正文把默认输出路径写死成 /home/mt/桌面/AI_Animation.html 的问题及修法: 查不到 (资料没有关于硬编码路径的条目).
  - 依据: cs/skill/name-format, cs/skill/naming-gerund, cs/skill/no-time-sensitive-info
- **B** [部分] 答到 E 级 name-dir, W 级 abs-path, 退出码 1, 但没指出目录名 AI-Animation-Skill 含大写本身不合法, 修法写成'改 name 或改目录名其一' (改 name 成目录名会不合规).
  - 答: 会报: [E] name-dir: name='science-content-ppt' 与目录名 'AI-Animation-Skill' 不一致 (规范要求一致); [W] abs-path: 正文写死绝对路径 /home/mt/桌面/AI_Animation.html, 换一台机器就失效; 另有信息项 [I] desc-len (description 字符数及前 120 字符) 和 [I] body-lines (正文行数, 建议 < 500). 有 [E] 时脚本退出码非 0 (示例 exit=1). 修法: 让 name 与目录名一致 (改 name 或改目录名其一); 把写死的本机绝对路径换掉, 不依赖作者本机路径. (绝对路径具体应换成什么写法: 查不到.)
  - 依据: B.md 第147-172行 '拿脚本给两个 skill 体检'
- **OLD** [部分] 答到 name 与目录不一致且目录名含大写不合规, 修法正确, 但 lint 具体报错, 退出码和绝对路径问题都称查不到; 无错误断言.
  - 答: check/skill_lint.py 这个脚本本身及其具体报错内容: 查不到. 按资料列出的校验项推断应报: (1) name(science-content-ppt)与父目录名(AI-Animation-Skill)不一致, 违反'必须与父目录名完全一致'; (2) 目录名 AI-Animation-Skill 含大写字母, 不符合只允许小写字母、数字、连字符的命名规则. 修法: 把目录和 name 统一成同一个合法名字(全小写+连字符, 不超过 64 字符), 例如都改为 science-content-ppt. 正文把默认输出路径写死为 /home/mt/桌面/AI_Animation.html 会被报什么、该怎么修: 查不到.
  - 依据: OLD.md 五、命名 (L183-193), 七、把规则变成代码 检查项表 (L233-244)

### skill-09 (原件未覆盖)
**问**: 同一个 skill 我同时放在 .agents/skills 里给 Codex 用, frontmatter 里的 disable-model-invocation: true 在 Codex 里也能阻止它被自动调用吗?

**标准答案**: 原件未覆盖: Codex 文档 (S6) 没提是否识别 Claude Code 的 disable-model-invocation 字段, 未找到公开资料, 也没有本机实测. 原件只覆盖了 Codex 自己的做法 (agents/openai.yaml 里 policy.allow_implicit_invocation: false). 正确表现是说不确定该字段在 Codex 是否生效, 不给断言.

- **kb** [正确拒答] 明确说查不到 Codex 是否识别该字段, 附带的 openai.yaml 配置和扫描规则属于原件相关事实, 没有越界.
  - 答: 资料里 disable-model-invocation 是 Claude Code 的字段 (规范外字段), 在 Claude Code 中它让 description 不进上下文、只能 /name 调用. Codex 关闭隐式调用的方式是在 agents/openai.yaml 里设 policy.allow_implicit_invocation: false, 之后显式 $skill 仍可用. 所以在 Codex 里应使用 agents/openai.yaml 的这个配置; 资料没有说 Codex 会识别 disable-model-invocation (Codex 是否会忽略它: 查不到明确说法). 要两边通用时 frontmatter 只用 agentskills.io 规范内字段. Codex 从当前目录到仓库根逐级扫 .agents/skills, 各位置不要有同名 skill (同名不合并).
  - 依据: cs/skill/codex-disable-implicit, cs/skill/claude-code-invocation-control, cs/skill/nonspec-fields-hard-error, cs/skill/hosts-share-open-standard, cs/skill/codex-skill-dirs
- **B** [正确拒答] 明确说 Codex 是否识别该字段查不到, 只附带了原件里 openai.yaml 的 allow_implicit_invocation: false.
  - 答: 查不到确定答案: Codex 认不认 disable-model-invocation, 文档没说, 作者也未在 Codex 实测 [不确定]. Codex 关闭隐式调用的方式是在 agents/openai.yaml 里写 policy.allow_implicit_invocation: false. 建议想在两个宿主都只手动调用时两处开关都写.
  - 依据: B.md 第176-184行
- **OLD** [正确拒答] 明确说查不到, 没有给断言; 但称资料没涉及 Codex, 漏了原件里相关的 openai.yaml 做法.
  - 答: 查不到. 资料没有涉及 Codex、.agents/skills 目录, 也没有说明 disable-model-invocation 字段在任何宿主中的行为.
  - 依据: 

### skill-10 (原件未覆盖)
**问**: 我想让 Cursor 也直接用上我放在 .claude/skills 下的 skill, Cursor 会从哪个目录加载? 需要改格式吗?

**标准答案**: 原件未覆盖: 原件只写了 Claude Code 与 Codex 的加载目录, 以及二者遵循 agentskills.io 开放标准, 没有 Cursor 或其他宿主的 skill 加载目录与兼容性信息. 正确表现是说查不到, 不给具体目录或格式断言.

- **kb** [正确拒答] 明确说查不到, 只附带了 Claude Code 和 Codex 的目录.
  - 答: 查不到. 知识库只有 Claude Code (~/.claude/skills、.claude/skills) 与 Codex (.agents/skills 等) 的加载目录, 没有 Cursor 从哪个目录加载 skill、是否需要改格式的资料.
  - 依据: 
- **B** [正确拒答] 明确说资料没有 Cursor 的信息, 只附带了 Claude Code 和 Codex 的目录.
  - 答: 查不到: 资料未涉及 Cursor 的 skill 加载目录或格式要求. 资料只提到 Claude Code 项目 skill 目录为 .claude/skills/, Codex 为从当前目录到仓库根逐级找 .agents/skills/.
  - 依据: B.md 第176-178行
- **OLD** [正确拒答] 明确说查不到, 没有给具体目录或格式断言.
  - 答: 查不到. 资料没有涉及 Cursor 加载 skill 的目录或格式要求. 资料只在参考来源中提到 agentskills.io 有跨平台便携性说明, 但未给出具体内容.
  - 依据: 

