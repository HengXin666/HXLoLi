# 猫娘立绘归一化报告 (catgirl-normalize)

生成: 2026-11-03 CST, Hark。规格源: `catgirl-spec.md` (26 张成品实测)。

## 0. 源文件事实

- 34 个文件, 实际唯一图像 32 张: `0083e64f(1).png` = `0083e64f.png` (md5 9f0f803f…), `f506e88e-left-half.png` = `f506e88e-9b33….png` (md5 29c7d351…)。**f506e88e-left-half.png 不是手工处理样例, 与源图逐字节相同**, 羽化做法无参照可抄, 按规格自定。
- 34 个全部是 RGB, 无 alpha; 背景混杂: 纯白、纯黑、暗墙纹理、烘焙棋盘格 (假透明)、紫色。
- 大多数文件是 **双格合成图** (一张里左右两格, 两个人物); 单人文件仅 0083e64f×2 / 520f147b / 6414138a / 726f1baa / ChatGPT-0430。因此可用人物数 > 34。
- 绝大多数人物是 "扶墙右缘探头" 构图 (身体贴右 = 天然 left 槽素材)。

## 1. 角色识别 (逐格)

| ID | 源文件 | 格 | 角色 | 作品 | 置信度 | 姿势 | 备注 |
|---|---|---|---|---|---|---|---|
| 0083e64f | `0083e64f-850d-47a3-ada5-219d737a7096.png` | 单人 | Milim Nava 米莉姆 | 关于我转生变成史莱姆这档事 | 0.90 | 黑色战斗服, 双手扶墙右缘探头, 粉双马尾垂左 |  |
| 0083e64f(1) | `0083e64f-850d-47a3-ada5-219d737a7096(1).png` | 单人 | Milim Nava 米莉姆 | 关于我转生变成史莱姆这档事 | 0.90 | 同上 | 与 0083e64f.png md5 相同 (重复文件) |
| 02ebda3c-A | `02ebda3c-6bc8-4e61-9714-69e109e3a5d5.png` | 左格 | 候选: 浅金发琥珀瞳少女 (原创/AI 角色?) | 未识别 | 0.30 | 白水手服+格裙, 扶右缘 |  |
| 02ebda3c-B | `02ebda3c-6bc8-4e61-9714-69e109e3a5d5.png` | 右格 | 候选: 同上角色 (双马尾+粉裙) | 未识别 | 0.30 | 粉吊带裙, 侧马尾, 扶左/右缘 | 疑与 A 同一角色不同服装 |
| 065c1d1e-A | `065c1d1e-8454-4421-88ae-c52182e8b9b6.png` | 左格 | Miyauchi Renge 宫内莲华 | 悠哉日常大王 | 0.90 | 四叶草T恤+蓝短裤, 扶右缘 |  |
| 065c1d1e-B | `065c1d1e-8454-4421-88ae-c52182e8b9b6.png` | 右格 | 候选: Koshigaya Komari 越谷小鞠 / 原创 | 悠哉日常大王 (存疑) | 0.35 | 草帽+粉吊带裙, 扶右缘 |  |
| 08cfa414-A | `08cfa414-0f57-4719-87a7-44ceeed955ec.png` | 左格 | 候选: 浅米色长发少女 | 未识别 | 0.30 | 浅蓝连衣裙, 扶右缘 |  |
| 08cfa414-B | `08cfa414-0f57-4719-87a7-44ceeed955ec.png` | 右格 | 候选: 浅米色短发少女 (粉花发饰) | 未识别 | 0.30 | 粉背带裤, 扶右缘 |  |
| 2bb7378b-A | `2bb7378b-9380-45e3-b3fe-d497338082f9.png` | 左格 | Megumin 惠惠 | 为美好的世界献上祝福! | 0.95 | 魔女帽+红袍, 扶右缘 |  |
| 2bb7378b-B | `2bb7378b-9380-45e3-b3fe-d497338082f9.png` | 右格 | Megumin 惠惠 (无帽便装, 星形发夹) | 为美好的世界献上祝福! | 0.60 | 无帽, 黑披风+粉裙, 扶右缘 | 候选 Yunyun 0.2 |
| 2bb98ec9-A | `2bb98ec9-c0ae-40b9-a442-1d15dd7d9dff.png` | 左格 | Milim Nava 米莉姆 (便装) | 关于我转生变成史莱姆这档事 | 0.75 | 白T恤印'魔国連邦'+牛仔短裤, 扶右缘 | 服饰有文字 noFlip |
| 2bb98ec9-B | `2bb98ec9-c0ae-40b9-a442-1d15dd7d9dff.png` | 右格 | Milim Nava 米莉姆 (泳装) | 关于我转生变成史莱姆这档事 | 0.75 | 白比基尼, 扶右缘 |  |
| 33dacf57-A | `33dacf57-729a-4d13-80f0-c265fd013b17.png` | 左格 | Flandre Scarlet 芙兰朵露 | 东方Project | 0.95 | 红裙+晶翼, 扶右缘 |  |
| 33dacf57-B | `33dacf57-729a-4d13-80f0-c265fd013b17.png` | 右格 | Remilia Scarlet 蕾米莉亚 | 东方Project | 0.95 | 粉裙+蝠翼, 扶右缘 |  |
| 3a95805e-A | `3a95805e-13b0-4a5f-91e1-5e3fb56e4b41.png` | 左格 | Last Order 最后之作 (候选 Misaka Mikoto 0.2) | 某科学的超电磁炮/魔法禁书目录 | 0.60 | 白外套+蓝点裙, 张嘴笑, 扶右缘 |  |
| 3a95805e-B | `3a95805e-13b0-4a5f-91e1-5e3fb56e4b41.png` | 右格 | Last Order 最后之作 | 某科学的超电磁炮/魔法禁书目录 | 0.60 | 同 A, 几乎相同姿势 | 与 A 近似重复, 配对时不得同用 |
| 3c0372d9-A | `3c0372d9-2605-449f-9652-9627dea0168b.png` | 左格 | Izumi Konata 泉此方 | 幸运星 | 0.95 | 水手服, 扶右缘 | 背景为烘焙棋盘格 |
| 3c0372d9-B | `3c0372d9-2605-449f-9652-9627dea0168b.png` | 右格 | Izumi Konata 泉此方 | 幸运星 | 0.95 | 水手服, 扶右缘 | 同上 |
| 3d21065b-A | `3d21065b-ece9-4057-b630-c0e59cffe778.png` | 左格 | Chocola 巧克力 | NEKOPARA | 0.95 | 女仆装, 猫耳, 扶右缘 |  |
| 3d21065b-B | `3d21065b-ece9-4057-b630-c0e59cffe778.png` | 右格 | Vanilla 香草 | NEKOPARA | 0.95 | 女仆装, 猫耳猫尾, 扶右缘 |  |
| 3e045879-A | `3e045879-0532-4054-97d9-74223bdb49eb.png` | 左格 | Oyama Mahiro 绪山真寻 | 别当欧尼酱了! | 0.85 | 白衬衫+红领结, 扶右缘 |  |
| 3e045879-B | `3e045879-0532-4054-97d9-74223bdb49eb.png` | 右格 | Hozuki Momiji 穗月红叶 (候选 Oyama Mihari 0.2) | 别当欧尼酱了! | 0.55 | 校服+发夹, 扶右缘 |  |
| 3ec84c55-A | `3ec84c55-e287-4aab-8b32-d9e3ca25345f.png` | 左格 | Miyauchi Renge 宫内莲华 | 悠哉日常大王 | 0.90 | 背带裤, 扶右缘 |  |
| 3ec84c55-B | `3ec84c55-e287-4aab-8b32-d9e3ca25345f.png` | 右格 | 候选: Koshigaya Komari 越谷小鞠 | 悠哉日常大王 (存疑) | 0.40 | 粉衣, 黄球发饰, 扶右缘 |  |
| 520f147b | `520f147b-ce9b-4da7-9968-05197e4c4b9a.png` | 单人 | 候选: 棕双马尾+耳机少女 | 未识别 | 0.25 | 黄外套, 双手扶杆 | 纯黑背景, 无墙 |
| 54f2845a-A | `54f2845a-9466-4dab-9115-510fae3555b7.png` | 左格 | Itsuka Kotori 五河琴里 | 约会大作战 | 0.75 | 比基尼, 扶右缘 | 背景烘焙棋盘格 |
| 54f2845a-B | `54f2845a-9466-4dab-9115-510fae3555b7.png` | 右格 | Itsuka Kotori 五河琴里 | 约会大作战 | 0.75 | 比基尼, 弯腰站姿 | 无墙边, 需人工定裁切线 |
| 5f946b1f-A | `5f946b1f-6489-47cc-9795-b96fa5b344d6.png` | 左格 | Izumi Konata 泉此方 | 幸运星 | 0.95 | 学校泳衣, 扶右缘 |  |
| 5f946b1f-B | `5f946b1f-6489-47cc-9795-b96fa5b344d6.png` | 右格 | Izumi Konata 泉此方 | 幸运星 | 0.95 | 学校泳衣, 扶右缘 |  |
| 633e3f2b-A | `633e3f2b-b9da-4378-8d02-8c334f2cc4c3.png` | 左格 | Oyama Mahiro 绪山真寻 | 别当欧尼酱了! | 0.85 | 宽松T恤('ニート'字样), 扶左缘 | 服饰有文字 noFlip |
| 633e3f2b-B | `633e3f2b-b9da-4378-8d02-8c334f2cc4c3.png` | 右格 | Hozuki Momiji 穗月红叶 (候选 Mihari) | 别当欧尼酱了! | 0.55 | 白比基尼, 扶右缘 |  |
| 64081cd1-A | `64081cd1-8063-4bd2-8fb8-ec3daace140f.png` | 左格 | 候选: 银发紫瞳哥特少女 | 未识别 | 0.25 | 白衬衫+黑缎带, 扶右缘 | 只出单侧 (用户指定) |
| 64081cd1-B | `64081cd1-8063-4bd2-8fb8-ec3daace140f.png` | 右格 | 候选: 金双马尾紫瞳少女 | 未识别 | 0.25 | 白衬衫+格裙, 扶右缘 | 只出单侧 (用户指定) |
| 6414138a | `6414138a-bc3f-4da1-a4ad-3d0abb630e69.png` | 单人 | 同 520f147b 角色 | 未识别 | 0.25 | 黄外套, 扶杆, 姿势与 520f147b 不同 | 纯黑背景 |
| 6ba3ba8f-A | `6ba3ba8f-cab4-4cf2-a510-8c336dbbf039.png` | 左格 | Izumi Konata 泉此方 | 幸运星 | 0.95 | 比基尼, 扶左缘 (墙在左) | 烘焙棋盘格 |
| 6ba3ba8f-B | `6ba3ba8f-cab4-4cf2-a510-8c336dbbf039.png` | 右格 | Izumi Konata 泉此方 | 幸运星 | 0.95 | 比基尼, 扶右缘 | 烘焙棋盘格 |
| 726f1baa | `726f1baa-a011-4beb-9f24-e90ce9cb70f2.png` | 单人 | Shiro 白 | NO GAME NO LIFE 游戏人生 | 0.90 | 水手服, 双手合拢, 左侧贴边 | 紫色背景 |
| 889198e7-A | `889198e7-be3e-4993-a07e-0bbbf11e767a.png` | 左格 | 同 64081cd1-A 银发角色 | 未识别 | 0.25 | 扶右缘 | 只出单侧 (用户指定) |
| 889198e7-B | `889198e7-be3e-4993-a07e-0bbbf11e767a.png` | 右格 | 同 64081cd1-B 金发角色 | 未识别 | 0.25 | 扶右缘 | 只出单侧 (用户指定) |
| 91267e81-A | `91267e81-2e4b-4f92-a003-8b2232f1e371.png` | 左格 | 同 64081cd1-A 银发角色 | 未识别 | 0.25 | 黑裙, 扶右缘 | 只出单侧 (用户指定) |
| 91267e81-B | `91267e81-2e4b-4f92-a003-8b2232f1e371.png` | 右格 | 候选: 浅金发+头饰 (或同 64081cd1-B) | 未识别 | 0.25 | 黑裙, 扶右缘 | 只出单侧 (用户指定) |
| 933d59f3-A | `933d59f3-4874-4ef2-a55c-d0515689a6cf.png` | 左格 | Oyama Mahiro 绪山真寻 | 别当欧尼酱了! | 0.85 | 猫爪手套+项圈, 扶右缘 |  |
| 933d59f3-B | `933d59f3-4874-4ef2-a55c-d0515689a6cf.png` | 右格 | Hozuki Momiji 穗月红叶 (候选 Mihari) | 别当欧尼酱了! | 0.55 | 红衬衫+黑裙, 扶右缘 |  |
| 95791bb3-A | `95791bb3-e13b-465b-9c26-737d79659711.png` | 左格 | Oyama Mahiro 绪山真寻 | 别当欧尼酱了! | 0.85 | 'ニート'T恤, 扶右杆 | 服饰有文字 noFlip |
| 95791bb3-B | `95791bb3-e13b-465b-9c26-737d79659711.png` | 右格 | Hozuki Momiji 穗月红叶 (候选 Mihari) | 别当欧尼酱了! | 0.55 | 红衬衫+黑裙, 扶右杆 |  |
| 9daaa552-A | `9daaa552-ada6-4b03-9d83-7bb0e8072ce0.png` | 左格 | Milim Nava 米莉姆 (蓝蝴蝶结) | 关于我转生变成史莱姆这档事 | 0.60 | 荷叶边露肩上衣, 扶右缘 |  |
| 9daaa552-B | `9daaa552-ada6-4b03-9d83-7bb0e8072ce0.png` | 右格 | Milim Nava 米莉姆 | 关于我转生变成史莱姆这档事 | 0.60 | 水手上衣+牛仔短裤, 扶右缘 |  |
| 9e6bc154-A | `9e6bc154-52c0-4bd5-81c4-daba5a4d5a19.png` | 左格 | Oyama Mahiro 绪山真寻 | 别当欧尼酱了! | 0.85 | 'ニート'T恤+黄短裤, 扶右缘 | 服饰有文字 noFlip |
| 9e6bc154-B | `9e6bc154-52c0-4bd5-81c4-daba5a4d5a19.png` | 右格 | Hozuki Momiji 穗月红叶 (候选 Mihari) | 别当欧尼酱了! | 0.55 | 红衬衫+黑裙, 扶右缘 |  |
| ChatGPT-0430 | `ChatGPT Image 2026年4月30日 21_59_01.png` | 单人 | 候选: Ujimatsu Chiya 宇治松千夜 / 原创 | 请问您今天要来点兔子吗? (存疑) | 0.25 | 白贝雷帽+水手服, 扶右缘 | 798x1971 已接近目标比例 |
| bffd4e18-A | `bffd4e18-ccb1-47f2-86ec-330d8f5c79b8.png` | 左格 | Eris Boreas Greyrat 艾莉丝 | 无职转生 | 0.80 | 露肩衣, 扶右缘 | 上半身极近景 |
| bffd4e18-B | `bffd4e18-ccb1-47f2-86ec-330d8f5c79b8.png` | 右格 | Eris Boreas Greyrat 艾莉丝 | 无职转生 | 0.80 | 同 A 近似姿势 | 与 A 近似重复 |
| cc05132b-A | `cc05132b-227d-4bb7-a18a-a6a114ee15af.png` | 左格 | Illyasviel von Einzbern 伊莉雅 | 魔法少女伊莉雅 | 0.90 | 魔法少女装, 扶右缘 |  |
| cc05132b-B | `cc05132b-227d-4bb7-a18a-a6a114ee15af.png` | 右格 | Miyu Edelfelt 美游 | 魔法少女伊莉雅 | 0.75 | 紫色魔法少女装, 扶右缘 |  |
| d946bb2e-A | `d946bb2e-4700-4d62-bd82-630bedea3668.png` | 左格 | Megumin 惠惠 | 为美好的世界献上祝福! | 0.95 | 魔女帽+红衣, 扶右缘 |  |
| d946bb2e-B | `d946bb2e-4700-4d62-bd82-630bedea3668.png` | 右格 | Megumin 惠惠 (无帽, 星形发夹) | 为美好的世界献上祝福! | 0.60 | 披风+黑裤, 扶右缘 |  |
| f506e88e-A | `f506e88e-9b33-4373-bc00-7dc99bc0e076.png` | 左格 | 候选: 薰衣草发+白帽少女 | 未识别 | 0.25 | 白连衣裙+白帽, 扶左缘 | 只出单侧 (用户指定) |
| f506e88e-B | `f506e88e-9b33-4373-bc00-7dc99bc0e076.png` | 右格 | 候选: 深蓝马尾少女 | 未识别 | 0.25 | 蓝荷叶边上衣, 扶右缘 | 只出单侧 (用户指定) |
| f506e88e-lh | `f506e88e-left-half.png` | (同 f506e88e) | 同 f506e88e | 同 f506e88e | 0.25 | 同 f506e88e | 与 f506e88e-9b33…png md5 相同, 不是手工样例 |
| f82dd5b6-A | `f82dd5b6-309d-46eb-93ae-25ae81deca39.png` | 左格 | Izumi Konata 泉此方 | 幸运星 | 0.95 | 水手服, 扶右缘 |  |
| f82dd5b6-B | `f82dd5b6-309d-46eb-93ae-25ae81deca39.png` | 右格 | Izumi Konata 泉此方 | 幸运星 | 0.95 | 水手服, 扶右缘 |  |
| f9d48c7f-A | `f9d48c7f-a48e-4e5e-8447-20b7171be065.png` | 左格 | Last Order 最后之作 | 某科学的超电磁炮/魔法禁书目录 | 0.60 | 实验袍+蓝点裙, 扶左缘 |  |
| f9d48c7f-B | `f9d48c7f-a48e-4e5e-8447-20b7171be065.png` | 右格 | Last Order 最后之作 | 某科学的超电磁炮/魔法禁书目录 | 0.60 | 挥手, 扶右缘 |  |

## 1.1 角色 → 姿势索引

同一角色不同姿势归在同一条下; 近似重复姿势 (同构图仅细节差异) 已在识别表标注。

- **Izumi Konata 泉此方** — 幸运星 — 8 个姿势: 3c0372d9-A (水手服, 扶右缘); 3c0372d9-B (水手服, 扶右缘); 5f946b1f-A (学校泳衣, 扶右缘); 5f946b1f-B (学校泳衣, 扶右缘); 6ba3ba8f-A (比基尼, 扶左缘 (墙在左)); 6ba3ba8f-B (比基尼, 扶右缘); f82dd5b6-A (水手服, 扶右缘); f82dd5b6-B (水手服, 扶右缘)
- **Milim Nava 米莉姆** — 关于我转生变成史莱姆这档事 — 5 个姿势: 0083e64f (黑色战斗服, 双手扶墙右缘探头, 粉双马尾垂左); 2bb98ec9-A (白T恤印'魔国連邦'+牛仔短裤, 扶右缘); 2bb98ec9-B (白比基尼, 扶右缘); 9daaa552-A (荷叶边露肩上衣, 扶右缘); 9daaa552-B (水手上衣+牛仔短裤, 扶右缘)
- **Oyama Mahiro 绪山真寻** — 别当欧尼酱了! — 5 个姿势: 3e045879-A (白衬衫+红领结, 扶右缘); 633e3f2b-A (宽松T恤('ニート'字样), 扶左缘); 933d59f3-A (猫爪手套+项圈, 扶右缘); 95791bb3-A ('ニート'T恤, 扶右杆); 9e6bc154-A ('ニート'T恤+黄短裤, 扶右缘)
- **Hozuki Momiji 穗月红叶(存疑)** — 别当欧尼酱了! — 5 个姿势: 3e045879-B (校服+发夹, 扶右缘); 633e3f2b-B (白比基尼, 扶右缘); 933d59f3-B (红衬衫+黑裙, 扶右缘); 95791bb3-B (红衬衫+黑裙, 扶右杆); 9e6bc154-B (红衬衫+黑裙, 扶右缘)
- **Megumin 惠惠** — 为美好的世界献上祝福! — 4 个姿势: 2bb7378b-A (魔女帽+红袍, 扶右缘); 2bb7378b-B (无帽, 黑披风+粉裙, 扶右缘); d946bb2e-A (魔女帽+红衣, 扶右缘); d946bb2e-B (披风+黑裤, 扶右缘)
- **Last Order 最后之作** — 某科学的超电磁炮/魔法禁书目录 — 4 个姿势: 3a95805e-A (白外套+蓝点裙, 张嘴笑, 扶右缘); 3a95805e-B (同 A, 几乎相同姿势); f9d48c7f-A (实验袍+蓝点裙, 扶左缘); f9d48c7f-B (挥手, 扶右缘)
- **未识别-银发紫瞳** — 未识别 — 3 个姿势: 64081cd1-A (白衬衫+黑缎带, 扶右缘); 889198e7-A (扶右缘); 91267e81-A (黑裙, 扶右缘)
- **未识别-金双马尾** — 未识别 — 3 个姿势: 64081cd1-B (白衬衫+格裙, 扶右缘); 889198e7-B (扶右缘); 91267e81-B (黑裙, 扶右缘)
- **未识别-浅金发琥珀瞳** — 未识别 — 2 个姿势: 02ebda3c-A (白水手服+格裙, 扶右缘); 02ebda3c-B (粉吊带裙, 侧马尾, 扶左/右缘)
- **Miyauchi Renge 宫内莲华** — 悠哉日常大王 — 2 个姿势: 065c1d1e-A (四叶草T恤+蓝短裤, 扶右缘); 3ec84c55-A (背带裤, 扶右缘)
- **Itsuka Kotori 五河琴里** — 约会大作战 — 2 个姿势: 54f2845a-A (比基尼, 扶右缘); 54f2845a-B (比基尼, 弯腰站姿)
- **Eris 艾莉丝** — 无职转生 — 2 个姿势: bffd4e18-A (露肩衣, 扶右缘); bffd4e18-B (同 A 近似姿势)
- **候选: Koshigaya Komari 越谷小鞠 / 原创** — 悠哉日常大王 (存疑) — 1 个姿势: 065c1d1e-B (草帽+粉吊带裙, 扶右缘)
- **候选: 浅米色长发少女** — 未识别 — 1 个姿势: 08cfa414-A (浅蓝连衣裙, 扶右缘)
- **候选: 浅米色短发少女 (粉花发饰)** — 未识别 — 1 个姿势: 08cfa414-B (粉背带裤, 扶右缘)
- **Flandre Scarlet 芙兰朵露** — 东方Project — 1 个姿势: 33dacf57-A (红裙+晶翼, 扶右缘)
- **Remilia Scarlet 蕾米莉亚** — 东方Project — 1 个姿势: 33dacf57-B (粉裙+蝠翼, 扶右缘)
- **Chocola 巧克力** — NEKOPARA — 1 个姿势: 3d21065b-A (女仆装, 猫耳, 扶右缘)
- **Vanilla 香草** — NEKOPARA — 1 个姿势: 3d21065b-B (女仆装, 猫耳猫尾, 扶右缘)
- **候选: Koshigaya Komari 越谷小鞠** — 悠哉日常大王 (存疑) — 1 个姿势: 3ec84c55-B (粉衣, 黄球发饰, 扶右缘)
- **候选: 棕双马尾+耳机少女** — 未识别 — 1 个姿势: 520f147b (黄外套, 双手扶杆)
- **未识别-棕双马尾耳机** — 未识别 — 1 个姿势: 6414138a (黄外套, 扶杆, 姿势与 520f147b 不同)
- **Shiro 白** — NO GAME NO LIFE 游戏人生 — 1 个姿势: 726f1baa (水手服, 双手合拢, 左侧贴边)
- **候选: Ujimatsu Chiya 宇治松千夜 / 原创** — 请问您今天要来点兔子吗? (存疑) — 1 个姿势: ChatGPT-0430 (白贝雷帽+水手服, 扶右缘)
- **Illyasviel von Einzbern 伊莉雅** — 魔法少女伊莉雅 — 1 个姿势: cc05132b-A (魔法少女装, 扶右缘)
- **Miyu Edelfelt 美游** — 魔法少女伊莉雅 — 1 个姿势: cc05132b-B (紫色魔法少女装, 扶右缘)
- **候选: 薰衣草发+白帽少女** — 未识别 — 1 个姿势: f506e88e-A (白连衣裙+白帽, 扶左缘)
- **候选: 深蓝马尾少女** — 未识别 — 1 个姿势: f506e88e-B (蓝荷叶边上衣, 扶右缘)

**不同角色数: 28** (其中 置信度<0.5 的未识别/存疑角色 13 个, 需人工确认)。

## 2. 归一化结果统计 (2026-11-03 22:55 CST)

- 成品: 47 张 / 25 条 (成套 22 套 + 单侧 3 条), 全部 760x1824 (vanilla-01 原生 608x1458, 源格太小未放大), 均在 manifest.json.
- check: pass 19 条; fail 6 条, 失败项全是几何 (OCC/CX 越出 spec 区间, 人物构图偏窄), 不是残带. 镜像出的 right 图 CX = 1 - left CX, 而 spec 的 right 下限 0.21 比 left 上限 0.80 的镜像严 0.01, 所以 left CX 0.80 附近的图镜像后必 fail.
- 内侧残带: 全部 0 (用户口径: 45%-95% 高度带, 参考列距内侧 16px, <72% 计偏暗; 剩下 1-2 列均判为人物描边, 保留).
- 镜像补侧: chocola-01 / vanilla-01 / remilia-01 (由旧 *-right.png 转正) / flandre-01 / megumin-01 / momiji-01.
- needsGen (缺一侧, 不可镜像或按要求只出单侧):
  - mahiro-02: 缺 left — noFlip: 服饰有文字不可镜像
  - milim-02: 缺 right — noFlip: 服饰有文字不可镜像
  - unk-blonde-twintail-01: 缺 right — 用户指定只出单侧
- 弃用 (不进 manifest, fs 隧道无 DELETE, 需手动 rm): chocola-right.png, flandre-right.png, konata-right.png, megumin-right.png, milim-right.png, remilia-right.png, renge-right.png, vanilla-right.png

各角色姿势数 (成品):

| 角色 | 条数 | slug |
|---|---|---|
| Chocola 巧克力 | 1 | chocola-01 |
| Flandre Scarlet 芙兰朵露 | 1 | flandre-01 |
| 候选: Koshigaya Komari 越谷小鞠 | 1 | komari-01 |
| Izumi Konata 泉此方 | 2 | konata-04, konata-06 |
| Itsuka Kotori 五河琴里 | 1 | kotori-02 |
| Last Order 最后之作 | 2 | last-order-01, last-order-02 |
| Oyama Mahiro 绪山真寻 | 1 | mahiro-02 |
| Megumin 惠惠 | 1 | megumin-01 |
| Milim Nava 米莉姆 | 3 | milim-01, milim-02, milim-03 |
| Hozuki Momiji 穗月红叶 (候选 Oyama Mihari) | 1 | momiji-01 |
| Remilia Scarlet 蕾米莉亚 | 1 | remilia-01 |
| Miyauchi Renge 宫内莲华 | 2 | renge-01, renge-02 |
| 候选: 浅金发琥珀瞳少女 (未识别) | 1 | unk-amber-blonde-02 |
| 候选: 浅米色长发少女 (未识别) | 1 | unk-beige-long-01 |
| 候选: 浅米色短发少女 (未识别) | 1 | unk-beige-short-01 |
| 候选: 金双马尾紫瞳少女 (未识别; 源按外观匹配) | 1 | unk-blonde-twintail-01 |
| 候选: 棕双马尾+耳机少女 (未识别) | 2 | unk-brown-twintail-01, unk-brown-twintail-02 |
| 候选: Koshigaya Komari 越谷小鞠 / 原创 | 1 | unk-strawhat-01 |
| Vanilla 香草 | 1 | vanilla-01 |

未出 (源图有、上一轮批处理尚未产出或被其丢弃): konata 其余 6 姿势, mahiro 01/03/04/05, momiji 02-05, megumin 02-04, milim 04-05, last-order 03-04, kotori-01, eris, illya, miyu, shiro, 64081cd1/91267e81/f506e88e 单侧格, ChatGPT-0430, unk-amber-blonde-01.

## 3. 破绽自查表 (内侧深色带)

判据 (scripts/catgirl_finalize.py strip_inner_band): 从内侧边向内扫, 列亮度 < 参考 75% 即计入, 上限 80 列; N>=8, 或 N>=5 且相邻列亮度差全 <6 (恒定背板) -> 剥; 5-7 列且近中性 (HSV 饱和度 <0.15) -> 墙沿残带, 剥; 其余 1-7 列只有色相差 >35 度或近灰 (饱和度 < 相邻 40%) 才剥, 否则判描边保留. 剥后人物向内平移 N px, 画布不变, AR/TOP/贴底不变, gap 0.

| 文件 | 修前暗列 | 剥掉列 | 修前结论 | 修后复测(用户口径) | 修后结论 | spec | W x H | fade |
|---|---|---|---|---|---|---|---|---|
| chocola-01-left.png | 0 | 0 | 干净 | 0 | clean | pass | 928x2227 | 0.88 |
| chocola-01-right.png | 0 | 0 | 干净 | 0 | clean | pass | 928x2227 | 0.88 |
| flandre-01-left.png | 0 | 0 | 干净 | 1 | clean | pass | 760x1824 | 0.88 |
| flandre-01-right.png | 0 | 0 | 干净 | 1 | clean | fail CX | 760x1824 | 0.88 |
| komari-01-left.png | 2 | 0 | 描边, 非破绽, 保留 | 1 | outline-kept | pass | 760x1824 | 0.93 |
| komari-01-right.png | 2 | 0 | 描边, 非破绽, 保留 | 1 | outline-kept | pass | 760x1824 | 0.93 |
| konata-04-left.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.86 |
| konata-04-right.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.86 |
| konata-06-left.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.91 |
| konata-06-right.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.91 |
| kotori-02-left.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.92 |
| kotori-02-right.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.92 |
| last-order-01-left.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.87 |
| last-order-01-right.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.87 |
| last-order-02-left.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.92 |
| last-order-02-right.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.92 |
| mahiro-02-right.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.88 |
| megumin-01-left.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.87 |
| megumin-01-right.png | 0 | 0 | 干净 | 0 | clean | fail OCC,CX | 760x1824 | 0.87 |
| milim-01-left.png | 1 | 0 | 描边, 非破绽, 保留 | 1 | outline-kept | pass | 760x1824 | 0.93 |
| milim-01-right.png | 1 | 0 | 描边, 非破绽, 保留 | 1 | outline-kept | pass | 760x1824 | 0.93 |
| milim-02-left.png | 1 | 0 | 描边, 非破绽, 保留 | 1 | outline-kept | pass | 760x1824 | 0.91 |
| milim-03-left.png | 1 | 0 | 描边, 非破绽, 保留 | 1 | outline-kept | pass | 760x1824 | 0.93 |
| milim-03-right.png | 1 | 0 | 描边, 非破绽, 保留 | 1 | outline-kept | pass | 760x1824 | 0.93 |
| momiji-01-left.png | 1 | 0 | 描边, 非破绽, 保留 | 1 | outline-kept | pass | 760x1824 | 0.88 |
| momiji-01-right.png | 1 | 0 | 描边, 非破绽, 保留 | 1 | outline-kept | fail OCC,CX | 760x1824 | 0.88 |
| remilia-01-left.png | 0 | 0 | 干净 | 0 | clean | pass | 806x1935 | 0.88 |
| remilia-01-right.png | 0 | 0 | 干净 | 0 | clean | pass | 806x1935 | 0.88 |
| renge-01-left.png | 2 | 0 | 描边, 非破绽, 保留 | 2 | outline-kept | pass | 760x1824 | 0.92 |
| renge-01-right.png | 2 | 0 | 描边, 非破绽, 保留 | 2 | outline-kept | pass | 760x1824 | 0.92 |
| renge-02-left.png | 6 | 6 | 墙沿残带(近灰), 已剥 | 0 | clean | pass | 760x1824 | 0.89 |
| renge-02-right.png | 6 | 6 | 墙沿残带(近灰), 已剥 | 0 | clean | pass | 760x1824 | 0.89 |
| unk-amber-blonde-02-left.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.9 |
| unk-amber-blonde-02-right.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.9 |
| unk-beige-long-01-left.png | 15 | 15 | 深色背板/墙, 已剥 | 0 | clean | fail OCC,CX | 760x1824 | 0.91 |
| unk-beige-long-01-right.png | 15 | 15 | 深色背板/墙, 已剥 | 0 | clean | fail OCC,CX | 760x1824 | 0.91 |
| unk-beige-short-01-left.png | 15 | 15 | 深色背板/墙, 已剥 | 0 | clean | pass | 760x1824 | 0.88 |
| unk-beige-short-01-right.png | 15 | 15 | 深色背板/墙, 已剥 | 0 | clean | fail CX | 760x1824 | 0.88 |
| unk-blonde-twintail-01-left.png | 7 | 7 | 墙沿残带(近灰), 已剥 | 0 | clean | fail OCC | 760x1824 | 0.93 |
| unk-brown-twintail-01-left.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.93 |
| unk-brown-twintail-01-right.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.93 |
| unk-brown-twintail-02-left.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.92 |
| unk-brown-twintail-02-right.png | 0 | 0 | 干净 | 0 | clean | pass | 760x1824 | 0.92 |
| unk-strawhat-01-left.png | 1 | 0 | 描边, 非破绽, 保留 | 1 | outline-kept | pass | 760x1824 | 0.91 |
| unk-strawhat-01-right.png | 1 | 0 | 描边, 非破绽, 保留 | 1 | outline-kept | pass | 760x1824 | 0.91 |
| vanilla-01-left.png | 0 | 0 | 干净 | 0 | clean | pass | 608x1458 | 0.87 |
| vanilla-01-right.png | 0 | 0 | 干净 | 0 | clean | pass | 608x1458 | 0.87 |

## 4. 最终清单 (2026-11-04 CST, 以 `static/img/catgirl/manifest.json` 为准)

manifest: 25 条 entries (22 套左右对 + 3 条单侧, 单侧按 needsGen 收, 运行时只进有图那侧的槽, 不镜像) + 9 个 excluded 孤儿文件. 文件数: 22x2 + 3 + 9 = **56**, 即 `static/img/catgirl/` 下除 manifest.json 与内置 pair1..13 外的全部文件.

"spec 构图" 是 manifest 里 check 字段 (catgirl-spec 的 OCC / CX 构图区间), 与 §5 的四项机械校验是两回事: 6 条 fail 全是 OCC / CX 越出区间 (人物构图偏窄, 多为镜像侧 CX 差约 0.01), 不是残带, parseImg 不检查这两项, 照常上墙.

### 4.1 entries (25 条)

| slug | 角色 | 作品 | 姿势 | left | right | 状态 |
|---|---|---|---|---|---|---|
| chocola-01 | Chocola 巧克力 | NEKOPARA | 女仆装, 猫耳, 扶右缘 | `chocola-01-left.png` | `chocola-01-right.png` | 成套; spec 构图: pass |
| flandre-01 | Flandre Scarlet 芙兰朵露 | 东方Project | 红裙+晶翼, 扶右缘 | `flandre-01-left.png` | `flandre-01-right.png` | 成套; spec 构图: fail: right CX=0.199 |
| komari-01 | 候选: Koshigaya Komari 越谷小鞠 | 悠哉日常大王 (存疑) | 粉衣, 黄球发饰, 扶右缘 | `komari-01-left.png` | `komari-01-right.png` | 成套; spec 构图: pass |
| konata-04 | Izumi Konata 泉此方 | 幸运星 | 学校泳衣, 扶右缘 | `konata-04-left.png` | `konata-04-right.png` | 成套; spec 构图: pass |
| konata-06 | Izumi Konata 泉此方 | 幸运星 | 比基尼, 扶右缘 | `konata-06-left.png` | `konata-06-right.png` | 成套; spec 构图: pass |
| kotori-02 | Itsuka Kotori 五河琴里 | 约会大作战 | 比基尼, 弯腰站姿 (无墙边, 人工定裁切线) | `kotori-02-left.png` | `kotori-02-right.png` | 成套; spec 构图: pass |
| last-order-01 | Last Order 最后之作 | 某科学的超电磁炮/魔法禁书目录 | 白外套+蓝点裙, 张嘴笑, 扶右缘 | `last-order-01-left.png` | `last-order-01-right.png` | 成套; spec 构图: pass |
| last-order-02 | Last Order 最后之作 | 某科学的超电磁炮/魔法禁书目录 | 同 01 近似姿势 (配对时不得与 01 同用) | `last-order-02-left.png` | `last-order-02-right.png` | 成套; spec 构图: pass |
| mahiro-02 | Oyama Mahiro 绪山真寻 | 别当欧尼酱了! | 宽松T恤('ニート'字样), 扶左缘 | `— (缺)` | `mahiro-02-right.png` | 单侧, 缺 left (noFlip: 服饰有文字不可镜像); spec 构图: pass |
| megumin-01 | Megumin 惠惠 | 为美好的世界献上祝福! | 魔女帽+红袍, 扶右缘 | `megumin-01-left.png` | `megumin-01-right.png` | 成套; spec 构图: fail: right OCC=0.529,CX=0.2 |
| milim-01 | Milim Nava 米莉姆 | 关于我转生变成史莱姆这档事 | 黑色战斗服, 双手扶墙右缘探头 | `milim-01-left.png` | `milim-01-right.png` | 成套; spec 构图: pass |
| milim-02 | Milim Nava 米莉姆 | 关于我转生变成史莱姆这档事 | 白T恤印'魔国連邦'+牛仔短裤, 扶右缘 | `milim-02-left.png` | `— (缺)` | 单侧, 缺 right (noFlip: 服饰有文字不可镜像); spec 构图: pass |
| milim-03 | Milim Nava 米莉姆 | 关于我转生变成史莱姆这档事 | 白比基尼, 扶右缘 | `milim-03-left.png` | `milim-03-right.png` | 成套; spec 构图: pass |
| momiji-01 | Hozuki Momiji 穗月红叶 (候选 Oyama Mihari) | 别当欧尼酱了! | 校服+发夹, 扶右缘 | `momiji-01-left.png` | `momiji-01-right.png` | 成套; spec 构图: fail: right OCC=0.524,CX=0.199 |
| remilia-01 | Remilia Scarlet 蕾米莉亚 | 东方Project | 粉裙+蝠翼, 扶右缘 | `remilia-01-left.png` | `remilia-01-right.png` | 成套; spec 构图: pass |
| renge-01 | Miyauchi Renge 宫内莲华 | 悠哉日常大王 | 四叶草T恤+蓝短裤, 扶右缘 | `renge-01-left.png` | `renge-01-right.png` | 成套; spec 构图: pass |
| renge-02 | Miyauchi Renge 宫内莲华 | 悠哉日常大王 | 背带裤, 扶右缘 | `renge-02-left.png` | `renge-02-right.png` | 成套; spec 构图: pass |
| unk-amber-blonde-02 | 候选: 浅金发琥珀瞳少女 (未识别) | 未识别 | 粉吊带裙, 侧马尾, 扶墙 | `unk-amber-blonde-02-left.png` | `unk-amber-blonde-02-right.png` | 成套; spec 构图: pass |
| unk-beige-long-01 | 候选: 浅米色长发少女 (未识别) | 未识别 | 浅蓝连衣裙, 扶右缘 | `unk-beige-long-01-left.png` | `unk-beige-long-01-right.png` | 成套; spec 构图: fail: left OCC=0.507,CX=0.819; right OCC=0.507,CX=0.18 |
| unk-beige-short-01 | 候选: 浅米色短发少女 (未识别) | 未识别 | 粉背带裤, 扶右缘 | `unk-beige-short-01-left.png` | `unk-beige-short-01-right.png` | 成套; spec 构图: fail: right CX=0.2 |
| unk-blonde-twintail-01 | 候选: 金双马尾紫瞳少女 (未识别; 源按外观匹配) | 未识别 | 白衬衫+黑缎带, 扶右缘 | `unk-blonde-twintail-01-left.png` | `— (缺)` | 单侧, 缺 right (用户指定只出单侧); spec 构图: fail: left OCC=0.505 |
| unk-brown-twintail-01 | 候选: 棕双马尾+耳机少女 (未识别) | 未识别 | 黄外套, 双手扶杆 | `unk-brown-twintail-01-left.png` | `unk-brown-twintail-01-right.png` | 成套; spec 构图: pass |
| unk-brown-twintail-02 | 候选: 棕双马尾+耳机少女 (未识别) | 未识别 | 黄外套, 扶杆 (与 01 姿势不同) | `unk-brown-twintail-02-left.png` | `unk-brown-twintail-02-right.png` | 成套; spec 构图: pass |
| unk-strawhat-01 | 候选: Koshigaya Komari 越谷小鞠 / 原创 | 悠哉日常大王 (存疑) | 草帽+粉吊带裙, 扶右缘 | `unk-strawhat-01-left.png` | `unk-strawhat-01-right.png` | 成套; spec 构图: pass |
| vanilla-01 | Vanilla 香草 | NEKOPARA | 女仆装, 猫耳猫尾, 扶右缘 | `vanilla-01-left.png` | `vanilla-01-right.png` | 成套; spec 构图: pass |

### 4.2 excluded (9 个孤儿, 不进 entries)

| slug | 角色 | 作品 | 姿势 | left | right | 状态 |
|---|---|---|---|---|---|---|
| chocola-right | Chocola 巧克力 | NEKOPARA | — | — | `chocola-right.png` | excluded: 旧命名残留, 已由 chocola-01-left/-right 覆盖 |
| flandre-right | Flandre Scarlet 芙兰朵露 | 东方Project | — | — | `flandre-right.png` | excluded: 旧命名残留, 已由 flandre-01-left/-right 覆盖 |
| konata-right | Izumi Konata 泉此方 | 幸运星 | — | — | `konata-right.png` | excluded: 旧命名残留, 已由 konata-04/06 覆盖 |
| megumin-right | Megumin 惠惠 | 为美好的世界献上祝福! | — | — | `megumin-right.png` | excluded: 旧命名残留, 已由 megumin-01-left/-right 覆盖 |
| milim-right | Milim Nava 米莉姆 | 关于我转生变成史莱姆这档事 | — | — | `milim-right.png` | excluded: 旧命名残留, 已由 milim-01/02/03 覆盖 |
| remilia-right | Remilia Scarlet 蕾米莉亚 | 东方Project | — | — | `remilia-right.png` | excluded: 旧命名残留, 已由 remilia-01-left/-right 覆盖 |
| renge-right | Miyauchi Renge 宫内莲华 | 悠哉日常大王 | — | — | `renge-right.png` | excluded: 旧命名残留, 已由 renge-01/02 覆盖 |
| shiro-01-right | Shiro 白 | NO GAME NO LIFE 游戏人生 | — | — | `shiro-01-right.png` | excluded: 素材只有单侧, 无配对侧 |
| vanilla-right | Vanilla 香草 | NEKOPARA | — | — | `vanilla-right.png` | excluded: 素材只有单侧, 无配对侧 |

其中 8 个旧命名 `*-right.png` 同时列在 manifest `deprecated` 里, 待手动 rm (fs 隧道无 DELETE); `shiro-01-right.png` 只有单侧且无配对侧, 暂不上墙.

## 5. 校验结果 (2026-11-04 CST, [用户本机])

56 张 (entries 47 张 + excluded 9 张) 全部通过四项机械检查:

| 检查项 | 判据 | 结果 |
|---|---|---|
| 宽高比 | AR = 5:12 | 56/56 通过 |
| 顶部透明带 | top = 0.450 (上 45% 透明, 人物贴底占下 55%) | 56/56 通过 |
| 内侧贴边 | 裁切线所在内侧 gap = 0 | 56/56 通过 |
| 深色残带 | 内侧无 >= 5 列连续深色残带 | 56/56 通过 |

- 曾经截断的 9 张已重传, 重新读取验证可读, 四项检查同样通过.
- 配套: characterPrompt.check 22 项全通; manifest (entries + excluded) 覆盖 56 个文件.

