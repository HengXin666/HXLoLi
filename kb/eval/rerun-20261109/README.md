# 补卡后重答 (2026-11-09 01:13)

补卡提交 8bdebc19f0 后, 卡库 (155 张) 重答原失分 3 题. 答题 agent 只见 kb/cards + concepts.yaml + index.json; 判分只见 gold + keypoints, 口径同主评测.

| 题 | 补卡前 | 补卡后 |
|---|---|---|
| bag-02 | 部分 | 正确 |
| bag-08 | 部分 | 部分 (只写"升数", 没写 20L / 30L; 判分者自注偏严) |
| skill-08 | 部分 | 正确 |

卡库合计: 27/30 -> 29/30. B 16/30, OLD 6/30 未重跑. 单次运行, 方向性结论.
文件: questions.json (3 题), answers-kb.json, grade-blind.json, grade-result.json
