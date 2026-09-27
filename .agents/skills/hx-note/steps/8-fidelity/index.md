# 8 fidelity — 有没有讲偏

## 只做这一件事

保真盲审: 检查 `index.md` 有没有偏离它的事实源 `.hx-info.md`。

## 产物

`ai-docs/.hx-staging/<slug>/review/fidelity-audit.md`。

## 实现

协议与 prompt: `../shared/voice/impl/blind-audit.md` 与 `assets/fidelity-audit-prompt.md`。

## 与步骤 7 的关系

**换另一个 sub-agent, 且顺序不可换** (先味道、后保真): 改文风会动句子, 动句子就可能带偏事实。
反过来做等于白做一遍。

## 判定

FAIL 不许交付。「新增」和「歪曲」两节必须清空: 新增的论断要么删、要么先补进 `.hx-info.md`。

**已知边界**: 引入/展望段的个人经历属于**文体要求** (它们来自画像, 不是技术论断),
不计入「新增」。派盲审时要告知审读者这条, 否则它会误报。

## 过关条件

报告判定 PASS; 「新增」与「歪曲」两节为空。
