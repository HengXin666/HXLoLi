# 7 review — 说不说人话

## 只做这一件事

AI 味评审。两件事并行:

- 把 `index.md` 交给人类看。
- 派 sub-agent 做 **AI 味盲审**。

## 产物

`ai-docs/.hx-staging/<slug>/review/voice-audit.md`。

## 实现

盲审协议 (污染控制三铁律、prompt 模板、异源与顺序对调两条硬约束、什么时候可以跳过):
`../shared/voice/impl/blind-audit.md`。
prompt 逐字取自 `assets/voice-audit-prompt.md`。

## 顺序

**先跑 `hx_voice.py lint` 再派盲审** —— lint 能抓的东西让盲审去抓是浪费一次调用, 而且满屏
低级命中会淹没盲审真正有价值的发现。

## 冲突处理

人类意见与盲审意见冲突时, **听人类的**; 但要把分歧点写进 `.hx-mitemite.md`。

## 过关条件

盲审结论已逐条落实, 或写明不改的理由; 新发现的表达已 `learn` 进语料库。
