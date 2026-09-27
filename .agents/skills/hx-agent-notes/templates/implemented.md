# Agent Note: <what shipped, as a present-tense claim>

Status: implemented

- **引入于**: `<short-sha>` — <这一条 note 是哪次提交引入的; 首次落地时填, 之后不再改>
- 影响: <受这条决策约束的路径 / 模块, 写得能让人一次找到>

> `<short-sha>` 由 `new-note.ts` 在创建时自动填当前 HEAD; 若这条 note 是与代码
> **同一次提交**落地的, 提交后回头补成那次提交的 sha (见 SKILL.md「首次更新」一节)。

## Problem

<What broke or had to change and why, stated so it stands without the solution.>

## Decision

<What shipped, present tense, naming the mechanism. If the code moves, renames a path,
or changes a default, this file changes in the same commit — facts only.>

<Bespoke sections go here.>

## Alternatives considered

- **<Strongest rival>** — <its best argument>, then why it lost.
- **<Do nothing / reuse what exists>** — <why that was not enough.>

## Consequences

<What this cost and what it bought, both halves. A relative claim ("faster", "smaller")
without a baseline is an unverified assertion — give the baseline or state the fact.>

## Testing

<Optional, present tense: what pins this decision, and which command proves it.>
