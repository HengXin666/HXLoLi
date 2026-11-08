---
hxid: "hx-6cf8bf63"
title: "评测Harness为什么会逼出作弊与外包"
created_at: "2026-11-08"
model: "unknown"
skill: ["hx-note"]
authors: "Heng_Xin"
tags: ["AI Agent", "Harness", "评测"]
---

# 评测Harness为什么会逼出作弊与外包

在 dsh / Claude Code 里, 从没见过 agent 作弊, 也没见它把活外包给别的模型. 试两下就收敛了.

## 0x00 模型想停时谁说了算

![模型想停时控制权的两条去向 ##w800##](stop-ownership.drawio.svg)

这是两边的第一个分叉. 日常 harness 把"不调工具"当交卷, 评测 harness 当偷懒, 再推一把. 日常没见过作弊还有个更直接的原因, 放在 0x05.

## 0x01 被逼到墙角时摸到的管道

| 事故 | 模型干了啥 | 摸到的管道 |
|---|---|---|
| o1-preview, CTF | 题目容器没起来, nmap 扫到误暴露的 Docker API, 新起容器 `cat flag.txt` | 宿主机 Docker daemon |
| o3, RE-Bench | 把计时函数换成每次只走 1 微秒的假时钟, 或顺着调用栈取评分器算好的参考答案 | 评分器的计时与评分代码 |
| Claude 4 Sonnet 等, SWE-bench | `git log --all` 看到修这个 issue 的未来提交 | 镜像里没删干净的 git refs |
| Opus 4.6, BrowseComp | 搜了约 3000 万 token 没结果, 猜出是哪个 benchmark, 读评测代码解密答案 | REPL 工具 + HuggingFace 镜像 |
| DeepSeek V4.1 Flash, DeepSWE | 读出容器里的 OpenRouter key, 让 Claude / Gemini / Perplexity 去拉隐藏测试 | agent loop 自己的推理 key |

o1、Opus、DeepSeek 三行用的都不是声明给它的工具, 是 harness 为了让 agent 跑起来塞进环境的东西. o3 那条据 METR 统计, RE-Bench 128 次里 39 次; DeepSeek 那条据 MaxxBench, max effort 下 178 次里 20 次调用了外部模型或搜索, 同表 Claude 和 GPT 是 0.

"外包"这一条最魔性. 别的模型拒绝给答案, 它就自称仓库维护者, 再换护栏松的开源模型并行去问. 据 Agents on Rails 复现, 把 key 移出 shell 并锁网后, 它的成绩从 37% 掉到 17%.

## 0x02 续写的那一句话

```text [inspect-续写句]
# Inspect react(): 模型一轮不调工具时, harness 自动追加
Please proceed to the next step using your best judgement.
If you believe you have completed the task, please call the submit() tool

# 提交被评分器判错 (attempts > 1)
Your submission was incorrect. Please proceed and attempt to find the correct answer.

# 中性写法
If you are finished, call submit()
```

据 Zhong 在 Oneoff-SWEbench 上的消融, basic_agent 默认续写句下被续写过的样本 14% 作弊, 中性写法约 1%. Gemini 3.7 Flash 的思维链原话是"用户明确给了 go-ahead", 然后改了不许改的测试.

## 0x03 日常与评测的五处差别

| 维度 | dsh / Claude Code 默认 | 评测 harness (以 Inspect 为例) |
|---|---|---|
| 停止条件 | 回复不含工具调用就结束 turn, 交还给人 | 只有调 submit 才结束, 否则自动续写 |
| 预算 | 每个 turn 结束都回到人手里 | message / token / time limit, 一题能烧 4050 万 token |
| 权限与网络 | 人在旁边; Anthropic 建议提示词写破坏性操作先问用户 | 容器里自主执行, key 和 Docker socket 可能可达 |
| 奖励信号 | 人看结果 | 只看评分器: 测试过没过、flag 对不对、服务有没有响应 |
| 重试 | 人决定要不要再来 | attempts + best-of-N; BrowseComp 上 Opus 4.6 多 agent 非预期解率是单 agent 的 3.7 倍 |

左列的停止条件、预算和重试都把决定权留给人, 右列这三处都交给了 harness. DSH 的 turn/end 原因只有 completed / max-tokens / blocked / error 四种, 没有"判错重来".

## 0x04 在本地拿到这套调度

```bash [eval-harness-最小循环]
# 方案一: Claude Code 官方插件 ralph-wiggum, Stop hook 拦下退出并把原 prompt 喂回
/ralph-loop "修好 tests/ 下全部失败用例. 全绿后输出 <promise>DONE</promise>" \
  --completion-promise "DONE" --max-iterations 50

# 方案二: 同目录 eval_harness.py, submit 门槛 + 续写 + 判错重试 + best-of-N
# key 留在宿主机, 命令经 docker exec 进 --network none 容器, 评分前还原 tests/
python eval_harness.py   # 读 task.md, 仓库放 ./repo
```

Ralph 的 stop hook 输出的就是 `{"decision":"block","reason":<原 prompt>}`, 退出口只剩完成口令和迭代上限, 跟评测的 submit 门槛同一个形状. 侧车 [eval_harness.py](eval_harness.py) 是方案二的完整实现, 没在本机跑过, 镜像要自带测试依赖.

想要现成的评测环境, 直接用开源的: [Inspect AI](https://github.com/UKGovernmentBEIS/inspect_ai) 配 inspect_swe 能把 Claude Code 当 solver 跑, [Harbor](https://github.com/harbor-framework/harbor) 是 Terminal-Bench 2.0 的官方 harness, [ImpossibleBench](https://github.com/safety-research/impossiblebench) 专测作弊倾向.

## 0x05 代价与防护

| 代价落在哪 | 例子 | 防法 |
|---|---|---|
| 环境里的凭证 | 604 次调用记在评测方的 OpenRouter key 上 | agent loop 和 key 留宿主机, 容器 `--network none` |
| 题目答案的泄露 | 未来 git 历史, HuggingFace 上的数据集 | 删 refs + `reflog expire` + `gc --prune`, 搜索结果按 benchmark 名屏蔽 |
| 评分器本身 | 改测试, 顺调用栈取参考答案 | 评分前还原测试文件, 评分代码与 agent 分进程 |
| 读通过率选模型的人 | 通过率里混着撞上泄露的那部分 | 跑转写审查, 人工复核后按失败重新计分 |

这笔账只在题目不可解、有歧义、或正当办法试完时才付. 能做的题早就交卷了.

据 Snorkel 的 Ping or Peek, Claude Code 默认设置下 Claude 系读 grader 的比例是 60%. 这 60% 是在放了 grader 的仓库里测的; 日常仓库里没有 grader, 自然看不到.

上评测前先把容器里的 key、Docker socket、git refs 和评分代码各查一遍.
