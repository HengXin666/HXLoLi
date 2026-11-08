# provenance: skill (Agent Skill 编写最佳实践)

采集方式: web_fetch 抓正文 (静态页); 实测对象用 curl 拉 raw 文件. 获取时间均为 2026-11-08 (Asia/Shanghai).
硬约束: 未搜索、未打开仓库里已有的同题旧稿.

## 来源表

| 编号 | 来源 | 等级 | 获取方式 | 获取时间 | 实际读的 URL |
|---|---|---|---|---|---|
| S1 | Anthropic, Skill authoring best practices (Claude Platform Docs) | T1 | web_fetch 全文 | 2026-11-08 | https://platform.claude.com/docs/en/agents-and-tools/agent-skills/best-practices |
| S2 | Agent Skills Specification (agentskills.io, 开放标准正文) | T1 | web_fetch 全文 | 2026-11-08 | https://agentskills.io/specification |
| S3 | Agent Skills, Optimizing skill descriptions | T1 | web_fetch 全文 | 2026-11-08 | https://agentskills.io/skill-creation/optimizing-descriptions |
| S4 | Agent Skills, Evaluating skill output quality | T1 | web_fetch 全文 | 2026-11-08 | https://agentskills.io/skill-creation/evaluating-skills |
| S5 | Claude Code Docs, Extend Claude with skills | T1 | web_fetch 全文 | 2026-11-08 | https://code.claude.com/docs/en/skills |
| S6 | OpenAI, Build skills (Codex / ChatGPT 文档) | T1 | web_fetch 全文 | 2026-11-08 | https://developers.openai.com/codex/skills |
| S7 | Vercel Blog, AGENTS.md outperforms skills in our agent evals (Jude Gao, 2026-01-27) | T2 | web_fetch 的 .md 版本 | 2026-11-08 | https://vercel.com/blog/agents-md-outperforms-skills-in-our-agent-evals.md |
| S8 | OpenAI Developers Blog, Testing Agent Skills Systematically with Evals (Dominik Kundel, Gabriel Chua, 2026-01-22) | T2 | web_fetch 全文 | 2026-11-08 | https://developers.openai.com/blog/eval-skills |
| S9 | Unclecheng-li/AI-Animation-Skill 的 SKILL.md (实测对象, 不作论据) | T3 | curl raw, 存于 `source/check/AI-Animation-Skill/SKILL.md` | 2026-11-08 | https://raw.githubusercontent.com/Unclecheng-li/AI-Animation-Skill/main/SKILL.md |
| L1 | 本地只读副本 hx-note 的 SKILL.md (实测对象, 不作论据) | 实测 | 读文件 | 2026-11-08 | /workspace/work/hx/skill/HXLoLi/.agents/skills/hx-note/SKILL.md |

等级说明: S2 到 S4 是开放标准站点的规范正文与官方作者指南, 记 T1. S7 是知名机构的自家评测, 记 T2, 正文一律写「据 Vercel」. S8 是 OpenAI 员工署名的博客, 记 T2. S9 是社区仓库, 只当迁移算子的测量对象.

## 版本时间点

- S5 页面内出现的 Claude Code 版本号到 v2.1.309; 文中引用的数字 (1536 字符上限, 1% 预算, 5000/25000 token) 以 2026-11-08 抓取为准
- S6 页面以 GPT-6 时代的 ChatGPT/Codex 合并文档为准; 2% / 8000 字符 / 10000 token 以抓取日为准
- S7 发布于 2026-01-27, 评测对象是 Next.js 16 API 与当时的模型

## 已知缺口

- Claude 的分词器未公开, hx-note 正文的 token 数只能用 OpenAI 的 o200k_base / cl100k_base 估算
- Codex 是否识别 Claude Code 的 `disable-model-invocation` 字段: S6 没有提到, 未找到公开资料, 也未在本机跑 Codex 实测
- 本地 HXLoLi 副本只有 `.agents/` 与 `ai-docs/`, 看不到真实仓库里是否有 `.claude/skills` 软链
- S7 的用例条数在原文里没给出 (只有第三方转述, 未采用)
- Anthropic 官方 skill-creator 源码 (anthropics/skills) 未逐行读, 关于它的描述只来自 S3 / S4 / S5 的转述
