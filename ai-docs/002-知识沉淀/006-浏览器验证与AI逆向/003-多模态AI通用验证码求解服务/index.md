---
title: "用AI从零实现通用验证码求解服务:OhMyCaptcha原理剖析"
created_at: "2026-08-26"
model: "deepseek-v4-pro"
skill: ["hx-make-ai-docs"]
authors: "Heng_Xin"
tags: ["AI验证码求解", "AI逆向", "多模态大模型", "浏览器自动化", "异步任务"]
---

# 用AI从零实现通用验证码求解服务:OhMyCaptcha原理剖析

> [!NOTE]
> 本文由 AI 辅助沉淀, 需要用户 review 后再提交.
>
> 当"看懂图片"这件事从传统 CV 的模板匹配/图像处理, 变成"直接问多模态大模型", 验证码求解的架构会简化多少? 代价又是什么?

## 0x00 背景

本笔记沉淀 [nax 的博客](https://naxbr.com/article/ohmycaptcha-analysis) 文章《用 AI 从零实现通用验证码求解服务：OhMyCaptcha 原理剖析》(2026-06-02). 原文剖析开源项目 OhMyCaptcha: 用 Playwright + 多模态大模型构建覆盖 19 种验证码类型的自托管求解服务.

选择这篇做 AI 逆向视角沉淀的原因:

- 它给出了一个清晰的验证码二分法: **环境检测型(不需要 AI) vs 视觉挑战型(需要 AI)**, 这决定了求解器的技术路线.
- 它展示了用**多模态大模型直接替代传统 CV 模块**的现代做法: VLM 看图返回结构化 JSON(坐标/分类), 而不是 OpenCV 模板匹配/OCR 流水线.
- 它的**双模型后端架构(本地小模型 + 云端大模型)**和 **YesCaptcha 兼容的异步任务队列 + 插件化 Solver**, 是自托管 AI 服务可复用的工程范式.

边界说明: 本文只做架构与原理沉淀, 不提供对真实第三方验证码平台的批量破解步骤. 所有代码演示均来自原文, 练习应限于授权测试环境.

## 0x01 核心结论

### 1.1 验证码二分法: 决定技术路线

| 类型 | 验证码 | 检测/挑战方式 | 破解思路 | 是否需要 AI |
| --- | --- | --- | --- | --- |
| 环境检测型 | reCAPTCHA v3 | 浏览器环境 + 行为评分 | 反检测 + 模拟行为 | 否 |
| 环境检测型 | Cloudflare Turnstile | 浏览器环境 + TLS 指纹 | 反检测 + 点击 | 否 |
| 环境检测型 | hCaptcha(基础) | 浏览器环境 | 反检测 + 点击 | 否 |
| 视觉挑战型 | reCAPTCHA v2(高风险) | 选择包含特定物体的图片 | 视觉模型识别 | 是 |
| 视觉挑战型 | hCaptcha(图片挑战) | 选择匹配的图片 | 视觉模型分类 | 是 |
| 视觉挑战型 | 图片验证码 | 识别扭曲文字/点击目标 | 多模态模型推理 | 是 |
| 视觉挑战型 | FunCaptcha | 选择正确的旋转图片 | 视觉模型分类 | 是 |

OhMyCaptcha 两种都支持: 浏览器自动化处理环境检测型, AI 模型处理视觉挑战型.

### 1.2 整体架构: 异步任务队列 + 插件化求解器

核心是 **YesCaptcha 兼容的异步任务系统**:

```text
客户端 createTask(type, params)
  -> TaskManager 创建任务 -> 异步派发给对应 Solver
  -> Solver 执行求解(浏览器 / AI)
  -> 客户端 getTaskResult(taskId) -> 获取结果
```

为什么异步: 验证码求解通常 5-30 秒, 同步等待会 HTTP 超时. 先返回 taskId, 客户端轮询.

Solver 协议(标准策略模式):

```python
class Solver(Protocol):
    async def solve(self, params: dict[str, Any]) -> dict[str, Any]: ...
```

一个 Solver 实例注册多种任务类型(如 `RecaptchaV3TaskProxyless` / `RecaptchaV3EnterpriseTask` 共用一个 v3_solver). 任务纯内存存储, TTL 10 分钟 — 简单、无外部依赖, 但重启即丢, 不适合高可用.

## 0x02 关键细节

### 2.1 原理一: reCAPTCHA v3(纯浏览器, 无需 AI)

流程: Playwright 打开目标 URL(注入反检测) -> 等待 grecaptcha 加载 -> 调用 `grecaptcha.execute(siteKey, {action})` 获取 token.

反检测脚本(比 CF 场景简单, 因为 Playwright 本身检测特征更少):

```javascript
Object.defineProperty(navigator, 'webdriver', {get: () => undefined});
Object.defineProperty(navigator, 'languages', {get: () => ['en-US', 'en']});
Object.defineProperty(navigator, 'plugins', {get: () => [1, 2, 3, 4, 5]});
window.chrome = {runtime: {}, loadTimes: () => {}, csi: () => {}};
```

关键 JS 处理两种情况: 页面已有 reCAPTCHA 脚本 / 需要手动注入 `api.js?render=key`. 行为模拟: `page.mouse.move` 两段移动 + 间隔, 提高行为评分. 所有求解器统一 3 次重试 + 2s 间隔.

> 对比 [001-CF过盾工程](../001-CF过盾工程-从零实现Turnstile绕过/index.md): Turnstile 需要穿透 Shadow DOM 点击 + TLS 指纹续命; reCAPTCHA v3 只需要在真实浏览器里调用 `execute()` 拿 token — 印证"不同验证码的检测面不同, 破解复杂度也不同".

### 2.2 原理二: reCAPTCHA v2(浏览器 + AI 音频转写)

最复杂, 因为点击 checkbox 后可能弹出视觉挑战. 策略: **遇到视觉挑战就切到音频挑战, 用 AI 转写音频**.

```text
点击 checkbox(iframe[title="reCAPTCHA"] 内 #recaptcha-anchor)
  -> 检查 #g-recaptcha-response 是否有值(>20字符)
  -> 有 -> 直接通过; 没有 -> 走音频路径
  -> 点击 #recaptcha-audio-button
  -> 下载音频(三种 selector 兜底: .rc-audiochallenge-tdownload-link / a[href*='.mp3'] / audio source)
  -> 云端大模型(gpt-5.4)转写: "Transcribe exactly what is spoken, digits only, separated by spaces"
  -> 填入 #audio-response, 点击 #recaptcha-verify-button
```

这展示了"挑战路径切换"的逆向思路: 视觉挑战难解, 但同一验证码的音频挑战可能更弱, 直接绕过硬路径.

### 2.3 原理三: 图片验证码识别(多模态视觉模型, 最有创意)

受 Argus 项目启发, 用多模态大模型直接"看"验证码图片并返回结构化操作指令.

三种类型 + 模型输出:

- `click`: 点击指定目标 -> 目标坐标列表 `[{x, y, label}]`.
- `slide`: 滑块滑动到缺口 -> 缺口坐标 + 滑块坐标 + 拖拽距离.
- `drag_match`: 拖拽匹配(物体->影子) -> 配对坐标列表 `[{from, to}]`.

**标准化坐标空间**: 所有图片缩放为 1440x900(LANCZOS), 让模型输出坐标通用. 这正是"把不同来源的问题统一到固定接口"的逆向工程手法.

System Prompt 设计是成功关键(原文特别强调 slide 类型的陷阱):

```text
You are a Computer Vision Data Annotation Assistant.
Input Image: 1440x900 pixels, origin (0,0) at top-left.
Step 1 -- Identify the CAPTCHA type: "click" / "slide" / "drag_match"
Step 2 -- Return STRICT JSON only.

// slide 类型: "gap" 是背景图上的拼图缺口, 不是浮动的拼图块!
// 模型容易把缺口和浮动拼图块混淆, 必须显式纠正
```

调用参数: 本地模型 Qwen3.5-2B, `temperature=0.05`(接近确定性), `max_tokens=1024`. JSON 解析容错: 处理 ```json ... ``` 代码块包裹.

### 2.4 原理四: 图片分类验证码(按类型定制 Prompt)

识别 = "看懂图片内容并返回坐标"; 分类 = "判断图片是否包含指定目标". 为每种分类任务设计专门 System Prompt:

| 任务 | 输入 | 输出格式 |
| --- | --- | --- |
| hCaptcha 分类 | 单图/多图 + 问题 | `{"answer": true/false}` 或 `{"answer": [0, 2, 5]}` |
| reCAPTCHA v2 分类 | 3x3 或 4x4 宫格 | `{"objects": [0, 3, 6]}`(0-indexed, 左到右, 上到下) |
| FunCaptcha 分类 | 2x3 宫格 | `{"objects": [3]}` |

多图片输入统一打包(兼容 `image` / `images` / `body` / `queries` 字段); base64 数据统一补 MIME 前缀. 这是"适配多种调用方输入格式"的接口归一化.

### 2.5 原理五: 双模型后端架构

```text
本地模型 (SGLang)  Qwen3.5-2B       -> 图片验证码识别 / 图片分类(调用频繁, 延迟低, 免费)
云端模型 (API)     gpt-5.4          -> reCAPTCHA v2 音频转写 / 复杂推理(频率低, 能力强, 有成本)
```

为什么分两个: 图片识别/分类调用非常频繁, 每次都打远程 API 慢且贵 -> 本地部署小模型; 音频转写频率低但需要强推理 -> 用云端大模型. 两者都暴露 OpenAI 兼容 `/v1/chat/completions`, 代码可以统一.

配置向后兼容: 新环境变量优先, 旧变量 fallback(如 `CLOUD_BASE_URL` -> `CAPTCHA_BASE_URL`).

### 2.6 最小可运行版本(原文)

FastAPI + 内存 dict 任务存储 + 两个 solver(reCAPTCHA v3 / 图片识别), 展示了完整骨架: 反检测 -> 浏览器取 token -> 多模态看图 -> 异步任务 API. 这 100 行左右的骨架就是"把原理变成可运行服务"的最小闭环.

## 0x03 验证与引用

- 原文: [用 AI 从零实现通用验证码求解服务：OhMyCaptcha 原理剖析](https://naxbr.com/article/ohmycaptcha-analysis), nax 的博客, 2026-06-02.
- 姊妹篇: [深入理解 Cloudflare 过盾原理](../001-CF过盾工程-从零实现Turnstile绕过/index.md)(本分类 001).
- 双模型架构与 [005-LLM与Agent](../../005-LLM与Agent/001-Agent要懂的LLM基础知识/index.md) 中"能力分层"的思想一致: 高频低难任务用低成本模型, 低频高难任务用强模型.
- 边界说明: 本文只沉淀架构与原理, 不提供对真实第三方验证码平台的批量破解步骤; 练习应限于授权测试环境.

## 0x04 总结

读这篇笔记, 需要问自己的问题是:

> 当"理解图片"从规则代码变成模型推理, 你的求解系统是变简单了, 还是把复杂度转移到了 prompt 工程和模型选型上?

- 如果只看实现, VLM 直出坐标确实省掉了传统 CV 的大量图像处理代码.
- 但 system prompt 的陷阱纠正(slide 的 gap vs 拼图块)、坐标空间标准化、双模型成本分层, 才是这套架构真正需要逆向建模的地方.

对通用 AI 逆向而言, OhMyCaptcha 最值得带走的是:**用多模态大模型作为"通用感知接口", 把不同验证码统一成"看图 -> 结构化 JSON"的协议**, 再配合插件化 Solver 和异步任务队列, 就是一个可自托管的通用逆向求解服务范式.
