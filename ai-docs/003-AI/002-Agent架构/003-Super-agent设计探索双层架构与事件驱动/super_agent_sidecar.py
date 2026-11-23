"""Super-agent 最小调度骨架: 过滤 / 订阅直通 / Router 批处理 / 异步任务.

只用标准库, LLM 用确定性的 mock 函数代替. 直接运行: python3 super_agent_sidecar.py
字段名是自拟的, 语义对应 DeepSeek Harness《Super-agent 设计探索》3.1 / 3.2 / 2.1 / 4.1 节.
"""
from __future__ import annotations

import asyncio
from collections import defaultdict
from dataclasses import dataclass, field
from itertools import count


@dataclass
class Trigger:
    id: str
    source: str              # "im" / "github" / "ci" / "task"
    thread_id: str
    text: str
    ref_ids: list[str] = field(default_factory=list)

    @property
    def summary(self) -> str:   # Inline: 只带前 50 字, 其余 Spill
        return self.text[:50]


# ---- 1. 程序层准入 (Fast Path 第一道) ----
ADMISSION = {                # source -> explicit_only / candidate / ignore
    "im": "candidate",
    "github": "candidate",
    "ci": "candidate",
    "bot": "ignore",
    "task": "candidate",     # 异步任务完成后回投的事件
}


def admit(t: Trigger) -> bool:
    mode = ADMISSION.get(t.source, "ignore")
    if mode == "ignore":
        return False
    if mode == "explicit_only":
        return "@agent" in t.text
    return True


# ---- 2. 订阅表: thread_id -> worker_id 集合, 多对多 ----
SUBSCRIPTIONS: dict[str, set[str]] = defaultdict(set)
_worker_seq = count(1)


def mock_router_llm(batch: list[Trigger]) -> list[tuple[Trigger, str]]:
    """mock 的 Router 模型: 一次调用处理整批, 每条只给一个短决定."""
    decisions = []
    for t in batch:
        if any(w in t.text for w in ("修", "加", "review", "失败")):
            decisions.append((t, "spawn"))
        else:
            decisions.append((t, "ignore"))
    return decisions


class Router:
    def __init__(self) -> None:
        self.inbox: asyncio.Queue[Trigger] = asyncio.Queue()
        self.llm_calls = 0
        self.decisions = 0

    async def run(self, workers: "dict[str, Worker]") -> None:
        while True:
            first = await self.inbox.get()
            batch = [first]
            while not self.inbox.empty():           # 推理期间积压的一并带上
                batch.append(self.inbox.get_nowait())
            self.llm_calls += 1
            await asyncio.sleep(0.05)               # 假装在推理
            for t, d in mock_router_llm(batch):
                self.decisions += 1
                if d == "spawn":
                    wid = f"worker-{next(_worker_seq)}"
                    SUBSCRIPTIONS[t.thread_id].add(wid)   # 判断落成程序状态
                    workers[wid] = Worker(wid, self)
                    await workers[wid].handle(t)
            print(f"  [router] 第 {self.llm_calls} 次调用, batch={len(batch)}")


class Worker:
    def __init__(self, wid: str, router: Router) -> None:
        self.wid = wid
        self.router = router
        self.seen: list[str] = []

    async def handle(self, t: Trigger) -> None:
        self.seen.append(t.id)
        if "跑测试" in t.text:
            task_id = start_async_task(self.wid, t.thread_id, seconds=0.3)
            print(f"  [{self.wid}] {t.id}: 发起 {task_id} -> Started, 不阻塞")
        else:
            print(f"  [{self.wid}] {t.id}: {t.summary}")


# ---- 4. 异步任务: 立即返回 Started, 完成后作为新 Trigger 回投 ----
_task_seq = count(1)
_pending: list[asyncio.Task] = []
DISPATCH_Q: asyncio.Queue[Trigger] = asyncio.Queue()


def start_async_task(wid: str, thread_id: str, seconds: float) -> str:
    task_id = f"task-{next(_task_seq)}"

    async def _job() -> None:
        await asyncio.sleep(seconds)
        await DISPATCH_Q.put(Trigger(f"{task_id}-done", "task", thread_id,
                                     f"{task_id} 完成: 12 passed", [wid, task_id]))

    _pending.append(asyncio.create_task(_job()))
    return task_id


# ---- 3. 分发: 过滤 -> 已订阅直通 Worker / 未订阅进 Router 收件箱 ----
async def dispatcher(router: Router, workers: dict[str, Worker], stats: dict) -> None:
    while True:
        t = await DISPATCH_Q.get()
        if not admit(t):
            stats["dropped"] += 1
            continue
        subs = SUBSCRIPTIONS.get(t.thread_id)
        if subs:                                    # Fast Path: 不调模型
            stats["direct"] += 1
            for wid in subs:
                await workers[wid].handle(t)
        else:                                       # Slow Path: 交给 Router
            stats["to_router"] += 1
            await router.inbox.put(t)


async def main() -> None:
    router, workers = Router(), {}
    stats = {"dropped": 0, "direct": 0, "to_router": 0}
    tasks = [asyncio.create_task(router.run(workers)),
             asyncio.create_task(dispatcher(router, workers, stats))]

    burst = [
        Trigger("m1", "im", "im-42", "登录页按钮错位, 谁能修一下"),
        Trigger("m2", "bot", "im-42", "[机器人] 今日站会提醒"),
        Trigger("m3", "github", "pr-7", "PR #7 review: 缓存层加个 TTL"),
        Trigger("m4", "im", "im-99", "中午吃什么"),
        Trigger("m5", "ci", "pr-8", "CI 失败: test_auth 超时"),
    ]
    print("== 突发: 5 条同时到达")
    for t in burst:
        await DISPATCH_Q.put(t)
    await asyncio.sleep(0.2)

    print("== 后续: 同一 Thread 的跟进")
    for t in [Trigger("m6", "im", "im-42", "顺便把 hover 颜色也改了"),
              Trigger("m7", "github", "pr-7", "跑测试看看")]:
        await DISPATCH_Q.put(t)
    await asyncio.sleep(0.5)

    print("== 统计")
    print(f"  丢弃 {stats['dropped']}, 直通 {stats['direct']}, 进 Router {stats['to_router']}")
    print(f"  Router 调用 {router.llm_calls} 次, 做了 {router.decisions} 个决定")
    print(f"  订阅表 {dict(SUBSCRIPTIONS)}")
    for t in tasks + _pending:
        t.cancel()


if __name__ == "__main__":
    asyncio.run(main())
