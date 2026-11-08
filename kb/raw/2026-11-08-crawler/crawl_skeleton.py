import asyncio
import random

import httpx


async def crawl(urls, concurrency=8, delay=1.0, jitter=0.5):
    sem = asyncio.Semaphore(concurrency)
    # 连接/读/写各 10s; pool=None: 在信号量后面排队的请求不会因为等连接而超时
    timeout = httpx.Timeout(10.0, pool=None)
    limits = httpx.Limits(max_connections=concurrency)
    async with httpx.AsyncClient(timeout=timeout, limits=limits, follow_redirects=True) as client:

        async def one(url):
            async with sem:
                await asyncio.sleep(delay * random.uniform(1 - jitter, 1 + jitter))
                r = await client.get(url)
                r.raise_for_status()
                return r.text

        return await asyncio.gather(*(one(u) for u in urls), return_exceptions=True)
