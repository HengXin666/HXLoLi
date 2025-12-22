import time
import requests

class ApiReqRateLimiter:
    def __init__(self, max_req_per_sec: int = 60) -> None:
        self._max_req_per_sec = max_req_per_sec
        self._req_count = 0
        self._last_req_time = 0

    def _check_rate_limit(self) -> None:
        """检查请求频率限制"""
        if self._req_count >= self._max_req_per_sec:
            now = time.time()
            if now - self._last_req_time < 1:
                print(f"请求频率限制，等待 {1 - (now - self._last_req_time)} 秒后重试")
                time.sleep(1 - (now - self._last_req_time))
            self._req_count = 0
            self._last_req_time = now
    
    def get(self, url: str, headers: dict, params: dict | None = None) -> requests.Response:
        self._check_rate_limit()
        self._req_count += 1
        return requests.get(url, headers=headers, params=params)
    
    def post(self, url: str, headers: dict, data: dict | None = None) -> requests.Response:
        self._check_rate_limit()
        self._req_count += 1
        return requests.post(url, headers=headers, data=data)