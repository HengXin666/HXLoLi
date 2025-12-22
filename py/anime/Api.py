import argparse
import os
from queue import Queue
import threading
from typing import Callable, Dict, List, Tuple
import requests
import json
import time

from ANiMeType import (
    ANiMeData,
    ANiMeDecoder,
    ANiMeEncoder,
    ANiMeRecord,
    Actor,
    Character,
    Episode,
    KeyValue,
    Relation,
    SubjectType,
    UserStatus,
)
from ApiReqRateLimiter import ApiReqRateLimiter

# --- 配置 ---
# 替换为你的 Bangumi 用户名
USERNAME = "heng_xin"
# 替换为你的 Access Token (在 https://next.bgm.tv/demo/access-token 生成)
ACCESS_TOKEN = ""
# API 的基础 URL
BASE_URL = "https://api.bgm.tv"
# 数据存储路径
DATA_ROOT_PATH = "./data"

# --- 请求头 ---
# 强烈建议设置 User-Agent, 这是 API 文档的要求
# 格式: App-Name/Version (your-github-url or your-email)
HEADERS = {
    "Authorization": f"Bearer {ACCESS_TOKEN}",
    "User-Agent": "HXLoLi/1.0 (https://github.com/HengXin666/HXLoLi)",
    "accept": "application/json",
}

# 带请求频率限制的 api 请求
apiReq = ApiReqRateLimiter(119)

class Api:
    def __init__(
        self, anime_record: List[ANiMeRecord] = [], actor_map: Dict[int, Actor] = {}
    ) -> None:
        self._anime_record: List[ANiMeRecord] = anime_record  # 番剧记录
        self._actor_map: Dict[int, Actor] = actor_map  # 声优信息

        self._record_map: Dict[int, ANiMeRecord] = {}  # 番剧记录索引映射
        for record in self._anime_record:
            self._record_map[record.anime_data.id] = record

    @staticmethod
    def _get_img_and_save(id: int, type: str, url: str) -> bool:
        if url == "":
            return False
        save_path = f"{DATA_ROOT_PATH}/{type}/{id}.jpg"
        if not os.path.exists(f"{DATA_ROOT_PATH}/{type}"):
            os.makedirs(f"{DATA_ROOT_PATH}/{type}")
        if os.path.exists(save_path):
            return False
        try:
            response = requests.get(url, headers=HEADERS)
            response.raise_for_status()
            with open(save_path, "wb") as f:
                f.write(response.content)
        except requests.exceptions.RequestException as e:
            print(f"获取图片失败: {e}")
        except json.JSONDecodeError:
            print("解析图片响应失败, 内容可能不是有效的JSON。")
            print("响应内容:", response.text)
        return True

    def requires(self, username: str, lambda_func: Callable = lambda: None) -> None:
        def _task(idx: int) -> None:
            print(f"{idx}: {self._anime_record[idx].anime_data.name}")
            self._get_anime_characters(idx)
            self._get_relations(idx)
            self._get_episodes(idx)
            lambda_func()

        self._get_user_watching_anime(username, _task)

    def download_all_img(self) -> None:
        def _download_img_thread(queue: Queue[Tuple[str, int, str]]) -> None:
            cnt: int = 0
            while not queue.empty():
                type, id, url = queue.get()
                is_ok = self._get_img_and_save(id, type, url)
                cnt += is_ok
                if is_ok:
                    print(f"[{threading.current_thread().name}]: 已下载 {id} 图片")
                    if cnt % 100 == 0:
                        print(
                            f"[{threading.current_thread().name}]: 已下载 {cnt} 张图片"
                        )
                        time.sleep(3)
                queue.task_done()

        # 基于线程安全的队列 <type, id, url> 开启多线程爬虫
        queue: Queue[Tuple[str, int, str]] = Queue()
        for record in self._anime_record:
            queue.put(("anime", record.anime_data.id, record.anime_data.image_url))
            for actor in record.anime_data.characters:
                queue.put(("actor", actor.id, actor.image_url))
            for relation in record.anime_data.relations:
                queue.put(("relation", relation.id, relation.image_url))
        for _ in range(16):
            threading.Thread(target=_download_img_thread, args=(queue,)).start()
        print("主线程: 等待所有下载任务完成...")
        queue.join()
        print("主线程: 所有图片下载任务已完成! ")

    def _get_subjects_info(self, id: int, record_ref: ANiMeRecord) -> None:
        url = f"{BASE_URL}/v0/subjects/{id}"
        try:
            response = requests.get(url, headers=HEADERS)
            response.raise_for_status()
            # 补充更新
            data = response.json()
            record_ref.anime_data.name = data["name"]
            record_ref.anime_data.name_cn = data["name_cn"]
            record_ref.anime_data.summary = data["summary"]
            record_ref.anime_data.eps = data["eps"]
            record_ref.anime_data.total_episodes = data["total_episodes"]
            record_ref.anime_data.date = data["date"]
        except requests.exceptions.RequestException as e:
            print(f"获取番剧信息失败: {e}")
        except json.JSONDecodeError:
            print("解析番剧信息响应失败, 内容可能不是有效的JSON。")
            print("响应内容:", response.text)

    def _get_user_watching_anime(self, username: str, lambda_func: Callable) -> None:
        url = f"{BASE_URL}/v0/users/{username}/collections"
        limit = 100
        for i in range(99999999):
            params = {
                "subject_type": 2,  # 仅请求动画
                "limit": limit,  # 限制返回的条数
                "offset": i * limit,  # 偏移量
            }
            try:
                response = apiReq.get(url, headers=HEADERS, params=params)
                # 如果请求失败, 则抛出异常
                response.raise_for_status()
                for it in response.json().get("data", []):
                    if it["subject_id"] in self._record_map:
                        # 仅更新 UserStatus
                        ref = self._record_map[it["subject_id"]]
                        ref.user_status.watch_status = it["type"]
                        ref.user_status.watched_eps = it["ep_status"]
                        ref.user_status.last_update = it["updated_at"]
                        ref.user_status.comment = it["comment"]
                        ref.user_status.tags = it["tags"]
                        continue
                    record = ANiMeRecord(
                        ANiMeData(
                            id=it["subject_id"],
                            name=it["subject"]["name"],
                            name_cn=it["subject"]["name_cn"],
                            short_summary=it["subject"]["short_summary"],
                            summary="",
                            score=it["subject"]["score"],
                            image_url=it["subject"]["images"]["large"],
                            eps=it["subject"]["eps"],
                            total_episodes=it["subject"]["eps"],
                            date=it["subject"]["date"],
                        ),
                        UserStatus(
                            watch_status=it["type"],
                            watched_eps=it["ep_status"],
                            last_update=it["updated_at"],
                            comment=it["comment"],
                            tags=it["tags"],
                        ),
                    )
                    self._get_subjects_info(record.anime_data.id, record)  # 补充更新
                    self._record_map[record.anime_data.id] = record
                    self._anime_record.append(record)
                    lambda_func(len(self._anime_record) - 1)
                if len(self._anime_record) < limit * (i + 1):
                    return
            except requests.exceptions.RequestException as e:
                print(f"获取用户追番列表失败: {e}")
                return
            except json.JSONDecodeError:
                print("解析追番列表响应失败, 内容可能不是有效的JSON。")
                print("响应内容:", response.text)
                return

    def _get_anime_kara_data(self, kara_ref: Character) -> None:
        """获取角色详细信息"""
        url = f"{BASE_URL}/v0/characters/{kara_ref.id}"
        try:
            response = apiReq.get(url, headers=HEADERS)
            response.raise_for_status()
            data = response.json()
            kara_ref.name_cn = data["infobox"][0]["value"]
            kara_ref.summary = data["summary"]
            kara_ref.birth_year = data.get("birth_year", None)
            kara_ref.birth_month = data.get("birth_mon", None)
            kara_ref.birth_day = data.get("birth_day", None)
            for kv in data["infobox"]:
                if kv["key"] == "简体中文名" or kv["key"] == "别名":
                    continue
                kara_ref.tags.append(KeyValue(key=kv["key"], value=kv["value"]))
        except requests.exceptions.RequestException as e:
            print(f"获取角色信息失败: {e}")
        except json.JSONDecodeError:
            print("解析角色信息响应失败, 内容可能不是有效的JSON。")
            print("响应内容:", response.text)

    def _get_anime_characters(self, data_idx: int) -> None:
        """获取番剧角色声优信息
        Args:
            data_idx (int): _anime_record 中的索引
        Returns:
            List[ANiMeRecord]: _description_
        """
        url = f"{BASE_URL}/v0/subjects/{self._anime_record[data_idx].anime_data.id}/characters"
        try:
            response = apiReq.get(url, headers=HEADERS)
            response.raise_for_status()
            for it in response.json():
                character = Character(
                    id=it["id"],
                    name=it["name"],
                    relation=it["relation"],
                    image_url=it["images"]["large"],
                    actor_ids=[actor["id"] for actor in it["actors"]],
                )
                self._get_anime_kara_data(character)
                self._anime_record[data_idx].anime_data.characters.append(character)

                # 记录角色的声优
                for actor in it["actors"]:
                    if actor["id"] not in self._actor_map:
                        actor_data = Actor(
                            id=actor["id"],
                            name=actor["name"],
                            short_summary=actor["short_summary"],
                            image_url=actor["images"]["large"],
                        )
                        self._actor_map[actor_data.id] = actor_data
        except requests.exceptions.RequestException as e:
            print(
                f"获取番剧 {self._anime_record[data_idx].anime_data.id} 角色失败: {e}"
            )
        except json.JSONDecodeError:
            print(
                f"解析番剧 {self._anime_record[data_idx].anime_data.id} 角色响应失败。"
            )
            print("响应内容:", response.text)

    def _get_relations(self, data_idx: int) -> None:
        """获取番剧关联信息"""
        url = f"{BASE_URL}/v0/subjects/{self._anime_record[data_idx].anime_data.id}/subjects"
        try:
            response = apiReq.get(url, headers=HEADERS)
            response.raise_for_status()
            for it in response.json():
                relation = Relation(
                    id=it["id"],
                    name=it["name"],
                    name_cn=it["name_cn"],
                    relation=it["relation"],
                    type=SubjectType(it["type"]),
                    image_url=it["images"]["large"],
                )
                self._anime_record[data_idx].anime_data.relations.append(relation)
        except requests.exceptions.RequestException as e:
            print(
                f"获取番剧 {self._anime_record[data_idx].anime_data.id} 关联失败: {e}"
            )
        except json.JSONDecodeError:
            print(
                f"解析番剧 {self._anime_record[data_idx].anime_data.id} 关联响应失败。"
            )
            print("响应内容:", response.text)

    def _get_episodes(self, data_idx: int) -> None:
        """获取番剧剧集信息"""
        url = f"{BASE_URL}/v0/episodes"
        limit = 100
        try:
            for i in range(99999999):
                params = {
                    "subject_id": self._anime_record[data_idx].anime_data.id,
                    "limit": limit,
                    "offset": limit * i,
                }
                response = apiReq.get(url, headers=HEADERS, params=params)
                response.raise_for_status()
                for it in response.json()["data"]:
                    episode = Episode(
                        id=it["id"],
                        name=it["name"],
                        name_cn=it["name_cn"],
                        type=it["type"],
                        ep=it["ep"],
                        sort=it["sort"],
                        duration_seconds=it["duration_seconds"],
                        air_date=it["airdate"],
                        desc=it["desc"],
                    )
                    self._anime_record[data_idx].anime_data.episodes.append(episode)
                if len(self._anime_record[data_idx].anime_data.episodes) < limit * (i + 1):
                    return
        except requests.exceptions.RequestException as e:
            print(
                f"获取番剧 {self._anime_record[data_idx].anime_data.id} 剧集失败: {e}"
            )
        except json.JSONDecodeError:
            print(
                f"解析番剧 {self._anime_record[data_idx].anime_data.id} 剧集响应失败。"
            )
            print("响应内容:", response.text)

def load_from_json() -> Api:
    """从 json 文件中加载数据
    运行路径, 应该在 py/anime 下
    如果文件不存在, 则返回一个空的 Api 对象
    """
    if not os.path.exists(DATA_ROOT_PATH):
        os.makedirs(DATA_ROOT_PATH)
    try:
        # 番剧记录
        with open(f"{DATA_ROOT_PATH}/ANiMeRecord.json", "r", encoding="utf-8") as f:
            anime_record: List[ANiMeRecord] = json.load(f, cls=ANiMeDecoder)
    except:
        anime_record = []
    try:
        # 声优信息
        with open(f"{DATA_ROOT_PATH}/Actor.json", "r", encoding="utf-8") as f:
            actor_list: List[Actor] = json.load(f, cls=ANiMeDecoder)
    except:
        actor_list = []
    actor_map: Dict[int, Actor] = {}
    for actor in actor_list:
        actor_map[actor.id] = actor
    return Api(anime_record, actor_map)


def save_to_json(api: Api) -> None:
    """保存数据为 json 文件
        运行路径, 应该在 py/anime 下
    Args:
        api (Api): 数据
    """
    if not os.path.exists(DATA_ROOT_PATH):
        os.makedirs(DATA_ROOT_PATH)
    # 番剧记录
    with open(f"{DATA_ROOT_PATH}/ANiMeRecord.json", "w", encoding="utf-8") as f:
        f.write(
            json.dumps(
                api._anime_record, cls=ANiMeEncoder, indent=4, ensure_ascii=False
            )
        )
    # 声优信息
    with open(f"{DATA_ROOT_PATH}/Actor.json", "w", encoding="utf-8") as f:
        f.write(
            json.dumps(
                list(api._actor_map.values()),
                cls=ANiMeEncoder,
                indent=4,
                ensure_ascii=False,
            )
        )


if __name__ == "__main__":
    # 接受命令行参数
    ag = argparse.ArgumentParser()
    ag.add_argument(
        "-u", "--username", help="Bangumi 用户名", required=False, default="heng_xin"
    )
    ag.add_argument("-o", "--token", help="Bangumi Token", required=True)
    args = ag.parse_args()

    # 重新绑定
    USERNAME = args.username
    ACCESS_TOKEN = args.token
    HEADERS = {
        "Authorization": f"Bearer {ACCESS_TOKEN}",
        "User-Agent": "HXLoLi/1.0 (https://github.com/HengXin666/HXLoLi)",
        "accept": "application/json",
    }

    api = load_from_json()
    # 边爬取边输出为 json 文件
    api.requires("heng_xin", lambda: save_to_json(api))
    # 下载图片
    api.download_all_img()
