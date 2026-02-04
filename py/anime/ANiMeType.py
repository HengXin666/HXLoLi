from dataclasses import dataclass, field, fields, is_dataclass
from enum import Enum
from typing import Any, List, get_type_hints
import json


# 观看状态枚举
class WatchStatus(Enum):
    """动漫观看状态枚举"""

    WANT_TO_WATCH = 1  # 想看
    WATCHED = 2  # 看过
    WATCHING = 3  # 在看
    ON_HOLD = 4  # 搁置
    DROPPED = 5  # 抛弃


# 主题类型枚举
class SubjectType(Enum):
    """主题类型枚举"""

    BOOK = 1  # 书籍
    ANIME = 2  # 动画
    MUSIC = 3  # 音乐
    GAME = 4  # 游戏
    NO_TYPE = 5  # 没有这个!
    THREE_DIMENSION = 6  # 三次元


class EpisodeType(Enum):
    """剧集类型枚举"""

    NORMAL = 0  # 本篇(正片, 正式集数)
    SP = 1  # 特别篇(SP)
    OP = 2  # OP
    ED = 3  # ED
    PROMO = 4  # 预告/宣传
    MAD = 5     # MAD
    OTHER = 6   # 其他



@dataclass
class KeyValue:
    key: str
    value: str


# 声优信息
@dataclass
class Actor:
    """声优信息数据类"""

    id: int  # 声优ID
    name: str  # 声优名称
    short_summary: str  # 声优简介
    image_url: str  # 声优头像URL


# 角色信息
@dataclass
class Character:
    """角色信息数据类"""

    id: int  # 角色ID
    name: str  # 角色名称
    relation: str  # 角色定位: 主角/配角/客串
    image_url: str  # 角色头像URL
    actor_ids: List[int]  # 声优信息 (此仅存储声优id)

    # characters/{id} 接口返回的数据 {
    name_cn: str = field(default_factory=str)  # 中文名称
    summary: str = field(default_factory=str)  # 角色简介
    birth_year: int | None = None  # 出生年份
    birth_month: int | None = None  # 出生月份
    birth_day: int | None = None  # 出生日期
    tags: List[KeyValue] = field(default_factory=list)  # 角色标签
    # }


# 关联信息
@dataclass
class Relation:
    """关联信息数据类"""

    id: int  # 主题ID
    name: str  # 名称
    name_cn: str  # 中文名称
    relation: str  # 类型: 游戏/插入曲
    type: SubjectType  # 主题类型枚举
    image_url: str  # 封面URL


# 剧集信息
@dataclass
class Episode:
    """剧集信息数据类"""

    id: int  # 剧集ID
    name: str  # 剧集名称
    name_cn: str  # 中文名称
    air_date: str  # 放送日期 (YYYY-MM-DD)
    desc: str  # 剧集简介
    duration_seconds: int  # 时长 (秒)
    ep: int  # 集数 (第几集)
    sort: int  # 排序 (第几集)
    type: EpisodeType  # 类型


# 番剧信息
@dataclass
class ANiMeData:
    """番剧信息数据类"""

    # collections 接口返回的数据 {
    id: int  # 番剧ID
    name: str  # 标题
    name_cn: str  # 中文标题
    short_summary: str  # 简介节选
    score: float  # BanGuMi 评分
    image_url: str  # 封面URL
    eps: int  # 集数 (正片)
    date: str  # 放送日期 (YYYY-MM-DD)
    # } // collections 接口返回的数据

    # subjects/{id} 接口返回的数据 {
    summary: str  # 简介
    total_episodes: int  # 总集数 (包含 OVA)
    # }

    # characters 接口返回的数据 {
    characters: List[Character] = field(default_factory=list)  # 角色信息
    # } // characters 接口返回的数据

    # subjects 接口返回的数据 {
    relations: List[Relation] = field(default_factory=list)  # 关联信息
    # } // subjects 接口返回的数据

    # episodes 接口返回的数据 {
    episodes: List[Episode] = field(default_factory=list)  # 剧集信息
    # }


# 用户状态信息
@dataclass
class UserStatus:
    """用户状态信息数据类"""

    watch_status: WatchStatus  # 观看状态

    # collections 接口返回的数据 {
    watched_eps: int  # 已看集数 (不一定准确)
    last_update: str  # "2025-07-27T18:52:04+08:00"
    # 最后一次更新这个收藏条目状态的时间(例如, 从"想看"改为"在看")
    comment: str  # 用户编写的评论
    tags: List[str] = field(default_factory=list)  # 用户编写的标签
    # } // collections 接口返回的数据


# 番剧记录
@dataclass
class ANiMeRecord:
    """番剧记录数据类"""

    anime_data: ANiMeData  # 番剧信息
    user_status: UserStatus  # 用户状态信息


def shallow_asdict(obj):
    """一个非递归版本的 asdict"""
    if not is_dataclass(obj):
        raise TypeError("shallow_asdict() should be called on dataclass instances")
    result = []
    for f in fields(obj):
        value = getattr(obj, f.name)
        result.append((f.name, value))
    return dict(result)


class ANiMeEncoder(json.JSONEncoder):
    def default(self, obj: Any) -> Any:
        if is_dataclass(obj) and not isinstance(obj, type):
            # 使用非递归的 asdict
            d = shallow_asdict(obj)
            d["__dataclass__"] = obj.__class__.__name__
            return d

        if isinstance(obj, Enum):
            return obj.value

        return super().default(obj)


CLASS_MAP = {
    "Actor": Actor,
    "Character": Character,
    "Relation": Relation,
    "ANiMeData": ANiMeData,
    "UserStatus": UserStatus,
    "ANiMeRecord": ANiMeRecord,
    "KeyValue": KeyValue,
    "Episode": Episode,
}


class ANiMeDecoder(json.JSONDecoder):
    """自定义JSON解码器, 用于反序列化dataclass和enum对象"""

    def __init__(self, *args, **kwargs):
        """初始化解码器"""
        super().__init__(object_hook=self.object_hook, *args, **kwargs)

    def object_hook(self, obj: Any) -> Any:
        """
        自定义JSON反序列化方法。
        这个方法会从最内层的JSON对象开始, 递归地向外执行。
        """
        # 检查这是否是一个我们编码过的 dataclass 字典
        if isinstance(obj, dict) and "__dataclass__" in obj:
            # 1. 弹出"线索", 获取类名
            class_name = obj.pop("__dataclass__")

            # 2. 从映射中找到对应的类对象
            cls = CLASS_MAP.get(class_name)

            if cls:
                # 3. 在创建实例前, 需要特殊处理 Enum 类型
                # 因为 JSON 中存的是值 (如 1, 2), 而 dataclass 构造函数需要 Enum 成员
                type_hints = get_type_hints(cls)
                for field_name, field_type in type_hints.items():
                    # 检查类型提示是否为一个 Enum 子类
                    if isinstance(field_type, type) and issubclass(field_type, Enum):
                        if field_name in obj:
                            # 将字典中的值 (e.g., 1) 转换为 Enum 成员 (e.g., SubjectType.BOOK)
                            obj[field_name] = field_type(obj[field_name])

                # 4. 使用字典中剩余的键值对作为关键字参数来创建类的实例
                # object_hook 的递归特性保证了嵌套的 dataclass 在此时已经被转换成对象了
                try:
                    return cls(**obj)
                except (TypeError, AttributeError) as e:
                    print(f"Error deserializing {class_name}: {e}")
                    pass  # 如果创建失败, 可以选择返回原字典或进行其他处理

        # 5. 如果不是我们认识的格式, 原样返回
        return obj
