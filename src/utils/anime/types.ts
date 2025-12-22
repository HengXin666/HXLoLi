// ================= enums =================

// 对应 WatchStatus
export enum WatchStatus {
  WANT_TO_WATCH = 1, // 想看
  WATCHED = 2, // 看过
  WATCHING = 3, // 在看
  ON_HOLD = 4, // 搁置
  DROPPED = 5, // 抛弃
}

// 对应 SubjectType
export enum SubjectType {
  BOOK = 1, // 书籍
  ANIME = 2, // 动画
  MUSIC = 3, // 音乐
  GAME = 4, // 游戏
  NO_TYPE = 5, // 没有这个!
  THREE_DIMENSION = 6, // 三次元
}

// 对应 EpisodeType
export enum EpisodeType {
  NORMAL = 0, // 本篇(正片, 正式集数)
  SP = 1, // 特别篇(SP)
  OP = 2, // OP
  ED = 3, // ED
  PROMO = 4, // 预告/宣传
  OTHER = 5, // MAD 等其他
}

// ================= basic structs =================

// 对应 KeyValue
export interface KeyValue {
  key: string; // 键
  value: string; // 值
}

// ================= actor / character =================

// 对应 Actor
export interface Actor {
  id: number; // 声优ID
  name: string; // 声优名称
  short_summary: string; // 声优简介
  image_url: string; // 声优头像URL
}

// 对应 Character
export interface Character {
  id: number; // 角色ID
  name: string; // 角色名称
  relation: string; // 角色定位: 主角/配角/客串
  image_url: string; // 角色头像URL
  actor_ids: number[]; // 声优ID列表

  // characters/{id} 接口补充字段
  name_cn: string; // 中文名称
  summary: string; // 角色简介
  birth_year: number | null; // 出生年份
  birth_month: number | null; // 出生月份
  birth_day: number | null; // 出生日期
  tags: KeyValue[]; // 角色标签
}

// ================= relation =================

// 对应 Relation
export interface Relation {
  id: number; // 主题ID
  name: string; // 名称
  name_cn: string; // 中文名称
  relation: string; // 关联类型: 游戏/插入曲等
  type: SubjectType; // 主题类型
  image_url: string; // 封面URL
}

// ================= episode =================

// 对应 Episode
export interface Episode {
  id: number; // 剧集ID
  name: string; // 剧集名称
  name_cn: string; // 中文名称
  air_date: string; // 放送日期(YYYY-MM-DD)
  desc: string; // 剧集简介
  duration_seconds: number; // 时长(秒)
  ep: number; // 集数(第几集)
  sort: number; // 排序用集数
  type: EpisodeType; // 剧集类型
}

// ================= anime data =================

// 对应 ANiMeData
export interface ANiMeData {
  // collections 接口字段
  id: number; // 番剧ID
  name: string; // 标题
  name_cn: string; // 中文标题
  short_summary: string; // 简介节选
  score: number; // BanGuMi评分
  image_url: string; // 封面URL
  eps: number; // 正片集数
  date: string; // 放送日期(YYYY-MM-DD)

  // subjects/{id} 接口字段
  summary: string; // 完整简介
  total_episodes: number; // 总集数(包含OVA等)

  // characters 接口字段
  characters: Character[]; // 角色信息列表

  // subjects 接口字段
  relations: Relation[]; // 关联信息列表

  // episodes 接口字段
  episodes: Episode[]; // 剧集信息列表
}

// ================= user status =================

// 对应 UserStatus
export interface UserStatus {
  watch_status: WatchStatus; // 观看状态
  watched_eps: number; // 已观看集数
  last_update: string; // 最后更新时间(ISO 8601)
  comment: string; // 用户评论
  tags: string[]; // 用户标签
}

// ================= record =================

// 对应 ANiMeRecord
export interface ANiMeRecord {
  anime_data: ANiMeData; // 番剧信息
  user_status: UserStatus; // 用户状态
}
