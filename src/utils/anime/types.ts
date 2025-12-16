// 对应 WatchStatus
export enum WatchStatus {
  WANT_TO_WATCH = 1,
  WATCHED = 2,
  WATCHING = 3,
  ON_HOLD = 4,
  DROPPED = 5,
}

// 对应 SubjectType
export enum SubjectType {
  BOOK = 1,
  ANIME = 2,
  MUSIC = 3,
  GAME = 4,
  NO_TYPE = 5,
  THREE_DIMENSION = 6,
}

// 对应 Actor
export interface Actor {
  id: number;
  name: string;
  short_summary: string;
  image_url: string;
}

// 对应 Character
export interface Character {
  id: number;
  name: string;
  relation: string;
  image_url: string;
  actor_ids: number[];
}

// 对应 Relation
export interface Relation {
  id: number;
  name: string;
  name_cn: string;
  relation: string;
  type: SubjectType; // 直接使用 enum 类型
  image_url: string;
}

// 对应 ANiMeData
export interface ANiMeData {
  id: number;
  name: string;
  name_cn: string;
  summary: string;
  score: number;
  image_url: string;
  eps: number;
  date: string;
  characters: Character[];
  relations: Relation[];
}

// 对应 UserStatus
export interface UserStatus {
  watch_status: WatchStatus; // 直接使用 enum 类型
  watched_eps: number;
  last_update: string;
  comment: string;
  tags: string[];
}

// 对应 ANiMeRecord
export interface ANiMeRecord {
  anime_data: ANiMeData;
  user_status: UserStatus;
}
