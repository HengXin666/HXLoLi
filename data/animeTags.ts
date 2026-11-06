// 番剧标签的**词表渲染部分**. 这里只定义"有哪些标签、怎么显示、属于哪组、推荐分类去哪个专属页",
// 不再含任何"哪部番是什么标签"的分配.
//
// 2026-11-04 tag 重构 (hx-anime-tag-refactor-design.md 第 3.1 / 3.3 / 4 / 10 节):
//   - 主站只读 HXLoLi-ANiMe/data/labels/<bgm_subject_id>.json 的人工标注 (加载见 src/utils/anime/labels.ts).
//     未标注 = 没有文件 = 不展示任何 tag.
//   - 原来的手工分配表 ANIME_TAGS (bgm id -> 标签) 已从本文件移除, 历史见 git; 它与 user_status.tags、
//     data/animeOverlay.json 的 tags、HXLoLi-ANiMe/data/ANiMeTags.json 一起, 都**不再参与渲染**.
//   - 本文件是 HXLoLi-ANiMe/data/labelVocab.json (version 1) 的镜像: 主站与数据子仓库是两个仓库, 主站构建时拿不到子仓库文件,
//     所以抄一份. 词表的唯一来源仍是 labelVocab.json: 增删标签先改那边, 再同步这里 (display / slug 必须逐字一致).

/** 镜像的 labelVocab.json 版本; 那边升版本时这里跟着改 */
export const ANIME_LABEL_VOCAB_VERSION = 1;

// ---------------------------------------------------------------- 推荐分类 (susume, 有专属页, 不走 ?tag= 筛选)

export type AnimeSusumeKey = "monosugoi" | "loli" | "loligaaru" | "imouto";

export interface AnimeSusumeDef {
    key: AnimeSusumeKey;
    /** 标注里的标签名 (= labelVocab.json susume[].display) */
    display: string;
    /** bgm 同步名 */
    bgmTag: string;
    /** 专属页 slug: /anime/susume/<slug> */
    slug: string;
    /** 专属页 / 入口卡上的副标题 */
    subtitle: string;
    /** 判定口径 (来自 labelVocab.json 的 note) */
    rule: string;
}

/** 顺序即首页入口卡、详情页徽章的顺序 */
export const ANIME_SUSUME_DEFS: readonly AnimeSusumeDef[] = [
    {
        key: "monosugoi",
        display: "物凄い",
        bgmTag: "物凄い",
        slug: "monosugoi",
        subtitle: "我最喜欢的番",
        rule: "最爱, 只在观后感模式邀请勾选",
    },
    {
        key: "loli",
        display: "ロリ",
        bgmTag: "ロリ",
        slug: "loli",
        subtitle: "萝莉为卖点",
        rule: "主角或核心角色是小学生年纪 / 幼女外形 (含合法ロリ), 且是作品卖点之一; 只在回忆或 Q 版里出现的不算, 初中生不算",
    },
    {
        // 2026-11-06: 由普通标注升为推荐分类 (用户: ロリがある 算 susume). 比 ロリ 宽: 有这样的角色就算, 不要求是卖点
        key: "loligaaru",
        display: "ロリがある",
        bgmTag: "ロリがある",
        slug: "loli-ga-aru",
        subtitle: "有萝莉角色",
        rule: "作品里有外形或年龄偏小的角色, 哪怕只有一个、不是卖点也算; 标了 ロリ 的番自动带上, 不必重复勾选",
    },
    {
        key: "imouto",
        display: "シスコン",
        bgmTag: "シスコン",
        slug: "imouto",
        subtitle: "妹控",
        rule: "旧名 妹控 废弃, 不做自动迁移",
    },
];

// ---------------------------------------------------------------- 普通标注 (genre, 走 /anime?tag= 筛选)

/** 普通标注只有一组; 筛选面板按组分栏 (见 components/anime/Tag/TagFilter.tsx) */
export type AnimeTagGroup = "标注";

export const ANIME_TAG_GROUPS: readonly AnimeTagGroup[] = ["标注"];

export type AnimeTagDef = {
    tag: string;
    group: AnimeTagGroup;
    /** bgm 同步名 */
    bgmTag: string;
    /** 判定口径, 筛选面板里显示; 新词表里没写口径的留空 */
    rule?: string;
};

/** 顺序即 labelVocab.json genre[] 的顺序, 也是筛选面板与卡片上标签的顺序 */
export const ANIME_TAG_DEFS: readonly AnimeTagDef[] = [
    { tag: "异世界", bgmTag: "异世界", group: "标注", rule: "转生 / 穿越 / 召唤到另一个世界, 或以异世界为主舞台" },
    { tag: "热血", bgmTag: "热血", group: "标注" },
    { tag: "战斗", bgmTag: "战斗", group: "标注", rule: "战斗 / 冒险是主线" },
    { tag: "推理", bgmTag: "推理", group: "标注" },
    { tag: "校园", bgmTag: "校园", group: "标注", rule: "学校 / 学生会 / 社团是主舞台 (大学社团也算); 主角只是学生身份、故事不在学校里展开的不算" },
    { tag: "日常", bgmTag: "日常", group: "标注", rule: "以日常生活为主, 没有贯穿全篇的主线冲突" },
    { tag: "搞笑", bgmTag: "搞笑", group: "标注", rule: "以喜剧 / 吐槽为主要看点" },
    { tag: "恋爱", bgmTag: "恋爱", group: "标注", rule: "恋爱关系是主线" },
    { tag: "致郁", bgmTag: "致郁", group: "标注" },
];

/**
 * 隐含标签: 标了左边的番, 渲染时自动带上右边的 (标注文件里不必重复勾选). 镜像 labelVocab.json genre[].impliedBy.
 * 2026-11-06: ロリ (卖点) 一定也是 ロリがある (有这样的角色). 两者现在都是推荐分类, 隐含关系不变.
 */
export const ANIME_TAG_IMPLIES: Readonly<Record<string, readonly string[]>> = {
    ロリ: ["ロリがある"],
};

/**
 * 旧链接里的标签名 -> 新词表名. **只用于解析 URL** (旧式 /anime?tag=萝莉 / ?tag=妹控 跳到对应 susume 页),
 * 不是数据迁移: 标注文件里只会出现词表内的名字. 旧标签 百合 / 催泪 / 悬疑 不在新词表, 不做映射.
 */
export const ANIME_TAG_ALIASES: Readonly<Record<string, string>> = {
    纯爱: "恋爱",
    "\u841d\u8389": "ロリ",
    "\u59b9\u63a7": "シスコン",
};
