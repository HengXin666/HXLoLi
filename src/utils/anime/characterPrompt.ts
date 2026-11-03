/*
 * 番剧角色立绘的生图提示词: 模板只在这一个文件里, 组件只传角色信息进来.
 * 纯函数, 不 import 任何模块 (校验脚本 src/components/anime/CharacterMenu/characterPrompt.check.mjs 直接转译执行它).
 */

/** 拼提示词需要的全部输入. URL 都要求是浏览器可直接打开的绝对地址 */
export interface CharacterPromptInput {
    /** 番剧名 (优先中文名) */
    animeName: string;
    /** 番剧原名, 与 animeName 相同时可省略 */
    animeOriginalName?: string;
    /** 番剧封面 URL (站内 CDN 地址), 作为参考图 */
    coverUrl: string;
    /** 番剧封面在 Bangumi 的原始地址, 作为备用参考图 */
    coverFallbackUrl?: string;
    /** 角色名 (优先中文名) */
    characterName: string;
    /** 角色原名 */
    characterOriginalName?: string;
    /** 角色图片 URL (站内 CDN 地址) */
    characterImageUrl: string;
    /** 角色图片在 Bangumi 的原始地址 */
    characterImageFallbackUrl?: string;
    /** 主角 / 配角 / 客串 */
    relation?: string;
}

export type PeekSide = "left" | "right";

/** 探头立绘的姿势描述: 两个朝向共用, 方向词由 side 决定 */
export const PEEK_POSE =
    "从画面上方往下俯探身子, 上半身探出、下半身留在画外 (半身出画), 头和肩先探出来, 双手收在胸前或轻搭在上边缘内侧 (不碰竖直裁切线), 视线朝画面中央往下看; 镜头是仰角 (从下往上看), 角色俯身、头发和衣摆自然下垂";

const SIDE_TEXT: Record<PeekSide, { place: string; face: string; crop: string; outer: string }> = {
    left: {
        place: "放在页面左侧",
        face: "身体与脸朝右 (朝向页面内侧)",
        crop: "竖直裁切线在画面右边缘 (内侧), 角色被这条直线切开, 像是从正文卡片的左边缘后面探出来",
        outer: "画面左边缘与上边缘留透明空白, 不让肢体碰到外侧边",
    },
    right: {
        place: "放在页面右侧",
        face: "身体与脸朝左 (朝向页面内侧)",
        crop: "竖直裁切线在画面左边缘 (内侧), 角色被这条直线切开, 像是从正文卡片的右边缘后面探出来",
        outer: "画面右边缘与上边缘留透明空白, 不让肢体碰到外侧边",
    },
};

/** 负面清单, 抄自 .agents/research/catgirl-genprompt.md 第 6 节; 探头两份全用, 纯角色那份用与构图无关的几条 */
export const PEEK_NEGATIVE: readonly string[] = [
    "出画处 / 竖直裁切线处不能是手臂、手、手指, 只能是躯干、肩、头发",
    "不能背对正文 (身体朝外), 也不能是\"站在墙边\"露出整个上半身",
    "不能画墙、地台、阴影、光晕、背景、文字、签名、水印",
    "左右两份必须分别生成, 不能把同一张图左右镜像翻转凑成一对",
    "头顶不能被上边缘切掉, 头上方要留出空白",
];

export const CHARACTER_NEGATIVE: readonly string[] = [
    "不能背对画面",
    "不能画墙、地台、阴影、光晕、背景、文字、签名、水印",
    "头顶、手脚不能被画面边缘切掉",
];

/** 一次复制两份探头提示词时的分隔行 */
export const PROMPT_SEPARATOR = "\n\n---\n\n";

const negativeBlock = (items: readonly string[]): string[] => ["", "负面清单 (出现任一条即重新生成):", ...items.map((x) => `- ${x}`)];

const nameWithOriginal = (name: string, original?: string): string =>
    original && original.trim() && original !== name ? `${name} (${original})` : name;

const urlLines = (label: string, main: string, fallback?: string, note?: string): string[] => {
    const lines = [`${label}${note ? ` (${note})` : ""}: ${main}`];
    if (fallback && fallback !== main) lines.push(`${label}备用地址 (Bangumi 原图): ${fallback}`);
    return lines;
};

const referenceBlock = (input: CharacterPromptInput, withCover: boolean): string[] => [
    `番剧名: ${nameWithOriginal(input.animeName, input.animeOriginalName)}`,
    `角色名: ${nameWithOriginal(input.characterName, input.characterOriginalName)}${input.relation ? ` (${input.relation})` : ""}`,
    ...urlLines("角色图片", input.characterImageUrl, input.characterImageFallbackUrl),
    ...(withCover ? urlLines("番剧封面", input.coverUrl, input.coverFallbackUrl, "参考图, 用于画风与配色") : []),
];

/**
 * 单个朝向的探头立绘提示词. 必含: 番剧封面 URL、番剧名、角色名、角色图片 URL、姿势描述.
 * .agents/notes/implemented/feature/2026-11-03-anime-character-context-menu.md
 */
export function buildPeekPrompt (input: CharacterPromptInput, side: PeekSide): string {
    const s = SIDE_TEXT[side];
    return [
        `【探头立绘 · ${side === "left" ? "左侧" : "右侧"}】`,
        ...referenceBlock(input, true),
        "",
        `请画一张二次元动画风格的角色立绘: 「${input.characterName}」, 来自番剧「${input.animeName}」.`,
        "外貌、发型、发色、瞳色、服装严格以「角色图片」为准; 画风、线条和配色参考「番剧封面」.",
        `姿势: ${PEEK_POSE}.`,
        `构图: 二次元角色从上往下俯探身子, 半身出画, ${s.place}, ${s.face}; ${s.crop}; ${s.outer}.`,
        "竖版画幅 (约 2:3), 透明背景 (RGBA PNG), 无背景、无场景、无文字、无水印、无边框, 角色以外的像素全透明, 轮廓干净不带白边或绿边.",
        "表情生动、带一点好奇, 高清, 细节清晰.",
        ...negativeBlock(PEEK_NEGATIVE),
    ].join("\n");
}

/** 通用的纯角色立绘: 透明底、无背景, 不涉及探头构图. 网站模式也提供 */
export function buildCharacterPrompt (input: CharacterPromptInput): string {
    return [
        "【角色独立立绘】",
        ...referenceBlock(input, false),
        "",
        `请画一张二次元动画风格的角色全身立绘: 「${input.characterName}」, 来自番剧「${input.animeName}」.`,
        "外貌、发型、发色、瞳色、服装严格以「角色图片」为准.",
        "自然站姿, 正面或微侧身, 完整入画.",
        "透明背景 (RGBA PNG), 只有角色本身, 角色以外的像素全透明, 轮廓干净不带白边.",
        "高清, 细节清晰.",
        ...negativeBlock(CHARACTER_NEGATIVE),
    ].join("\n");
}

/** 三份一起出: 探头左 / 探头右 / 纯角色 */
export function buildAllPrompts (input: CharacterPromptInput): { left: string; right: string; character: string } {
    return {
        left: buildPeekPrompt(input, "left"),
        right: buildPeekPrompt(input, "right"),
        character: buildCharacterPrompt(input),
    };
}
