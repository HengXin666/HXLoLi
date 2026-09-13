/**
 * hxdeck —— HXLoLi 演示页 UI 系统 (独立层)
 *
 * 边界: 自成一体的 UI 系统; 允许依赖 node_modules, 不引用博客自身组件/样式,
 * 也不被博客原有代码引用 (删掉本目录不影响博客).
 *
 * 分层:
 *   theme/   端口层  —— token 契约 + 主题实现, 换主题不改控件
 *   Deck     运行时  —— 整屏纵向位移 / 导航 / 键盘 / 滚轮 / 触屏
 *   ui       控件层  —— 版式原语 + 内容块 + 图片容器
 *   charts   数据层  —— 饼/折线/柱/树/指标 (recharts)
 *   code     代码层  —— VS Code 配色静态高亮
 *   meme     皮肤层  —— 吉祥物/纹样等主题素材槽位
 */

/* 外挂嵌入 (drawio / html / image) */
export { Embed, inferKind } from './embed';
export type { EmbedProps, EmbedKind } from './embed';

/* 博客内嵌与语法 */
export { PptEmbed } from './PptEmbed';
export { PptDeckFromCode } from './PptDeckFromCode';
export { registerDeck, getDeck, listDecks } from './decks';
// 副作用导入: 注册内置演示页
import './decks-builtin';
export type { DeckEntry } from './decks';
export type { PptEmbedProps } from './PptEmbed';
export { parsePptText, parsePptArgs } from './syntax';
export { usePageParam } from './usePageParam';
export type { PptDirective } from './syntax';

/* 主题注册与切换 */
export { ThemeSwitch } from './ThemeSwitch';
export { ThemePicker } from './ThemePicker';
export { listThemes, getTheme, loadTheme, loadManifest, loadThemeFile, registerTheme } from './theme/registry';
export type { ThemeInfo } from './theme/registry';

/* 运行时 */
export { Deck, default } from './Deck';
export type { DeckProps } from './Deck';
export { Slide } from './Slide';
export type { SlideProps } from './Slide';
export { Nav } from './Nav';
export { Brand } from './brand';

/* 版式原语 */
export { Rise, Cover, Head, Card } from './blocks';

/* 控件层 v2 */
export {
    PageHeader,
    Split,
    Columns,
    Bullets,
    Callout,
    Quote,
    QuoteBlock,
    Steps,
    Badge,
    Divider,
    KeyValues,
    Figure,
} from './ui';

/* 图 (架构/流程/时序/数据流/状态) —— 内联 archify 产物并跟随主题 */
export { Diagram, DiagramWithNotes, DiagramGrid, DiagramPlaceholder, extractSvg, diagramVars, autoZoom, minFontSize, resolveAsset } from './diagram';
export type { DiagramProps, DiagramAsset, DiagramKind } from './diagram';

/* 控件补充集 */
export { Timeline, Meter, Gauge, CompareTable, FullBleed, CodeDiff, parseDiff } from './ui-more';
export type { DiffLine } from './ui-more';

/* 数据层 */
export { PieChartBlock, LineChartBlock, BarChartBlock, Tree, Stat } from './charts';

/* 代码层 */
export { CodeBlock, Code } from './code';
export type { CodeBlockProps } from './code';

/* 皮肤层 */
export { Mascot, SlotMascot, Meme } from './meme';

/* 主题编辑器与序列化 */
export { default as ThemeEditor } from './ThemeEditor';
export { serializeTheme, parseTheme, validateTheme, mergeTheme, downloadTheme, readThemeFile, blankTheme } from './theme/serialize';
export type { ThemeFormat } from './theme/serialize';

/* 主题 */
export { hxloliTheme } from './theme/hxloli';
export { whaleTheme } from './theme/whale';
export { useDeckTheme, seriesColors } from './theme/context';
export * from './theme/types';

/* 资源 */
export { assetUrl, assetMap, normalizeSlot, slotVars } from './assets';
export type { SkinSlot, DeckSkin, EmojiKey } from './assets';