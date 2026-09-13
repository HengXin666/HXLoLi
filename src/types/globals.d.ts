/**
 * 静态资源与第三方模块的类型声明.
 *
 * 命名注意: 本文件**不能**叫 `hxdeck.d.ts` —— 那样会与 `src/hxdeck/` 目录同名,
 * TS 会优先解析 .d.ts, 导致 `import ... from '@site/src/hxdeck'` 全部指向声明文件,
 * 报出莫名其妙的类型错误 (例如把 ReactNode 退化成 string 交集).
 */

declare module '*.webp' {
    const src: string;
    export default src;
}
declare module '*.png' {
    const src: string;
    export default src;
}
declare module '*.jpg' {
    const src: string;
    export default src;
}
declare module '*.svg' {
    const src: string;
    export default src;
}

declare module 'js-yaml' {
    export function load(input: string): unknown;
    export function dump(input: unknown, options?: Record<string, unknown>): string;
    const _default: { load: typeof load; dump: typeof dump };
    export default _default;
}
