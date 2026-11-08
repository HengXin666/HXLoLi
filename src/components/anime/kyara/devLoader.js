/*
 * kyara 的 dev 专属模块加载器, 与 src/components/home/lazyChunks.js 的 loadAnimeOverlayEditor 同一做法:
 * 条件必须是 process.env.NODE_ENV 常量比较, webpack 把它替换成字面量后由 ConstPlugin 判死分支, 死分支里的 import() 不会被解析,
 * 生产构建不产生 dev/ 这个 chunk, 写入地址 (/__hx/kyara/) 因此不进任何产物. 不要改成运行时变量或函数里再判断.
 * 写成 .js 是因为 tsconfig 的 nodenext 不认无扩展名的动态 import; 类型见同名 .d.ts.
 */
export const loadKyaraDev = process.env.NODE_ENV === 'development'
    ? () => import('./dev/index')
    : null;
