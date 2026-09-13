/**
 * HXLoLi 站点 Cloudflare Workers 入口 (静态资源模式)
 *
 * 部署配置见同目录 wrangler.toml:
 *   [assets]
 *   directory = "./build"
 *   binding   = "ASSETS"   <-- 必须存在, 否则 env.ASSETS 为 undefined
 *
 * 踩坑记录 (2026-09-13): wrangler.toml 只写 directory 不写 binding 时, 部署本身
 * 不会报错, 但只要 Worker 脚本被执行到 env.ASSETS.fetch 就会抛
 * "TypeError: Cannot read properties of undefined (reading 'fetch')",
 * 线上表现为 Cloudflare Error 1101 (Worker threw exception)。
 * 命中静态文件的请求由 Cloudflare 资源路由直接处理(不跑脚本), 所以只有
 * 目录页 / 404 兜底 / /search-index.json 这类需要脚本兜底的 URL 会 1101。
 */

const SEARCH_INDEX_PATTERN = /^\/search-index(-[a-zA-Z0-9]+)?\.json$/;

function missingAssetsBinding() {
  return new Response(
    'ASSETS binding is not configured for this Worker. ' +
      'Add "binding = \"ASSETS\"" to the [assets] section of wrangler.toml.',
    { status: 500, headers: { 'Content-Type': 'text/plain; charset=utf-8' } },
  );
}

export default {
  async fetch(request, env) {
    // 显式取绑定, 缺失时给出可读报错, 而不是抛出难以定位的 TypeError
    const assets = env.ASSETS;
    if (!assets) return missingAssetsBinding();

    const url = new URL(request.url);

    // 拦截 search-index.json 请求, 从 chunk 文件流式合并返回
    // (CI 里 search-index.json 超过 25MiB 会被拆成 search-index-chunk-*.txt)
    if (SEARCH_INDEX_PATTERN.test(url.pathname)) {
      const manifestUrl = new URL('/search-index-manifest.json', url.origin);
      const manifestResp = await assets.fetch(manifestUrl);
      if (!manifestResp.ok) {
        // 没有 manifest 说明索引没被拆分, 直接回源
        return assets.fetch(request);
      }
      const manifest = await manifestResp.json();

      // 预先获取所有 chunk 的 Response 对象
      const chunkResponses = [];
      for (const chunk of manifest.chunks) {
        const chunkUrl = new URL('/' + chunk, url.origin);
        const chunkResp = await assets.fetch(chunkUrl);
        if (!chunkResp.ok) {
          return new Response('Search index chunk not found: ' + chunk, { status: 500 });
        }
        chunkResponses.push(chunkResp);
      }

      // 使用 ReadableStream 流式拼接, 避免在内存中合并大字符串
      const { readable, writable } = new TransformStream();
      (async () => {
        const writer = writable.getWriter();
        try {
          for (const resp of chunkResponses) {
            const reader = resp.body.getReader();
            while (true) {
              const { done, value } = await reader.read();
              if (done) break;
              await writer.write(value);
            }
          }
        } finally {
          await writer.close();
        }
      })();

      return new Response(readable, {
        headers: {
          'Content-Type': 'application/json',
          'Cache-Control': 'public, max-age=604800, immutable',
        },
      });
    }

    const resp = await assets.fetch(request);

    if (resp.status === 404) {
      // GIF → WebP 回退: 超大 GIF 在构建时被转为 WebP
      if (url.pathname.endsWith('.gif')) {
        const webpUrl = new URL(url.pathname.replace(/\.gif$/i, '.webp'), url.origin);
        const webpResp = await assets.fetch(new Request(webpUrl, request));
        if (webpResp.ok) return webpResp;
      }

      // 404 兜底: 回落到 Docusaurus 生成的 404.html
      const notFound = await assets.fetch(new URL('/404.html', url.origin));
      if (notFound.ok) {
        return new Response(notFound.body, {
          status: 404,
          headers: notFound.headers,
        });
      }
    }

    return resp;
  },
};
