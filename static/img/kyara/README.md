# kyara 角色配图目录约定

每个角色一个子目录, 目录名就是页面地址: `static/img/kyara/{name}/` → `/anime/kyara/{name}` (中日文名可以, 地址里会自动编码).

## 文件命名

```
NNN-标题@作者.ext
```

- `NNN`: 至少 3 位数字的序号, 页面按序号数值升序渲染 (序号相同再按文件名).
- `标题`: 随意, 可以含 `-` 和 `@`; 拆作者时只认**最后一个** `@`.
- `@作者`: 可省略, 省略时页面不显示作者.
- 扩展名: `png` / `jpg` / `jpeg` / `webp` / `gif` / `avif` (大小写不限).
- 正则: `^(\d{3,})-(.+?)(?:@([^@]+))?\.(png|jpe?g|webp|gif|avif)$`

| 文件名 | 序号 | 标题 | 作者 |
|---|---|---|---|
| `001-春日@ふーみ.png` | 1 | 春日 | ふーみ |
| `010-a-b@x@y.webp` | 10 | a-b@x | y |
| `002-无作者.jpg` | 2 | 无作者 | (无) |
| `abc.png` | 不合规, 跳过并在构建日志里 warn | | |

自动生成文件名时, `/ \ : * ? " < > |` 与 `@` 会被换成全角 (`／ ＼ ： ＊ ？ ＂ ＜ ＞ ｜ ＠`).

## 可选元数据 `kyara.json`

```json
{
  "displayName": "春日野穹",
  "work": "缘之空",
  "bangumiCharacterId": 12345,
  "accent": "#9ecbff",
  "cover": "003-雪@ふーみ.png"
}
```

全部可省. `displayName` 缺省用目录名; `cover` 写本目录里的文件名, 缺省取第一张; `accent` 是角色页的强调色.

## 导入

- **手动**: 按上面的规则把图片放进目录, dev 下会自动重扫.
- **命令行** (pixiv 只能走这里或 dev 写入通道, 因为原图要带 Referer):
  ```
  node scripts/kyara-import.mjs <name> <url...>
  node scripts/kyara-import.mjs --list <name>
  ```
  支持 `pixiv.net/artworks/{id}`、`pixiv.net/i/{id}`、`x.com|twitter.com/{user}/status/{id}` (经 api.fxtwitter.com 取原图) 和图片直链. 自动接在当前最大序号后面.
- **页面本地模式**: 角色页打开本地模式后可拖拽 / 粘贴图片、贴链接导入, 存在浏览器 IndexedDB (`hx-kyara`), 排在仓库图片之后; 可导出符合命名规则的 zip. 本机 `npm run start` 时还能一键写进本目录.

## dev 写入通道 (只在本机 dev server 上存在, 生产构建没有)

请求必须来自 localhost 并带 `X-HX-Kyara: 1` 头.

- `POST /__hx/kyara/{name}/import` `{ "url": "..." }`: 解析 → 下载 → 按下一个序号落盘
- `PUT /__hx/kyara/{name}/file/{filename}` 二进制体: 写入一个已合规命名的文件 (已存在回 409, `?overwrite=1` 覆盖)
- `POST /__hx/kyara/{name}/reorder` `{ "order": ["003-…", "001-…", …] }`: 按顺序重写序号, 标题和作者不变

运行时清单: `/kyara/index.json` 与 `/kyara/data/{name}.json` (build 时写进产物, dev 下现读现算).
