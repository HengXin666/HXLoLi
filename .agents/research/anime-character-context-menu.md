# 番剧详情: 角色立绘右键菜单 (复制提示词)

## 目录

1. 数据来源与本地模式判定
2. 菜单项
3. 两种模式的差异
4. 提示词模板与成品示例
5. 键盘与移动端交互
6. 视觉与主题
7. 改动文件
8. 验证

## 1. 数据来源与本地模式判定

数据链路 (只读, 本功能不新增请求):

- 记录来自 `src/utils/anime/animeStore.ts` 的 `useAnimeRecordsState(baseUrl)`: 从 HXLoLi-ANiMe 仓库的 CDN 拉 `ANiMeRecord.json`, 再盖上本地 overlay. 详情页 `src/pages/anime/details.tsx` 按 `?id=` 取 `record.anime_data` (类型 `ANiMeData`, 见 `src/utils/anime/types.ts`).
- 番剧名: `anime_data.name_cn || anime_data.name`, 原名 `anime_data.name`.
- 番剧封面: 页面显示用 `toAnimeCdnUrl(\`/data/anime/${id}.jpg\`)` (CDN 引擎选中的节点; 引擎没初始化时回落 `https://cdn.jsdmirror.com/gh/<user>/HXLoLi-ANiMe@<tag>/data/anime/<id>.jpg`). Bangumi 原图在 `anime_data.image_url`, 作备用参考图.
- 角色列表: `anime_data.characters: Character[]`, 字段 `id / name / name_cn / relation (主角/配角/客串) / image_url / actor_ids / summary / ...`. 页面里 `Characters` 按主角 -> 配角 -> 客串排序, 默认显示 12 个.
- 角色图: 页面显示用 `toAnimeCdnUrl(\`/data/kyara/${char.id}.jpg\`)`, 作为 `<a class=charImg>` 的 background-image (不是 `<img>`, 所以浏览器原生右键菜单里没有"另存图片"). Bangumi 原图在 `char.image_url`.

本地模式判定 (沿用仓里已有的两套, 不新增):

- `src/utils/cdn/linkJsDelivr.ts` 导出的是 `isLocalDev()` (hostname 为 `localhost` 或 `127.0.0.1`), 仓里没有名为 `isLocalHost` 的函数, 任务里说的就是它.
- `src/components/DevTools/useDevService.ts` 的 `useDevService()`: 挂载后 fetch `http://localhost:3310/health`, 1.5s 超时, `{ ok: true }` 时 `ready = true` (dev-edit-server 在跑).
- 本功能: `本地模式 = isLocalDev() || useDevService().ready`. 两者任一成立就显示开发者菜单项. `isLocalDev()` 在挂载后的 effect 里读 (SSR 与首帧一律按网站模式), 菜单只在交互时出现, 不会有水合不一致.
- 探测只在 `Characters` 这一层调用一次 (一个页面一个 probe), 不是每张角色卡一次. 生产站上 MitemitePanel / InlineEditor 本来就在调同一个 probe, 本功能没有增加新的对外暴露面.

## 2. 菜单项

在番剧详情页 "角色" 一节的角色立绘上右键 (或触屏长按、键盘菜单键 / Shift+F10) 弹出自绘菜单. 菜单顶部一行是角色名, 本地模式在右侧多一个 "本地" 小徽章.

| 菜单项 | 本地模式 | 网站模式 | 点了之后 |
| --- | --- | --- | --- |
| 复制探头立绘提示词 (左右两份) | 有 (带粉点标记) | 无 | 调 `buildAllPrompts(input)`, 把 `left` + 分隔行 `---` + `right` 一次写进剪贴板 |
| 复制角色独立立绘提示词 (透明底) | 有 | 有 | 调 `buildCharacterPrompt(input)`, 写进剪贴板 |
| 下载角色图 | 有 | 有 | fetch 站内 CDN 角色图成 blob, 存成 `<角色名>.jpg`; 失败时新开标签页打开该图, 用户可直接另存 |
| 复制角色图片链接 | 有 | 有 | 把站内 CDN 角色图 URL 写进剪贴板 |
| 打开 Bangumi 原图 (新标签页) | 有 | 有 | 新标签页打开 `char.image_url` (Bangumi 原始大图); 该角色没有原图地址时这一项不显示 |

开发者项与其余项之间有一条分隔线. 每次操作后底部出一个 1.8s 的轻提示 (toast, `role=status`), 例如 "已复制探头立绘提示词 (左右两份)", 剪贴板被拒时提示 "复制失败: 浏览器拒绝了剪贴板权限". 全程没有原生 alert.

## 3. 两种模式的差异

探头立绘提示词是站长给页面两侧探头布局生图用的开发者工具: 它带着构图约束、负面清单和站内 CDN 地址, 对访客没有意义, 放在公开站点上只会让菜单变长、让人困惑, 所以只在本地模式 (`isLocalDev()` 为真, 即在 localhost / 127.0.0.1 打开站点, 或 dev-edit-server 的 `/health` 探测通过) 才出现. 网站模式 (生产静态站) 给访客的是"拿到这个角色"所需的一切: 一键复制通用的纯角色透明底立绘提示词, 直接下载角色图 (文件名是角色名), 复制图片链接, 或打开 Bangumi 原图. 判定沿用仓里已有的两套, 不新增第三种; 两者都在挂载后才算, SSR 与首帧一律是网站模式, 菜单只在交互时出现, 不会水合不一致. 过滤是运行时的: 生产产物里仍有 "复制探头立绘提示词" 的文案, 但它不含任何写入通道, 不需要像本人标注编辑器那样做 NODE_ENV 死分支裁切.

## 4. 提示词模板与成品示例

模板只在 `src/utils/anime/characterPrompt.ts` 一处, 组件里没有任何提示词文案. 导出:

- `buildPeekPrompt(input, side: "left" | "right")`: 单个朝向的探头立绘提示词. 正文逐行写出番剧名、角色名、角色图片 URL、番剧封面 URL (参考图) 与姿势描述, 再写构图约束 (从上往下俯探身子、半身出画、身体朝内、竖直裁切线在内侧) 与负面清单.
- `buildCharacterPrompt(input)`: 纯角色透明底立绘, 不含探头构图.
- `buildAllPrompts(input)`: 返回 `{ left, right, character }`.
- 常量 `PEEK_POSE` (姿势描述)、`PEEK_NEGATIVE` (抄自 `catgirl-genprompt.md` 第 6 节的 5 条)、`CHARACTER_NEGATIVE`、`PROMPT_SEPARATOR`.

方向约定: 左侧 = 放在页面左侧、身体与脸朝右、竖直裁切线在画面右边缘; 右侧 = 放在页面右侧、身体与脸朝左、竖直裁切线在画面左边缘. 两侧都是"朝内、裁切线在内侧".

成品示例 (真实数据, 取自 HXLoLi-ANiMe 的 `ANiMeRecord.json`, CDN tag `HX-20261102180459-V1HA7J`): 番剧 Fate/strange Fake (bgm subject 443831), 角色 吉尔伽美什 (bgm character 1227, 主角). 文中 4 个图片地址本轮都用 curl 验过返回 200 image/jpeg, jsDelivr 镜像带 `access-control-allow-origin: *`. 以下三段是 `node src/components/anime/CharacterMenu/characterPrompt.check.mjs --print` 的原样输出, 可直接复制去生图.

### 4.1 buildPeekPrompt(input, "left")

```text
【探头立绘 · 左侧】
番剧名: Fate/strange Fake
角色名: 吉尔伽美什 (ギルガメッシュ) (主角)
角色图片: https://cdn.jsdmirror.com/gh/HengXin666/HXLoLi-ANiMe@HX-20261102180459-V1HA7J/data/kyara/1227.jpg
角色图片备用地址 (Bangumi 原图): https://lain.bgm.tv/pic/crt/l/db/ca/1227_crt_Dq9kZ.jpg?r=1498989532
番剧封面 (参考图, 用于画风与配色): https://cdn.jsdmirror.com/gh/HengXin666/HXLoLi-ANiMe@HX-20261102180459-V1HA7J/data/anime/443831.jpg
番剧封面备用地址 (Bangumi 原图): https://lain.bgm.tv/pic/cover/l/f0/51/443831_CNG0F.jpg

请画一张二次元动画风格的角色立绘: 「吉尔伽美什」, 来自番剧「Fate/strange Fake」.
外貌、发型、发色、瞳色、服装严格以「角色图片」为准; 画风、线条和配色参考「番剧封面」.
姿势: 从画面上方往下俯探身子, 上半身探出、下半身留在画外 (半身出画), 头和肩先探出来, 双手收在胸前或轻搭在上边缘内侧 (不碰竖直裁切线), 视线朝画面中央往下看; 镜头是仰角 (从下往上看), 角色俯身、头发和衣摆自然下垂.
构图: 二次元角色从上往下俯探身子, 半身出画, 放在页面左侧, 身体与脸朝右 (朝向页面内侧); 竖直裁切线在画面右边缘 (内侧), 角色被这条直线切开, 像是从正文卡片的左边缘后面探出来; 画面左边缘与上边缘留透明空白, 不让肢体碰到外侧边.
竖版画幅 (约 2:3), 透明背景 (RGBA PNG), 无背景、无场景、无文字、无水印、无边框, 角色以外的像素全透明, 轮廓干净不带白边或绿边.
表情生动、带一点好奇, 高清, 细节清晰.

负面清单 (出现任一条即重新生成):
- 出画处 / 竖直裁切线处不能是手臂、手、手指, 只能是躯干、肩、头发
- 不能背对正文 (身体朝外), 也不能是"站在墙边"露出整个上半身
- 不能画墙、地台、阴影、光晕、背景、文字、签名、水印
- 左右两份必须分别生成, 不能把同一张图左右镜像翻转凑成一对
- 头顶不能被上边缘切掉, 头上方要留出空白
```

### 4.2 buildPeekPrompt(input, "right")

```text
【探头立绘 · 右侧】
番剧名: Fate/strange Fake
角色名: 吉尔伽美什 (ギルガメッシュ) (主角)
角色图片: https://cdn.jsdmirror.com/gh/HengXin666/HXLoLi-ANiMe@HX-20261102180459-V1HA7J/data/kyara/1227.jpg
角色图片备用地址 (Bangumi 原图): https://lain.bgm.tv/pic/crt/l/db/ca/1227_crt_Dq9kZ.jpg?r=1498989532
番剧封面 (参考图, 用于画风与配色): https://cdn.jsdmirror.com/gh/HengXin666/HXLoLi-ANiMe@HX-20261102180459-V1HA7J/data/anime/443831.jpg
番剧封面备用地址 (Bangumi 原图): https://lain.bgm.tv/pic/cover/l/f0/51/443831_CNG0F.jpg

请画一张二次元动画风格的角色立绘: 「吉尔伽美什」, 来自番剧「Fate/strange Fake」.
外貌、发型、发色、瞳色、服装严格以「角色图片」为准; 画风、线条和配色参考「番剧封面」.
姿势: 从画面上方往下俯探身子, 上半身探出、下半身留在画外 (半身出画), 头和肩先探出来, 双手收在胸前或轻搭在上边缘内侧 (不碰竖直裁切线), 视线朝画面中央往下看; 镜头是仰角 (从下往上看), 角色俯身、头发和衣摆自然下垂.
构图: 二次元角色从上往下俯探身子, 半身出画, 放在页面右侧, 身体与脸朝左 (朝向页面内侧); 竖直裁切线在画面左边缘 (内侧), 角色被这条直线切开, 像是从正文卡片的右边缘后面探出来; 画面右边缘与上边缘留透明空白, 不让肢体碰到外侧边.
竖版画幅 (约 2:3), 透明背景 (RGBA PNG), 无背景、无场景、无文字、无水印、无边框, 角色以外的像素全透明, 轮廓干净不带白边或绿边.
表情生动、带一点好奇, 高清, 细节清晰.

负面清单 (出现任一条即重新生成):
- 出画处 / 竖直裁切线处不能是手臂、手、手指, 只能是躯干、肩、头发
- 不能背对正文 (身体朝外), 也不能是"站在墙边"露出整个上半身
- 不能画墙、地台、阴影、光晕、背景、文字、签名、水印
- 左右两份必须分别生成, 不能把同一张图左右镜像翻转凑成一对
- 头顶不能被上边缘切掉, 头上方要留出空白
```

### 4.3 buildCharacterPrompt(input)

```text
【角色独立立绘】
番剧名: Fate/strange Fake
角色名: 吉尔伽美什 (ギルガメッシュ) (主角)
角色图片: https://cdn.jsdmirror.com/gh/HengXin666/HXLoLi-ANiMe@HX-20261102180459-V1HA7J/data/kyara/1227.jpg
角色图片备用地址 (Bangumi 原图): https://lain.bgm.tv/pic/crt/l/db/ca/1227_crt_Dq9kZ.jpg?r=1498989532

请画一张二次元动画风格的角色全身立绘: 「吉尔伽美什」, 来自番剧「Fate/strange Fake」.
外貌、发型、发色、瞳色、服装严格以「角色图片」为准.
自然站姿, 正面或微侧身, 完整入画.
透明背景 (RGBA PNG), 只有角色本身, 角色以外的像素全透明, 轮廓干净不带白边.
高清, 细节清晰.

负面清单 (出现任一条即重新生成):
- 不能背对画面
- 不能画墙、地台、阴影、光晕、背景、文字、签名、水印
- 头顶、手脚不能被画面边缘切掉
```

## 5. 键盘与移动端交互

打开方式:

- 鼠标右键: 菜单出现在指针位置下方 (FloatingLayer 下方优先, 放不下翻到上方, 左右夹在视口内 8px).
- 键盘: Tab 聚焦到角色图 (它本来就是链接), 按菜单键或 Shift+F10. 这时 contextmenu 事件坐标为 0, 菜单改为贴着角色图下方.
- 触屏长按: 按住 500ms 触发 (`LONG_PRESS_MS`). 期间手指移动超过 10px (`LONG_PRESS_SLOP`, 视为滚动)、抬起或 pointercancel 都取消. 触发后松手的那次 click 会被拦下 (capture 阶段 preventDefault + stopPropagation), 不会跳去 bgm.tv, Tooltip 的轻点切换也不会被触发. Android Chrome 长按时自己也会发 contextmenu, 两条路径谁先到都只开一次菜单.
- iOS: Safari 长按不发 contextmenu, 由上面的 500ms 计时器接管; 角色图加了 `-webkit-touch-callout: none` 与 `user-select: none`, 系统的链接预览 / 存图菜单不会弹出来盖住自绘菜单.

菜单打开后 (WAI-ARIA menu 模式, 容器 `role=menu`, 每项 `role=menuitem` 的 button):

| 键 | 行为 |
| --- | --- |
| 打开时 | 焦点自动进第一项 |
| ArrowDown / ArrowUp | 下一项 / 上一项, 首尾循环 |
| Home / End | 第一项 / 最后一项 |
| Enter / 空格 | 执行当前项 (原生 button 行为), 执行后菜单关闭, 焦点回到角色图 |
| Esc | 关闭, 焦点回到角色图; 事件不再冒泡 |
| Tab / Shift+Tab | 关闭, 焦点回到角色图 (不让焦点跳到 body 末尾的弹层之外) |

其他关闭条件: 在菜单外按下指针 (pointerdown 捕获阶段判断)、页面滚动 (菜单内部滚动除外)、窗口尺寸变化. 在菜单上再右键不会弹出浏览器原生菜单. 右键另一张角色图时, 旧菜单在 pointerdown 时关闭, 新菜单随即打开.

## 6. 视觉与主题

- 对齐对象: 弹层外壳直接复用 `src/components/common/Tooltip` 导出的 `FloatingLayer` 与 `Tooltip.module.css` (fixed 定位、140ms 淡入、`prefers-reduced-motion` 下不动画); 选项的尺寸与高亮照 `src/components/common/Tooltip/Select.module.css` 的下拉列表 (13px 字、`7px 10px` 内边距、圆角 7px、当前项站点粉 `rgba(255,136,255,.14)` 底). 番剧页筛选的下拉就是这套, 菜单看起来是同一家族.
- 深色主题: 卡片 `rgba(24,24,27,.96)` + 1px `rgba(255,255,255,.12)` 边 + 圆角 10px + `0 10px 30px rgba(0,0,0,.45)` 阴影 + 8px 背景模糊, 选项字 `#e4e4e7`, 分隔线 `rgba(255,255,255,.1)`.
- 亮色主题 (`html[data-theme='light']`): 卡片换成 `rgba(255,255,255,.98)` + 1px `rgba(0,0,0,.1)` 边 + `0 10px 30px rgba(0,0,0,.16)` 阴影, 选项字 `#1c1e21`, 高亮粉 16%, "本地" 徽章文字压暗为 `#8c4b8c` (与详情页 note 里主题色文字的取值一致, 对白底 6.0:1).
- 开发者项前有一个 6px 主题色圆点, 一眼区分; 菜单头一行 11.5px 半透明角色名.
- 轻提示: 底部居中, 与菜单同一套深 / 浅卡片, 160ms 上浮淡入, `pointer-events: none` 不挡点击.
- 菜单宽 256px, z-index 10000 (与 Select 弹层一致, 压过 Tooltip 的 9999), 内容层 padding 4px.

## 7. 改动文件

- `src/utils/anime/characterPrompt.ts` (新建): 提示词模板的唯一来源, 纯函数、零 import, 导出三份拼装函数与常量.
- `src/components/anime/CharacterMenu/CharacterContextMenu.tsx` (新建): `CharacterMenuScope` (菜单实例、本地模式判定、键盘、关闭逻辑、复制 / 下载、toast) 与 `useCharacterMenuTrigger` (右键、长按、click 拦截).
- `src/components/anime/CharacterMenu/CharacterContextMenu.module.css` (新建): 菜单、选项、亮色主题覆盖、toast、iOS callout 关闭.
- `src/components/anime/CharacterMenu/characterPrompt.check.mjs` (新建): 可机械运行的校验脚本.
- `src/pages/anime/details.tsx` (修改): import 菜单; `CharacterCard` 算出角色图 URL 并把 trigger 挂到角色图 `<a>` 上 (className 合并, 事件经 Tooltip 的 cloneElement 链式保留); `Characters` 多收一个 `anime` 参数并用 `CharacterMenuScope` 包住角色网格; 页面组件组装 `menuAnime` (番剧名、原名、封面 CDN URL、Bangumi 封面原图).
- `.agents/notes/implemented/feature/2026-11-03-anime-character-context-menu.md` (新建): Agent Notes 决策记录, `## Code` 列 3 个代表文件, 三处代码在声明上方反向引用它. 它不在任务的写区清单里, 但仓里红线要求代码与 note 成对, 所以补上, 请人工确认.
- 本文档.

没有碰: `static/img/`、`src/components/BlogWithCats/`、`src/theme/Root.tsx`、`common/Tooltip` 本身、`AnimeDetailPage.module.css`.

## 8. 验证

### 8.1 提示词校验脚本

在 HXLoLi 目录运行:

```bash
node src/components/anime/CharacterMenu/characterPrompt.check.mjs          # 逐条断言, 退出码 0 通过 / 1 失败
node src/components/anime/CharacterMenu/characterPrompt.check.mjs --print  # 额外打印输入与三份提示词全文 (即第 4 节)
```

它用仓库 node_modules 里的 typescript 把 `characterPrompt.ts` 转译成 JS 后执行, 不启动站点、不联网. 断言覆盖: 左右两份都含 番剧封面 URL / 番剧名 / 角色名 / 角色图片 URL / 姿势描述 五项; 都含 俯探身子 / 半身出画 / 朝向页面内侧 / 竖直裁切线 / (内侧) 与透明背景; 左朝右且裁切线在右边缘、右朝左且裁切线在左边缘; `buildAllPrompts` 三个字段与单函数输出一致; 5 条负面清单在两份里都完整出现; 纯角色那份含角色名与图片 URL、透明底、且不含 俯探 / 裁切线 / 半身出画; 原名与译名相同时不重复, 没有备用地址时不输出备用行.

本轮在沙箱 (Node v22.23.3, typescript 5) 跑过, 结果 22 项全部通过:

```text
ok   left: 含番剧封面 URL
ok   left: 含番剧名
ok   left: 含角色名
ok   left: 含角色图片 URL
ok   left: 含姿势描述
ok   left: 构图约束 俯探 / 半身出画 / 朝内 / 竖直裁切线在内侧
ok   left: 透明背景
ok   right: 含番剧封面 URL
ok   right: 含番剧名
ok   right: 含角色名
ok   right: 含角色图片 URL
ok   right: 含姿势描述
ok   right: 构图约束 俯探 / 半身出画 / 朝内 / 竖直裁切线在内侧
ok   right: 透明背景
ok   左: 身体朝右, 裁切线在右边缘
ok   右: 身体朝左, 裁切线在左边缘
ok   buildAllPrompts 返回 left / right / character
ok   探头两份都带完整负面清单
ok   独立立绘: 含角色名与角色图片 URL
ok   独立立绘: 透明底 / 无背景
ok   独立立绘: 不涉及探头构图
ok   原名与译名相同时不重复, 没有备用地址时不输出备用行
```

### 8.2 类型检查

- 沙箱里用与仓库相同的 tsconfig (`module` / `moduleResolution` = nodenext, `strict`, `jsx: react`, `@site/*` 路径) 对 `characterPrompt.ts`、`CharacterContextMenu.tsx` 与真实的 `common/Tooltip/index.tsx`、`DevTools/useDevService.ts` 跑 `tsc --noEmit`: 0 错误 (`linkJsDelivr` 用只含 `isLocalDev` 签名的桩, css module 用通配声明).
- `details.tsx` 的改动只有新 import、一个 hook 调用、一处 props 展开和一个新参数, 未在沙箱完整检查 (它依赖大量 Docusaurus 模块). 本机全仓 `npx tsc --noEmit` 相对基线 (`scripts/quality-baseline.json`) 是否新增错误, 以本机输出为准: 已给出命令, 把结果写进 `.agents/research/tsc-after-char-menu.txt`, 改动文件里不应出现任何报错.

### 8.3 Agent Notes 红线

`uv run scripts/redlines/agent_notes.py --diff` 需要在本机跑 (沙箱只能经 fs 隧道读写文件, 不能在本机执行命令). 预期: 新 note 与 3 个代表文件同 diff 配对; `details.tsx` 所在目录已有的 `anime-local-overlay` / `anime-detail-redesign` / `anime-detail-sidebar-and-hero` 三篇与 `components/anime` 目录已有的分享类 note 会因"同目录兄弟代码改动"报 review; `CharacterContextMenu.module.css` 报 resource-review. 结果文件约定为 `.agents/research/agent-notes-char-menu.txt` 与 `.json`.

### 8.4 手工点过的路径

沙箱没有能打开本机站点的浏览器, 按仓规也不能替你启动博客, 所以本轮没有在真实浏览器里点过菜单. 请本机启动后核对: ① localhost 打开任一番剧详情, 右键角色图出现 5 项且第一项带粉点; ② 复制探头提示词后粘贴, 应为第 4.1 + 4.2 两段以 `---` 分隔; ③ 方向键 / Home / End / Enter / Esc / Tab 行为如第 5 节表格; ④ 切亮色主题看菜单与 toast 是否白底; ⑤ 手机或 DevTools 触屏模拟长按 500ms 出菜单、拖动取消、松手不跳 bgm; ⑥ 生产构建 (或用局域网 IP 访问且不跑 dev-edit-server) 时第一项消失.

### 8.5 写文档时发现并修掉的问题

- 姿势描述原写 "一只手扶在裁切线边缘", 与负面清单第 1 条 (裁切线处不能是手) 自相矛盾; 改为 "双手收在胸前或轻搭在上边缘内侧 (不碰竖直裁切线)".
- 备用地址行原来是 "番剧封面 (参考图, 用于画风与配色) (备用, Bangumi 原图)", 两层括号读起来别扭; 改为 "番剧封面备用地址 (Bangumi 原图)".
- 校验脚本的示例输入原先是手写的占位 id 与地址, 换成了 `ANiMeRecord.json` 里的真实记录 (Fate/strange Fake · 吉尔伽美什).
