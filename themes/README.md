# 主题目录

把主题文件 (`.yaml` / `.yml` / `.json`) 放在这里, 站点就会自动识别.

- 文件名不重要, 主题的 `id` 才是标识
- 放进来后运行 `node .agents/skills/hx-archify/scripts/scan-themes.mjs HXLoLi/static/themes`
  (仓库根目录执行) 刷新清单 `index.json`
- 主题之间可以互相继承: 复制一份改 `id` / `name` 与少量字段即可
- 前端"主题切换器"会列出 `index.json` 里的全部主题, 读者可自行切换

主题编辑器 (`/hxdeck-editor`) 保存时导出的文件直接放到这个目录即可.
