# VS Code
## 一、解决代码补全加载很慢的问题

- 在大工程、元模版情况下, VSCode经常抽风, 加载半天.

我们需要:

- 禁用文件监听，请按 `ctrl + shift + P` 打开命令面板，然后输入 `Preferences: Open Workspace Settings`。然后将以下配置复制到setting.json文件中

```json
"files.watcherExclude": {
  "**/.git/objects/**": true,
  "**/.git/subtree-cache/**": true,
  "**/node_modules/**": true,
  "**/bower_components/**": true,
  "**/vendor/**": true,
  "**/.sass-cache/**": true,
  "**/test/**": true,
  "**/tests/**": true,
  "**/doc/**": true,
  "**/coverage/**": true,
  "**/logs/**": true,
  "**/*.log": true
}
```

- 安装clangd

去商店搜索`clangd`, 然后安装即可.