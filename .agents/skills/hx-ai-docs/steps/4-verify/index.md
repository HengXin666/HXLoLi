# 4 verify: 门禁和亲眼看

只做一件事: 证明笔记能编译, 能渲染, 图看得清. 不启动 dev server, 用用户已经开着的 `localhost:3000`

## 门禁

```bash
node scripts/check-kb-note.mjs '<笔记目录>'
uv run .agents/skills/hx-make-skill/scripts/prose_rules.py --check '<笔记目录>/index.md'
python3 .agents/skills/hx-note/scripts/cli/identity/hx_docs_id.py links
```

- check-kb-note 退出码必须是 0
- prose_rules 只允许多行 JSX 属性的 P4, 和 `> [!NOTE]`, 图片行这类误报
- links 必须报 `[OK]`

## 渲染

```bash
node .agents/skills/hx-ai-docs/scripts/render_check.ts '<页面 url>' /dev/shm/c/note dark
node .agents/skills/hx-ai-docs/scripts/render_check.ts '<页面 url>' /dev/shm/c/note light
```

脚本输出一行 JSON: 有没有错误边界, mermaid 报错, 控制台错误. 每张图和每个组件存成 `<前缀>-<n>.png`

- JSON 里 `errors` 必须为空. 例外: `<div> cannot be a descendant of <p>` 来自 `MDXComponents/Img` 对 Markdown 图片的包装, 是全站已知问题, 不算这篇的错
- 每张 PNG 都要打开看: 字没压线, 没出框, 亮暗两种模式都看得清

## 交付

- 给用户页面链接, 链接只用工具返回过的原样地址
- 说清这次删了什么, 改了什么, 哪里是示意
