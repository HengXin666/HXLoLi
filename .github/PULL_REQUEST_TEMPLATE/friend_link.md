---
name: 🔗 申请友链 / Add Friend Link
about: 添加或修改友链信息
title: "🔗 友链申请: [你的站点名称]"
labels: ["friend-link"]
---

## 📋 友链信息

> 请确保你已在 `data/friendLinks.ts` 的 `friendLinks` 数组 **末尾** 添加了一条记录, 格式如下:

```ts
{
  name: "站点名称",
  owner: "站长名称 (自定义, 随意填写)",
  url: "https://你的站点链接",
  github: "https://github.com/你的GitHub用户名",
  avatar: "头像链接 (建议使用 GitHub 头像)",
  description: "一句话简介 (不超过 50 字)",
}
```

## ✅ 自检清单

- [ ] 我只修改了 `data/friendLinks.ts` 文件
- [ ] `github` 字段链接与我的 GitHub 账号一致 (CI 自动校验)
- [ ] 所有链接均可正常访问 (HTTP 200)
- [ ] 站点描述不超过 50 字
- [ ] 如果是修改友链, 我只修改了自己之前提交的记录

## 📝 说明

> [!TIP]
> 此处可以留言, 您可以描述你的来意、站点内容等。

## 警告

> [!WARNING]
> - 请不要提交重复的友链。
> - 如果您是修改友链, 请确保您只修改了自己之前提交的记录。
> - 请不要上传恶意网址或广告。 (你会被列入黑名单!)
