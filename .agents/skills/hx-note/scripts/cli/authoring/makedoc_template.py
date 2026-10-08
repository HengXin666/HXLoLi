"""模板正文与 hxid 生成。"""
from __future__ import annotations

import re
import secrets
from pathlib import Path

from authoring.makedoc_meta import yaml_list, yaml_string


def new_hxid(existing: set[str] | None = None) -> str:
    """分配一个全局唯一的笔记 ID。

    hxid 是这篇笔记的持久身份: 目录被移动/改名后, 正文里指向它的
    `[标题](hxid:hx-xxxxxxxx)` 链接可以由 hx_docs_id.py resolve 重算回正确路径。
    创建时分配一次, 之后永不变更。
    """
    used = set(existing or ())
    while True:
        candidate = "hx-" + secrets.token_hex(4)
        if candidate not in used:
            return candidate

def read_existing_hxid(path: Path) -> str:
    """从已存在的文件头部读回 hxid。

    hxid 是笔记的持久身份, 一旦写下就不可变  所以 --force 覆盖时必须沿用旧值,
    绝不允许生成新的, 否则所有指向它的 [标题](hxid:...) 引用会一起变成悬空。
    """
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return ""

    match = re.match(r"\A---[ \t]*\r?\n(.*?)\r?\n---[ \t]*\r?\n", text, re.S)
    if not match:
        return ""

    field = re.search(r"^hxid[ \t]*:[ \t]*[\"']?([^\"'\r\n]+?)[\"']?[ \t]*$",
                      match.group(1), re.M)
    return field.group(1).strip() if field else ""

def build_doc(
    *,
    hxid: str,
    title: str,
    created_at: str,
    model: str,
    skills: list[str],
    author: str,
    tags: list[str],
) -> str:
    return f"""---
hxid: {yaml_string(hxid)}
title: {yaml_string(title)}
created_at: {yaml_string(created_at)}
model: {yaml_string(model)}
skill: {yaml_list(skills)}
authors: {yaml_string(author)}
tags: {yaml_list(tags)}
---

# {title}

> [!NOTE]
> 读者可见版本: 本文为面向读者的最终稿, 不含生成过程与本地操作痕迹. (AI 起草期间请在本目录 .hx-mitemite.md 记录待审核事项; 提交前删除本行提示)

## 0x00 背景

TODO: 说明为什么要写这篇文章, 它想解决什么问题.

## 0x01 核心结论

TODO: 先给出可以被复用的结论, 再展开推导.

## 0x02 关键细节

TODO: 记录关键概念、实现细节、设计取舍或源码依据.

## 0x03 <最后一段内容>

TODO: 正文到此结束. 展望 (50 字内, 抽象领域的判断) 接在这段末尾, 不另起标题.

**不要写"参考来源"章节**  引用关系由站点在页面底部自动渲染成方框 (本文引用 / 本文被引用 / 站外来源),
数据从正文的链接里抽. 正文里正常用链接即可.

> TODO(仅 AI 可见, 提交前删除): 需要用户审核的点请写在 .hx-mitemite.md, 不要写进正文.
"""
