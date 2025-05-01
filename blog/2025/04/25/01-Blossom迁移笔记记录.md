---
authors: Heng_Xin
title: Blossom迁移笔记记录
date: 2025-04-25 20:04:36
tags:
    - python
    - 工具
---

本文主要记录一下, 我从 Blossom 迁移百万字的笔记的过程... 主要使用了类`字典树`的思想编写一个Python程序, 以构建路径, 以及处理相对路、本地图片的问题...

<!-- truncate -->

## 与 AI 斗智斗勇

我直接把需求扔给AI, 我可懒得想...

提示词:

```text
帮我编写一个本地md整理程序

首先，原本的md文件是自带文件结构的

例如

C++/STL/array.md

你需要把他们重构为

001-C++/001-STL/001-array/index.md
比如 我提供的路径是 /C++/STL/array.md
   你应该生成为 /001-C++/001-STL/001-array/index.md

特别的
1.
图片全部以相对路径或者https的方式给出

如果它是本地路径，那么需要 把对应的图片 拷贝到 index.md 同级目录下
并且修改 md的图片链接

2. 关于 序号 001

我提供了一个文件，其内容格式为

┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ ↓↓ 文章列表 ↓↓ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┃ 排序 [ID] [版本] [时间] 文章路径
┠────────────────────────────────────────────────────────────────────────────
┃   1 [20157] [ 47] [2025-04-22 17:17:41] /计佬常識/数据结构与算法/[algorithm]目録
┃   2 [20155] [  9] [2025-04-09 16:23:48] /计佬常識/数据结构与算法/[导论]基础/[开端]时空复杂度
┃   3 [20156] [ 22] [2025-04-19 20:29:26] /计佬常識/数据结构与算法/[数据结构]线性表/线性表
┃   4 [20323] [  1] [2025-03-16 17:29:01] /计佬常識/数据结构与算法/[数据结构]线性表/顺序存储结构-数组
┃   5 [20324] [  2] [2025-03-16 17:26:58] /计佬常識/数据结构与算法/[数据结构]线性表/顺序存储结构-静态链表
┃   6 [20325] [  1] [2025-03-26 16:48:40] /计佬常識/数据结构与算法/[数据结构]线性表/链式存储结构-单向链表

你需要以这个 排序为 排序，

注意，对于单个文件夹内，文件和文件夹共用这个排序，也就是目录也是001-这种形式

然后数字不是照抄 我提供的文件的，而是每个目录都是从001开始，独立的，，只是排序用到了我提供的文件

3. .md的文字有些以 https引用文章，他们的格式类似：
[文件流](https://blog.hxloli.com/blog/#/articles?articleId=20180 "##20180##")
[字符串流](https://blog.hxloli.com/blog/#/articles?articleId=20183 "##20183##")
其中 articleId 已经在我提供我文件中给出。

你需要替换为 .md相对路径，而不是 https . (仅对于https://blog.hxloli.com/blog/而言)，其他的可能是正常的url

注意要考虑新链接 应该是带有排序和index.md的，建议最后再统一处理，先把文件目录结构生成好
```

结果就是 GPT-4o 乱回答, 连正则表达式都写不对...

ds也就写对了正则表达式...(还是我说了好几次才改对的 =-=)

> 它们主要是连序号都搞吧明白, 明明要求是 `单独`文件夹下, 序号从 `001` 开始...
>
> 根本妹人听懂...
>
> 而且GPT又傻又懒, 说了都不改... 必需以强硬的态度骂国粹了才改... 但依然是错的.
>
> ds 思考了很多, 但是不行...

因此只好我自己来了 (至少烦人的正则表达式我可以直接用...)

## Py Md 小工具

因为我注意到, 其序号在前的, 就是原本我 Blossom 中文件的排序, 那问题就简单多了

很容易就想到, 把一个 Path 按照 `/` 切分

如: `/计佬常識/数据结构与算法/[algorithm]目録` -> `[计佬常識, 数据结构与算法, [algorithm]目録]`

最后 补上一个 `index.md` 然后把内容复制过来即可.

特别的, 我们需要序号嘛. 然后注意到, 先来的序号肯定是小的, 然后就贪心就好了...

写一个`字典树`模拟建造的过程, 同时记录这个目录是否已经创建, 有就不`new`, 无就可以`mkdir`了

> (感觉可以不用那么麻烦...)

具体代码:

```python
import os
import re
import shutil

def fxxkPath(path: str) -> str:
    # 定义非法字符的正则模式 (Windows下的特殊字符)
    illegalChars = r'[<>:"/\\|?*]'
    
    # 用正则替换非法字符为合法字符
    res = re.sub(illegalChars, '_', path)
    
    # 返回合法的路径
    return res

# 特殊路径缓存
memoPathItem = {}

# 字典树, 边构建, 边输出
class Node:
    def __init__(self):
        self.next = {}  # 存储下一级节点
        self.idPath = {}  # 存储路径标识

    def _add(self, s: str, maePath: str):
        """模拟树结构的添加"""
        fkStr = fxxkPath(s)

        if (fkStr != s):
            memoPathItem[s] = fkStr
            s = fkStr

        if s in self.next:
            return self.next[s], self.idPath[s]
        
        node = Node()
        self.next[s] = node
        idPath = f"{len(self.next):03d}-{s}"
        self.idPath[s] = idPath
        
        os.makedirs(os.path.dirname(f"{maePath}/{idPath}/"), exist_ok=True)
        
        return node, idPath


def insert(root: Node, path: str, coutPath: str):
    """path insert to Tree"""
    list = path.split('/')
    node = root

    for s in list:
        if (len(s) == 0):
            continue
        node, newPath = node._add(s, coutPath)
        coutPath = f"{coutPath}/{newPath}"

    return coutPath

def parse_log(srcRoot, logPath, coutPath):
    """改进后的日志解析函数"""
    
    root = Node()

    # id-mpa: id - (NewPath(新路径), 原路径)
    idMap = {}
    
    with open(logPath, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            # 跳过非数据行
            if not line.startswith('┃') or line.startswith('┠') or '排序 [ID]' in line:
                continue
            
            # 使用更精确的正则表达式
            match = re.match(
                r'^┃\s+(\d+)\s+\[(\d+)\]\s+\[\s*(\d+)\s*\]\s+\[([^\]]+)\]\s+(.+)$',
                line
            )
            if not match:
                continue
            
            _, aid, _, _, path = match.groups()

            # 把 path 的空格和. 替换为 '' (空)
            path = path.replace(' ', '').replace('.', '').replace(':', '')

            path = path.strip()

            # 此处必然创建 index.md
            makeFile = f"{insert(root, path, coutPath)}/index.md"
            srcFile = f"{srcRoot}/{path}.md"

            print("build", makeFile)
            
            if os.path.exists(srcFile):
                shutil.copy2(srcFile, makeFile)
            else:
                print(f"Warning: Source file {srcFile} does not exist.")
            
            idMap[aid] = (makeFile, f"{srcRoot}/{os.path.dirname(path)}")

    return idMap


def syoriFile(idMap):
    """处理文件, 把图片进行复制, 把链接替换为相对路径"""

    # 处理图片 (需要copy过来)
    def process_image(match, current_dir, oldPath):
        alt = match.group(1)
        img_path = match.group(2)
        
        # @todo 特别的, 如果是本地路径, 但文件不存在, 则会报错或者终止!
        if not img_path.startswith('http'):
            img_name = os.path.basename(img_path)
            dest = os.path.join(oldPath, img_path)
            if os.path.exists(dest):
                shutil.copy(dest, f"{current_dir}/{img_name}")
                print("cp img:", img_name)
            else:
                print("图片不存在:", dest, "来自", current_dir, "index.md")
                exit(-1)
            return f'![{alt}](./{img_name})'
        return match.group(0)

    # 处理图片链接
    def replace_link(match, id_map, current_dir):
        text = match.group(1)
        aid = match.group(2)
        
        if aid in id_map:
            target = id_map[aid][0]
            rel_path = os.path.relpath(target, start=current_dir).replace(os.sep, '/')
            return f'[{text}]({rel_path})'
        return match.group(0)

    for id, path in idMap.items():  # 使用 .items() 遍历字典
        print(id, path)

        paPath = os.path.dirname(path[0])  # 获取当前文件的父目录

        with open(path[0], 'r+', encoding='utf-8') as f:
            content = f.read()
            
            # 处理图片
            content = re.sub(
                r'!\[(.*?)\]\(((?!http).*?)\)',
                lambda m: process_image(m, paPath, path[1]), 
                content
            )
            
            # 处理文章链接
            content = re.sub(
                r'\[(.*?)\]\(https://blog\.hxloli\.com/blog/.*?articleId=(\d+).*?\)',
                lambda m: replace_link(m, idMap, paPath),
                content
            )
            
            f.seek(0)
            f.write(content)
            f.truncate()

                
def main(logPath, srcRoot, outputDir):
    # 标准化路径
    srcRoot = os.path.normpath(srcRoot)
    outputDir = os.path.normpath(outputDir)
    
    # 解析日志, 并且复制到新路径, 并且 记录 id - (新路径, 原路径), 然后返回 idMap
    idMap = parse_log(srcRoot, logPath, outputDir)
    
    # 创建输出目录结构并复制文件
    syoriFile(idMap)

if __name__ == '__main__':
    main(
        logPath='./cin/BML_1_20250425_143534_243/log.txt',
        srcRoot='./cin/BML_1_20250425_143534_243',
        outputDir='./cout'
    )
```

还是很简单的, 但是在这区间还是遇到很多问题, 比如 Blossom 的文件是可以命名 `空格.:/` 这种东西的.

因为程序内部似乎没有限制... 当初也没有想那么多,

谁知道, 一导出直接炸了... `/`被强制创建文件夹、`空格.:`直接被替换为 `''` (空), 导致很多东西都解析不了...

以后命名需要注意一下. (不过影响的内容不多, 只有少数几篇有问题.. (但是`.空格`这种, 因为力扣题目的标题包含这些, 我就只好py先预处理替换了qwq))