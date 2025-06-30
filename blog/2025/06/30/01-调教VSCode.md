---
authors: Heng_Xin
title: 调教VSCode, 以代码提示C++
date: 2025-06-30 21:36:28
tags:
    - vscode
---

实习环境使用vscode, 然后我登录我的账号, 把我的插件和配置全量同步, 然后发现cpp没有代码提示了, 遂狠狠滴调教VsCode...

<!-- truncate -->

不知道为什么, clangd 在实习环境适配不好... (ubt系列linux发行版的锅, 比如麒麟os) 包管理能下载的东西太tm古老了!!!

并且 clangd 实际上是依赖 cmake 的(`compile_commands.json`), 但是公司项目是 cmake 编译测试项目, 发行版却是通过 makefile 直接编译的 ...

导致 clangd 根本不能用了 (我删除了 =-=)

然后还是不行, 我百思不得其解...

但是后面破案了, 我把默认的 vsc的c/c++插件禁用了, 怪不得没有提示 qwq...

然后写 `.vscode/c_cpp_properties.json`:

```json
{
  "configurations": [
    {
      "name": "default",
      "includePath": [
        "${workspaceFolder}/**"
      ],
      "defines": [],
      "compilerPath": "/usr/bin/g++",
      "cStandard": "c17",
      "cppStandard": "c++20",
      "intelliSenseMode": "linux-gcc-x64"
    }
  ],
  "version": 4
}
```

> [!TIP]
> `${workspaceFolder}/**`, 这表示把当前项目所有子目录都加入头文件搜索路径, 适合小型项目或不依赖太多三方库的项目!

如上配置, 基本上OK了吧...