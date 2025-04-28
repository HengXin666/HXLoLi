# Linux环境搭建--vim 插件

## 安装vim
(有的Linux可能不是自带的=-=)

```Bash
yum install -y vim
```

## 创建配置文件

在`默认目录`下创建文件 `~/.vimrc`，保存以下内容

```Bash
vim ~/.vimrc
```

```文本
set number
syntax on
set tabstop=4
set shiftwidth=4
```

解释:

- 显示行号：set number
- 打开语法高亮：syntax on
- Tab 大小：set tabstop=4
- 缩进大小：set shiftwidth=4

*保存成功后，试着用Vim写一段代码，发现已经可以显示行号、语法高亮提示、并且tab缩进改为4个空格了。*

# vim 备忘

- [Vim 备忘清单](https://quickref.me/zh-CN/docs/vim.html) # 这个网站还有其他的备忘清单, 感觉很不错



<div style="margin-top: 1080px;">

---
</div>

## 安装 Vundle

Vundle是Vim bundle的缩写，是一个Vim插件的管理器。通过Vundle我们可以安装很多有用的插件。具体可以查阅一下GitHub上的信息:

[Vundle.vim](https://github.com/VundleVim/Vundle.vim#about)

运行下面的命令下载Vundle (请确保网络顺畅...)

```Bash
git clone https://github.com/VundleVim/Vundle.vim.git ~/.vim/bundle/Vundle.vim
```

将下面的内容追加到`~/.vimrc`中

```文本
set nocompatible

filetype off   

set rtp+=~/.vim/bundle/Vundle.vim

call vundle#begin()

Plugin 'VundleVim/Vundle.vim'

call vundle#end()            

filetype plugin indent on
```

启动Vim，运行命令:

`:PluginInstall`

这样Vundle就安装成功了。

## 设置括号、引号匹配

需要用Vundle来安装一个`delimitMate`插件

在`.vimrc`文件中添加一行 (注意添加在`Plugin 'VundleVim/Vundle.vim'`下方一行)

```文本 
Plugin 'Raimondi/delimitMate'
```

然后启动Vim再次运行 :PluginInstall 命令。写入一段代码，看一看括号和引号的自动匹配功能是不是已经实现了

### 如果报错: 一般是网络原因

可以尝试手动安装 该库~ (但是上面的`Plugin 'Raimondi/delimitMate'`仍然要写在配置文件里面)

```Bash
git clone https://github.com/raimondi/delimitmate ~/.vim/bundle
```

## 自动补全

自动补全是代码编辑器中最重要的特性，当前Vim用的比较多的插件有YCM(YouCompleteMe)和Coc。具体的安装步骤可以参见GitHub上的说明：

YCM: https://github.com/ycm-core/YouCompleteMe

Coc: https://github.com/neoclide/coc.nvim

~~(为什么不使用VSCode远程连接来写Linux上的代码呢qwq?)~~