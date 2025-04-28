# Python安装Jupyter Notebook配置使用教程
## Jupyter Notebook是什么
- Jupyter Notebook是一个开源的web应用程序,一个交互式笔记本，支持运行 40 多种编程语言。
- 它允许您创建和共享文档,包含代码,方程,可视化和叙事文本。
- 用途包括:数据清洗和转换,数值模拟,统计建模、数据可视化、机器学习等等。
- 支持以网页的形式分享，GitHub 中天然支持 Notebook 展示，也可以通过 nbviewer 分享你的文档。当然也支持导出成 HTML、Markdown 、PDF 等多种格式的文档。
- 不仅可以输出图片、视频、数学公式，甚至可以呈现一些互动的可视化内容，比如可以缩放的地图或者是可以旋转的三维模型。

## 怎么样安装 Jupyter notebook

```cmd
pip install jupyter
```

## 配置Jupyter notebook目录路径

安装完成先不要启动，先配置目录路径。要不然默认打开和保存`Jupyter notebook`文件目录在C盘。

```cmd
jupyter notebook --generate-config
```

生成默认配置文件到`C:\Users\Administrator\.jupyter\jupyter_notebook_config.py`

找到默认配置文件的目录。很多配置文件都是生成到这个目录中。

打开`jupyter_notebook_config.py`搜索`c.NotebookApp.notebook_dir`

把`#`号去掉，把值改为你要存放`Jupyter notebook`文件的目录路径。

以后Jupyter notebook创建的文件都会保存到这个目录路径中。

## 如何启动Jupyter notebook

直接cmd输入, 然后就会跳转到浏览器
```cmd
jupyter notebook
```

## 如何使用
我安装的是中文的了, 还有需要请: [Python安装Jupyter Notebook配置使用教程](https://zhuanlan.zhihu.com/p/54302333)