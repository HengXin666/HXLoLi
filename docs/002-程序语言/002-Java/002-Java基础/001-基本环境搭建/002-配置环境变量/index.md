# Java 开发环境搭建
1. 安装目录结构说明
- jdk(Java Development Kit, Java开发工具包)

    bin目录：（binary）

    存放的是可执行文件

    db目录：（DataBase）

      存放的是使用 Java 语言开发的数据库

    include目录：

    存放的是C++语言的头文件（Java 是在C++语言的基础上开发出来）

    jre目录：（Java Runtime Enviroment，Java运行时环境）

    程序员在开发程序的时候，也会运行程序，运行程序需要运行时环境，因此就会使用到 jre。因为开发的时候会使用到开发工具包( JDK ),因此在安装 JDK 后会出现 JDK 目录lib目录：（Library）

    存放的是开发时使用的库文件，在业界，以.jar结尾的文件称为库文件或者jar包

    src.zip文件

    Java 开发工具包的源代码，也就是高斯林的团队写的代码

- jre（Java Runtime Enviroment，Java运行时环境）
主要是用于开发完成之后部署运行使用

## 配置 Java 环境变量

```
C:\Program Files\Java\jdk1.8.0_171\bin
C:\Program Files\Java\jdk1.8.0_171\jre\bin
```

b. 在命令行中输入 java 命令，然后按下回车键；如果显示了 java 命令相关的信息，则表明 java 命令可以执行

c. 在命令行中输入 javac 命令，然后按下回车键；如果显示了 javac 命令相关的信息，则表明 javac 命令可以执行。此时已经说明 java 环境已经搭建好

d. 在命令行中输入 java -version 命令，然后按下回车键；如果显示了 java 版本，则表示 java 命令可以执行