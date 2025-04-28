# Java 跨平台原理
## 1. Java程序是如何执行的
> javac java文件名
>
> java 类名

Java是一门高级语言，编写的程序不能够被计算机直接识别，因此计算机不能直接运行Java程序。于是詹姆斯高斯林就想到一个办法，提供一个Java编译器(Java Compiler => `javac`)，将Java源程序编译为class文件，再提供一个Java虚拟机(Java Virtual Machine => `JVM`)来执行class文件，而Java虚拟机就可以将class文件翻译为计算机能够识别的指令。java命令可以启动Java虚拟机，并告诉虚拟机应该加载的类，当虚拟机将类加载进来后，然后就将该类翻译为计算机能够识别的指令，从而使得我们编写的程序运行起来

## 2. 目前存在哪些常用平台
windows 7 8 10 11 unix linux mac

## 3. Java 如何实现跨平台
要想运行java程序，那么就必须要提供java的运行环境，而java运行环境中就包含了java虚拟机，java虚拟机就可以将class文件翻译为对应平台能够识别的指令，从而让程序在相应的平台下执行起来。
