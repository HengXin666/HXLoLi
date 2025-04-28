# protobuf（C++）的使用（windows）
## 前言
我在写游戏, 然后传输的时候, 不知道怎么传输数据(它似乎有点复杂), 查阅网络上的教程, 基本上都是 五子棋这种慢慢摸摸的游戏, 传输十分简单, 所以我问gpt怎么搞, 比如传输一个`std::vector<类>`是不是得for一个一个传输, 然后它说 **类不能直接传输** ~~(结构体又可以?!(实际上是已经被序列化了, 但是还是有问题, 比如不同机器的字节对齐不一样等等))~~(实际上这个就和json一样, 只不过是二进制的, 还是要序列化/反序列化(解析), 不过速度更快(2~3倍以上(比json)))

[protobuf（C++）的使用（windows）](https://blog.csdn.net/pengshengli/article/details/90378455) <-- 不行的...~~(我太菜了?)~~

[爱编程的大丙-Protobuf](https://subingwen.cn/cpp/protobuf/)

[Windows平台visual studio2022运行protoBuf](https://blog.csdn.net/qq135595696/article/details/125699770) <-- 神！


```text
libprotocd.lib
libprotobufd.lib
```

---

不用执行, 直接cmake编译的... 如上, 下面的还是不会...

## 安装 vcpkg

记得配置代理！！！！使用win的powerShell 使用代理

[Visual Studio开源库集成器Vcpkg全教程--利用Vcpkg轻松集成开源第三方库](https://blog.csdn.net/cjmqas/article/details/79282847)