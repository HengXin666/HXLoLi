# 多文件下内联函数的使用
## 事发
```C++
game::Position game::Player::getPosition() 
```

为什么我把上面那个改为

```C++
inline game::Position game::Player::getPosition() const 
```

也会提示:
```
严重性    代码    说明    项目    文件    行    禁止显示状态    详细信息
错误    LNK2019    无法解析的外部符号 "public: struct game::Position __cdecl game::Player::getPosition(void)const " (?getPosition@Player@game@@QEBA?AUPosition@2@XZ)，函数 "void __cdecl game::gameMainFun(void)" (?gameMainFun@game@@YAXXZ) 中引用了该符号    HX库    D:\command\cc++\C++\HX库\HX库\gameShow.obj    1
```

但是去掉就没有问题的可以运行?

## 解决方案

内联函数通常会在每个调用点进行内联展开，而不是生成单独的函数代码。因此，如果内联函数的定义在一个源文件中，但在另一个源文件中引用了该函数且未能内联展开，就会导致链接错误。

一种解决方法是将内联函数的定义放在头文件中，以便在引用该函数的源文件中进行内联展开。这样可以避免链接错误并确保内联函数在需要的地方正确展开。

所以 需要写在`.h`, 而不是`.cpp`:
```C++
// .h
// 获取坐标
inline Position getPosition() const {
    return this->position;
};

// 获取唯一id
inline unsigned short getId() const { 
    return this->properties.id; 
};
```

其他解决方案可以模仿迁移[多文件下模版使用](../001-多文件下模版使用/index.md)的, 我认为是大同小异的...