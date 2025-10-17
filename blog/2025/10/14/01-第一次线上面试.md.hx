---
authors: Heng_Xin
title: 记录第一次线上面试
date: 2025-10-14 20:41:46
tags:
    - 面试
---

这是我第一次线上面试, 人生中对于职业的第二次面试... (第一次是线下面试qwq)

<!-- truncate -->

> 本文暂时不外放!!!

这次是腾子的一面. (慌飞了, 昨天下午收到面试, 还只能约昨晚和今晚. 我当时真的是毫无准备...)

吃饭一点胃口都没有... 整个人都是慌的, 就像是老师点名, 而你已经知道下一个就一定是你而且还毫无准备的那种qwq..

我也是马上开始准备啊qwq, 首先把简历上的项目至少准备了. 所谓掌握的内容就不管了, 假装他不会考吧.. 只能自己发挥了.

看看了算法. 然后吃饭时候看别人面试(网上几乎没有什么C++的这种.. 有也是几年历史了qwq), 然后就是一些项目延伸的, 最后把自我介绍准备好, 稍微契合一下岗位需求...

## 开战

我提前几十分钟就来到tx会议了. 然后那时候校园网还抽搐了... 还好没有开始. 我切换为 流量热点, 继续读流畅自我介绍.

面试官也提前了5、6分钟就到了~

## 开面

1. 自我介绍

2. 直接开问 C++ 了

问了以下表达式的值: (有些例子忘记了qwq)

```cpp
// 64 位 os
void demo(char str[5]) {
    auto a = "123";
    char const* b = "123";
    std::string s = "123";
    char arr[] = "123";
    char arr2[5] = "123";

    sizeof(a);      // 8, 是指针
    sizeof(b);      // 8
    sizeof(s);      // 24 (?) 16 我不记得了, 但是我提到内部有短字符串优化, 使用共用体
    sizeof(arr);    // 4 (包含了 \0)
    sizeof(arr2);   // 5 因为明确指出大小了
    sizeof(str);    // 8 因为退化为指针了, 在函数参数

    std::strlen(a);         // 3
    std::strlen(b);         // 3
    std::strlen(s.data());  // 3
    std::strlen(arr);       // 3
    std::strlen(arr2);      // 3
    std::strlen(str);       // 3
    // 我回答都是3, 因为 strlen 就是通过 \0 判断长度的.

    // 面试官挖坑:
    char data[3] = {'1', '2', '3'};
    std::strlen(data); // ub ! 这不是期望的调用方法
}
```

再问了 auto 推导: (还有好几个case, 我忘记了qwq)

```cpp
void demo(char p[5]) {
    int a;
    int const b;
    int& c = a;

    auto a1 = b;    // int
    auto& a2 = b;   // const int&
    auto&& a3 = b;  // const int&

    int&& d = std::move(a);

    auto a4 = d;    // int
    auto&& a5 = d;  // int& 名称为左值
    auto p1 = p;    // char* 因为退化了

    int arr[3];
    auto p2 = arr;  // int*
    auto& p3 = arr; // int (&)[3]
}
```

然后说看你写的都没有问题, 就说看我对左右值都挺了解的, 让我说说什么是左值, 什么是右值.

> 我说, 左值 就相当于一个别名.
>
> 而右值一般是使用在所有权转移的时候, emmm.. 我是这么理解的 (说实话我当时脑子一片空白, 根本无法组织语言. 刚刚的话我是梦到什么就说什么qwq... 体会到看面试视频, 明明你认为这面试官说的很简单, 然后回答时候却乱哄哄的情景了吧qwq)
>
> 然后我看面试官的表情似乎很不满意我的回答... (我当时觉得我要完蛋了qwq)
>
> 然后我应该就补充了 右值, 它又分为 纯右值 和 将亡值. 然后我忘记我说的什么了. 然后面试官让我分别举例说说他们. (面试官大好人! 真的会主动引导面试者!)
>
> 我说 纯右值 就比如说字面量那些, 将亡值 就是一些表达式的计算过程的值 (比如 `(a + a)`) 或者 函数返回值 (函数返回值这里不严谨, 被面试官点到了)
>
> 然后我回复说是 比如返回 int 或者 auto 这种, 不是引用的那种返回值. 这是 将亡值 (面试官应该表示勉强ok)

> 好吧, 我好像忘记他问的是左值引用和右值引用还是左右值了qwq, 可能跨服聊天了qwq, 不对好像就是问 左值引用和右值引用 qwq... 记不清了

> [!TIP]
> 八股说, 左右值通常就是等号左右边的值.
>
> - 左值是可以取地址, 通常有名字的变量
> - 右值不可被取地址, 表示应该具体的数据值, 通常是常量、临时变量
>
> 语言律师:
>
> https://hengxin666.github.io/cppreference-zh-cn/zh/cpp/language/value_category.html
>
> - 泛左值 (glvalue)("泛化 (generalized)"的左值)是一个求值可确定某个对象或函数的标识的表达式
> - 亡值 (xvalue)("将亡 (expiring)"的值)是代表它的资源能够被重新使用的对象或位域的泛左值
> - 纯右值 (prvalue)("纯 (pure)"的右值)是求值符合下列之一的表达式
>   - 计算某个运算符的操作数的值(这种纯右值没有结果对象)
>   - 初始化某个对象(称这种纯右值有一个结果对象)。
>   - (结果对象可以是变量, 由 new 表达式创建的对象, 由临时量实质化创建的临时对象, 或者前述三类对象的成员。注意, 非 void 的弃值表达式有结果对象(即被实质化的临时量)。并且在作为 decltype 的操作数以外的情况下, 每个类类型或数组类型的纯右值都有结果对象; )
> - 左值 (lvalue) 是并非亡值的泛左值
> - 右值 (rvalue) 是纯右值或者亡值

最后就是一道场景设计题. 不需要实现代码, 只需要实现 API 和类的声明.

要求不能使用 STL 的数据结构, 设计一个 单向列表 给 C++ 开发者. 注意 API 要适量不是越多越好.

然后就开始了, 一共 15 min, 提前 ok 就叫一下面试官.

这是我写好的代码:

```cpp
template <class T>
struct Iterator {
    T operator*();
    bool operator==(Iterator const&) const;
    bool operator!=(Iterator const&) const;
private:
    Iterator* next;
    T t;
};

template <class T>
class List {
public:
    using iterator = Iterator<T>;
    using type = T;

    template <class U>
        // U -> T
    void push_front(U&& u);

    void pop_front();

    void erase(iterator it);

    template <class U>
        // U -> T
    void insert(iterator it, U&& u);

    template <class U>
    iterator find(U const& u) const;

    iterator begin();
    iterator end();

    // std::size_t size() const;
private:
    iterator* head;
};
```

可能是 8 分钟吧... 说实话我有点忘记 (或者说我一直都不是特别清楚迭代器类怎么写qwq), 然后就叫面试官了~

面试官指出 3 个问题:

1. 我看你一开始是使用 ListNode 的, 为什么又换成 迭代器了呢?

> 我回答是 考虑到 API 设计, 使用迭代器模式, 让用户可以使用统一的接口.

让后就指出目前我的迭代器的职责不清晰的问题, 引导我考虑一下应不应该存放数据.

2. 然后指出 push_front / pop_front 接口, 不应该是对尾部操作吗? 这样会导致插入数据倒序, 用户体验不好..

> 我回复是我设计是用头插法, 这样是 O(1).. 但是确实会有这个问题
>
> (PS: 到了最后我忘记要修改这个了, 实际上可以维护一个 head 和 tail, 这样就可以支持头插和尾插了qwq)
>
> (不过面试官好像也忘记这事情了, 后面也没有指出我qwq 也可能是知道但是没有再说了)

3. 指出 `template <class U> void push_front(U&& u);` 这种设计 ok, 可以指出单层构造. 但是不支持 `push_front(1, 2)` 这种啊

> 说实话, 我确实没有考虑到. 因为当初设计时候期望是用户可以直接 `push_front({1, 2})` 使用的qwq...
>
> 然后面试官还指出, 可能这个类更不支持 `=` 运算符, 你怎么搞? 再面试官问我怎么修改?
>
> 我回复是使用 C++11 的可变参数 然后配合 参数转发 (这个我忘记有没有说了), 然后达成原地构造的效果.

然后面试官又给我 10 min 进行修改.

最终:

```cpp
template <class T>
struct ListNode {
    template <class U>
    ListNode(U&& u);

    template <class... Args>
    ListNode(Args&&... args);

    ListNode* next();
    T& get();
    T const& get() const;
private:
    ListNode* _next;
    T _t;
};

template <class T>
struct Iterator {
    T operator*();
    bool operator==(Iterator const&) const;
    bool operator!=(Iterator const&) const;
private:
    ListNode<T>* node;
};

template <class T>
class List {
public:
    using iterator = Iterator<T>;
    using type = T;

    template <class... Args>
    void emplace_front(Args&&... args);

    template <class U>
        // U -> T
    void push_front(U&& u);

    void pop_front();

    void erase(iterator it);

    template <class U>
        // U -> T
    void insert(iterator it, U&& u);

    template <class... Args>
    void emplace(iterator it, Args&&... args);

    template <class U>
    iterator find(U const& u) const;

    iterator begin();
    iterator end();

    // std::size_t size() const;
private:
    iterator* head;
};
```

面试官表示没有什么问题.

然后是最后一个问题 (我不确定是什么时候问的了)

大概意思是, 日后到公司是做代码质量研发嘛, 你要如何让业务组的同学们避免代码质量上的问题呢? 以及如何量化你代码质量的成果是否是有效的呢?

> 我的回答大致是:
>
> 避免的话, 首先是制定一个代码规范, 然后让同事们都使用同一个规范, 不要各写各.
>
> 然后是教导他们为什么要这样写, 因为有的同事可能并不知道为什么要这样规范. 所以需要教导或者告知一下.. (后面我编不出来了, 脑子空白, 依旧是梦到什么说什么...qwq)
>
> 成果量化的话:
>
> 我认为是可以通过 (实施了这个什么什么之后) 代码的 BUG 率 / 段错误率的一个变化统计
>
> 以及 代码测试的代码覆盖率什么的, 初次编译的通过率什么的 进行量化. (依旧是梦到什么说什么; 不过似乎面试官没有怎么为难我, 表示ok就结束面试了!)
>
> 实际上应该可以再细分量化, 比如使用一个系统, 统一的记录每个人的 每次提交的编译的通过率、以及代码测试的通过率、编译时常等进行量化, 并且记录编译的时间点和编译的项目等.

3. 反问:

我问到时候来这里具体是负责做什么的: (我不是很记得面试官怎么回答了, 只有大致的qwq)

就是目前已经有静态代码工具了也有动态编译分析工具了. 就可能会有新工具的研发以及当项目版本迁移, 比如 gcc -> clang 的情况, 需要看看有什么需要注意的, 然后更业务的同学沟通; 最后就是 之前问题的那些事情 如何量化, 然后最后要做年终总结汇报什么的...

然后我问目前公司项目的C++版本最高支持是多少? (抱歉, 我以后面试都会问这个, 因为涉及到我的游戏体验啊qwq)

面试官说大部分是 C++17, 也建议到 C++17; 只是有一些项目是 C++11 的. 不过问题不大.

哦, 我还问了之后的面试流程. 复试就是主管面(主要考智力什么的), 然后是 HR 面(一般没有什么问题, 就看看人).

最后, 回去等通知~, 哦当晚(洗澡出来后)发现官网状态变为待复试 ??!!

> 顺带一提, 使用的是tx会议那里的面试工具, 可以写代码. 目前不清楚具体的C++版本, 那写的 C++ (7.x.x) 是什么意思? (GCC 7.x ?)