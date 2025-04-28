# 类型擦除 + 工厂模式, 告别 if-else
## 1. [类型擦除]: 以适配麻烦第三方库为例

假如我们用到了一个`msglib.h`这个第三方库, 它提供了一些类, 并且都有`speak`方法, 然而却不是一个虚函数 (无法多态访问), 如果我们希望使用一个数组存放这些类, 然后调用`speak`, 显然是很面倒的..

```C++
#pragma once
#include <iostream>

struct MoveMsg {
    int x;
    int y;

    void speak() {
        std::cout << "Move " << x << ", " << y << '\n';
    }
};

struct JumpMsg {
    int height;

    void speak() {
        std::cout << "Jump " << height << '\n';
    }
};

struct SleepMsg {
    int time;

    void speak() {
        std::cout << "Sleep " << time << '\n';
    }
};

struct ExitMsg {
    void speak() {
        std::cout << "Exit" << '\n';
    }
};
```

### 1.1 使用`std::variant<>`
显然可以使用`std::variant<>`把这些类装进去, 然后使用.

但是你需要把所有可能的类型罗列到模版中, 并且它占用的空间很大(sizeof = 最大的类的内存空间 + std::size_t(表示当前是那种类型))

并且不符合开闭原则, 你如果有新的类型需要支持, 那么必须要修改`std::variant<>`. 如果`std::variant<>`作为了多个函数的参数, 那岂不是烦也烦死啦? (using: 是不是忘了我?!)

总之, 不优雅..

### 1.2 类型擦除

我们可以提供一个基类(接口), 然后多个子类, 继承自基类, 子类组合一个成员, 而这个成员的类型就是需要擦除类型的类(那岂不是需要给每一个需要擦除类型的类都要适配一个类?! (当然! 但是我们有模版啊!))

- 对于如何自适应构造? 我们使用变长模版+完美转发即可.

```C++
#include <memory>
#include <vector>
#include "msglib.h"

struct MsgBase {
    virtual void speak() = 0;
    virtual ~MsgBase() = default;
};

template <class Msg>
struct MsgImpl : public MsgBase {
    Msg _msg;
    
    template <typename ...Ts>
    MsgImpl(Ts&&... ts) : _msg{std::forward<Ts>(ts)...}
    {}

    void speak() override {
        _msg.speak();
    }
};

template <class Msg, class ...Ts>
std::shared_ptr<MsgBase> makeMsg(Ts&&... ts) {
    return std::make_shared<MsgImpl<Msg>>(std::forward<Ts>(ts)...);
}

int main() {
    std::vector<std::shared_ptr<MsgBase>> arr;
    arr.push_back(makeMsg<MoveMsg>(1, 2));
    arr.push_back(makeMsg<JumpMsg>());
    arr.push_back(makeMsg<SleepMsg>(3));
    arr.push_back(makeMsg<ExitMsg>());

    for (auto&& it : arr) {
        it->speak();
    }
    return 0;
}
```

## 2. 工厂模式, 告别if-else

听说在Ros中, 有类似以下的C++代码:

```C++
#include <iostream>
#include <memory>
#include "msglib.h"

struct RobotClass {
    void recv_data() {
        int type = 0;
        std::cin >> type;

        if (type == 1) {
            is_mov = true;
            mov = std::make_shared<MoveMsg>(1, 2);
        }
        if (type == 2) {
            is_jmp = true;
            jmp = std::make_shared<JumpMsg>(1);
        }
        if (type == 3) {
            is_slp = true;
            slp = std::make_shared<SleepMsg>(1);
        }
        if (type == 4) {
            is_ext = true;
            ext = std::make_shared<ExitMsg>(1);
        }
    }

    void update() {
        if (is_mov) {
            mov->speak();
        }
        if (is_jmp) {
            jmp->speak();
        }
        if (is_slp) {
            slp->speak();
        }
        if (is_ext) {
            ext->speak();
        }
    }

    std::shared_ptr<MoveMsg> mov;
    std::shared_ptr<JumpMsg> jmp;
    std::shared_ptr<SleepMsg> slp;
    std::shared_ptr<ExitMsg> ext;

    bool is_mov;
    bool is_jmp;
    bool is_slp;
    bool is_ext;
};

int main() {
    RobotClass ros;
    ros.recv_data();
    ros.update();
    return 0;
}
```

这样的代码实在是太麻烦了, 首先是`if-else`, 然后如果要添加内容(成员), 就必须要修改三处位置...

我们可以使用类型擦除, 让他以多态形式存在, 然后使用工厂模式, 统一的创建他们出来.

如果有需求不同的构造函数呢? 需要用户进行不同的输入, 那么统一的工厂接口怎么适配不同的构造函数参数列表?

> 答: 不在构造函数里面构造了!
>
> 提供一个初始化函数, 以重载的方法供初始化; 子类就通过调用这个初始化方法, 以初始化类

```C++
#include <map>
#include <memory>
#include "msglib.h" // 不要忘了, 这个是第三方库, 我们只能适配, 不能修改

struct MsgBase {
    virtual void speak() = 0;
    virtual void load() = 0;
    virtual ~MsgBase() = default;

    using Ptr = std::shared_ptr<MsgBase>;
};

namespace msg_extra_funcs { // 无法为 Msg 们增加成员函数, 只能以重载的形式, 外挂追加
    void load(MoveMsg &msg) {
        std::cin >> msg.x >> msg.y;
    }

    void load(JumpMsg &msg) {
        std::cin >> msg.height;
    }

    void load(SleepMsg &msg) {
        std::cin >> msg.time;
    }

    void load(ExitMsg &) {
    }
}

template <class Msg>
struct MsgImpl : MsgBase {
    Msg msg;

    void speak() override {
        msg.speak();
    }

    void load() override {
        msg_extra_funcs::load(msg);
    }
};

struct MsgFactoryBase {
    virtual MsgBase::Ptr create() = 0;
    virtual ~MsgFactoryBase() = default;

    using Ptr = std::shared_ptr<MsgFactoryBase>;
};

template <class Msg>
struct MsgFactoryImpl : MsgFactoryBase {
    MsgBase::Ptr create() override {
        return std::make_shared<MsgImpl<Msg>>();
    }
};

template <class Msg>
MsgFactoryBase::Ptr makeFactory() {
    return std::make_shared<MsgFactoryImpl<Msg>>();
}

struct RobotClass {
    inline static const std::map<std::string, MsgFactoryBase::Ptr> factories = {
        {"Move", makeFactory<MoveMsg>()},
        {"Jump", makeFactory<JumpMsg>()},
        {"Sleep", makeFactory<SleepMsg>()},
        {"Exit", makeFactory<ExitMsg>()},
    };

    void recv_data() {
        std::string type;
        std::cin >> type;

        try {
            msg = factories.at(type)->create();
        } catch (std::out_of_range &) {
            std::cout << "no such msg type!\n";
            return;
        }

        msg->load();
    }

    void update() {
        if (msg)
            msg->speak();
    }

    MsgBase::Ptr msg;
};


int main() {
    RobotClass robot;
    robot.recv_data();
    robot.update();
    return 0;
}
```

不是哥们? 你这里用了虚函数诶! 会变慢的barabara...

> 你那个同事呢他得改几个地方, 首先这里他是if else,又要加一层if else, 然后他这里还要加个布啊判断, 然后这里还得判断布啊, 并得到相应的message那一堆指针, 啪啪啪下来, 你看脑脑壳都炸裂了是吧, 我们是这个这个自己方便重要, 还是计算机那么几纳秒的性能重要, 你为了那几纳秒的性能损失了你好几个小时, 吃饭时间, 是不是你亏啦, 计算机赚啦, 所以我们就把si全部拉给计算机去统一处理吧 ,我们人类的时间比它珍贵多了, 哎你该用的时候用, 你如果发现瓶颈了再去优化也不迟对吧, 先保证我们代码能跑了再说, 你那样写动不动一堆bug, 然后你调试半天几个小时跑掉了, 跑了半天,结果计算机就加速了几纳秒, 值得吧, 嗯啊所以乖乖给我去用虚函数哈!

如果你觉得`factories`中要添加, 就要写一小串. 那你就上宏嘛.

```C++
#define PER_MSG(ClassName) {#ClassName, makeFactory<ClassName##Msg>()},
    inline static const std::map<std::string, MsgFactoryBase::Ptr> factories = {
        PER_MSG(Move)
        PER_MSG(Jump)
        PER_MSG(Sleep)
        PER_MSG(Exit)
    };
#undef PER_MSG
```

更可以: 写一个`msgtypes.inl`:

```C++
PER_MSG(Move)
PER_MSG(Jump)
PER_MSG(Sleep)
PER_MSG(Exit)
```

然后这样:
```C++
#define PER_MSG(ClassName) {#ClassName, makeFactory<ClassName##Msg>()},
    inline static const std::map<std::string, MsgFactoryBase::Ptr> factories = {
        #include "msgtypes.inl"
    };
#undef PER_MSG
```

就像配置文件一样, 并且可以复用 (比如其他地方也需要一个其他操作, 但是宏设计为一样的 (反正用完就`#undef`了))