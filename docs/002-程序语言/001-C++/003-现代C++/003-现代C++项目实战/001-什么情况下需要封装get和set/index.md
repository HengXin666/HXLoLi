# 什么情况下需要封装get/set
> 之前我们在[二、RAII与智能指针](../../002-高性能并行编程与优化/002-二、RAII与智能指针/index.md)の『2.2.1.2 不变性: 请勿滥用封装』中提到`仅当出现“修改一个成员时，其他也成员要被修改，否则出错”`的现象时，才需要getter/setter封装。

~~如果你学过java, 那么你肯定疑惑得不得了, 似乎被当成傻子了~~

- [【C/C++】什么情况下需要封装get/set](https://www.bilibili.com/video/BV1Sm421G7P9/) 的学习笔记!

小彭老师指出，面向对象封装的意义在于维护“不变量”。get/set就是保证在设置一个成员的同时，另一个与之有依赖的成员也能更新，相当于对set操作做了个hook。如果你的结构体完全是平凡类型，每个成员都能随意单独取值，设值，互不干扰。那就没必要设置getter/setter了，直接public暴露出来即可。

还顺便介绍了构造函数参数非常多时的解决方案: builder模式([建造者模式](../../../../../001-计佬常識/002-设计模式/007-创建型模式/005-建造者模式/index.md))，以及C++20的指定初始化语法。

也介绍了接口臃肿的解决方案: 接口多继承。多继承含有成员的普通类是错误的，糟糕的设计。但是接口完全可以多重继承，即使是禁止类多继承的Java也支持接口的多继承。同一个类当然可以支持多种接口，比如iostream就同时支持istream和ostream接口。

最后，也介绍了当一个参数需要可选的多个接口时的解决方案: 定义共同的基类然后`dynamic_cast`，还提出了`dynamic_cast`的替代方案: `toDerived()`，在《以撒的结合》中就用了这种方案，例如`Entity`是实体类，可以通过`Entity:toPlayer()`获取玩家子类，如果不是玩家类则返回`null`。最后，还实现了经典的**访问者模式**，解决`dynamic_cast`和`toDerived()`流派**不符合开闭原则**的问题，这下对味了。

## 一、封装get/set的必要性和应用场景
- 对于**平凡类型**，可以直接访问成员而 **不需要** get/set.

- 但是对于**非平凡类型**，修改一个成员会影响另一个成员，需要使用get/set来**避免代码冗余和非法状态**。


封装get/set可以使类的状态保持合法同时避免用户绕过set函数单独设置成员导致出错。同时，对于可能进入**非法状态**的操作，应该封装为`private`，避免用户直接调用。

例如下面代码:

我们修改vector的大小(setSize), 其`.data()`(存放数据的指针)也会跟着改变

```C++
#include <print> // C++20
#include <vector>

int main() {
    std::vector<int> v;
    std::println("{} {}", (void *)v.data(), v.size()); // v.getData(), v.getSize()
    v.resize(14); // v.setSize(14)
    std::println("{} {}", (void *)v.data(), v.size()); // v.getData(), v.getSize()
    v.resize(16); // v.setSize(16)
    std::println("{} {}", (void *)v.data(), v.size()); // v.getData(), v.getSize()
    return 0;
}
```

那么假如我们自己实现一个vector类: 如果每次修改其内容, 都要delete再new, 那这样岂不是很麻烦?

```C++
struct Vector {
    int *data;
    size_t size;

    Vector() : m_data(new int[4]), m_size(4) 
    {}
};

void main() {
    Vector v;
    v.size = 16;
    delete[] v.data;
    v.data = new int[16]; // 实际上还是需要拷贝原来的数据, 这里省略了
}
```

因此我们可以将其封装起来:

```C++
struct Vector {
private:
    int *m_data;
    size_t m_size;

public:
    Vector() : m_data(new int[4]), m_size(4) 
    {}

    void setSize(size_t newSize) noexcept /* 禁止抛出异常 */ {
        m_size = newSize;
        delete[] m_data;
        m_data = new int[newSize];
    }

    int *data() const {
        return m_data;
    }

    size_t size() const {
        return m_size;
    }
};

int main() {
    Vector v;
    v.setSize(14);
    v.setSize(11);
    return 0;
}
```

对于上面为什么使用`private`, 是因为如果不这样, 那么使用这个类的人依然可以通过`v.size = 721`来修改其大小, 而实际上的指针却没有指向新的扩容的空间, 显然是有问题的, 故需要`private`保护起来!

> [!TIP]
> 一个好的类设计应该使它任意的操作, 都不会使它进入非法状态.
>
> > 类的状态有`中间状态`和`合法状态`
> >
> > 比如你进入到`setSize`里面, 执行到`m_size = newSize;`语句的时候是中间状态(非法状态), 等到你执行完`setSize`后, 类才变为合法状态.
> >
> > 又比如这个类它显然是线程不安全的: 一个线程`delete[] m_data;`时, m_data已经是野指针了, 在它`m_data = new int[newSize];`之前, 其他线程如果对它操作, 那么都是非法的!

通过上面的学习, 我们就知道, 对于下面这种 平凡类型, 直接`pubilc`即可:

```C++
#include <print> // C++20

struct Point {
    double x;
    double y;

    Point operator+(Point const &other) const {
        return Point(x + other.x, y + other.y);
    }
};

int main() {                           // 聚合初始化
    Point a = Point{ .x = 1, .y = 2 }; // 等价于 Point{1, 2}
    Point b = Point{ .x = 2, .y = 3 }; // 等价于 Point{2, 3}
    Point c = a + b;
    std::println("{} {}", c.x, c.y);
    c.x = 1;
    return 0;
}
```

## 二、一步一步使用建造者模式处理超长的构造函数参数列表
### 2.1 案例
比如现在我们有一个某文件数据库/总之是网络环节, 需要有一个文件指针`fd`/或者说是套接字, 然后需要指定一堆参数, 什么ip, 端口, 用户名, 密码...:
```C++
#include <print>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

struct Connection {
    int fd;

    Connection( // 超多参数的构造函数
        std::string serverAddress,
        int port,
        bool useSSH,
        std::string sshCertPath,
        std::string sshPKeyPath,
        std::string sshCAFilePath,
        std::string username,
        std::string password,
        bool enableFastTCPOpen,
        int tlsVersion,
        std::chrono::seconds connectTimeout,
        std::chrono::seconds readTimeout) {
        // code...
    }
};

Connection cnn("114.514.0.721", 2233, true, "xxx/yyy.pem", "xxx/zzz.pem", ...);
```

每次用都写这么一大串, 烦也烦死了...

即便他们有默认参数, 而我们需要修改其中一个值, 就要把整个该死的构造函数给抄下来, 那烦也烦死了...

有一个解决方案就是, 把这些死人参数提升为:

```C++
struct ConnectionBuilderBase {
    std::string serverAddress;
    int port;
    bool useSSH = false;
    std::string sshCertPath = "";
    std::string sshPKeyPath = "";
    std::string sshCAFilePath = "";
    std::string username = "admin";
    std::string password = "password";
    bool enableFastTCPOpen = true;
    int tlsVersion = 1;
    std::chrono::seconds connectTimeout = 10s;
    std::chrono::seconds readTimeout = 5s;
    std::vector<std::string> args;
};

struct Connection {
    int fd;

    Connection(ConnectionBuilderBase params) {
        // code...
    }
};

Connection cnn( ConnectionBuilderBase {
    .port = 23333,
    .username = "Heng_Xin",
})
```

那现在如果要实现: 如果`useSSH`为`true`那么需要指定`sshCertPath`&`sshPKeyPath`&`sshCAFilePath`, 而为`useSSH`为`false`则不需要指定。

如果按照上面的代码, 那么如果使用者不知道有这个要求, 设置为`true`, 又没有指定Path, 然后内部报错/抛出异常, 那么使用者就会叽叽歪歪你这个类也太难用了, 设计得好骚脑啊~我怎么知道...

亦或者, 使用了上面这种初始化方式, 但是以后又有新需求了, 新增了一个`sshHXFilePath`, 如何让用户感知到呢?

一种方法是:

```C++
#include <optional>

struct ConnectionBuilderBase {
    std::string serverAddress;
    int port;
    struct SSHParams {
        std::string sshCertPath = "";
        std::string sshPKeyPath = "";
        std::string sshCAFilePath = "";
    };
    std::optional<SSHParams> userSSH = std::nullopt; // 使用则指定 SSHParams, 不使用则这样
    std::string username = "admin";
    std::string password = "password";
    bool enableFastTCPOpen = true;
    int tlsVersion = 1;
    std::chrono::seconds connectTimeout = 10s;
    std::chrono::seconds readTimeout = 5s;
    std::vector<std::string> args;
};
```

当然, 这样还是不能让用户感知到新增变量..

那么就可以使用这种终极方法, 我不要构造函数了~!

### 2.2 使用`建造者模式`

```C++
struct Connection {
    int fd;

    explicit Connection(int fd_) : fd(fd_) {
    }

    Connection &read();
};

struct ConnectionBuilderBase {
    std::string serverAddress;
    int port;
    bool useSSH = false;
    std::string sshCertPath = "";
    std::string sshPKeyPath = "";
    std::string sshCAFilePath = "";
    std::string username = "admin";
    std::string password = "password";
    bool enableFastTCPOpen = true;
    int tlsVersion = 1;
    std::chrono::seconds connectTimeout = 10s;
    std::chrono::seconds readTimeout = 5s;
    std::vector<std::string> args;
};

struct ConnectionBuilder : ConnectionBuilderBase {
    ConnectionBuilder &withAddress(const std::string& addr) {
        serverAddress = addr;
        return *this;
    }

    ConnectionBuilder &withPort(int p) {
        port = p;
        return *this;
    }

    ConnectionBuilde &withAddressAndPort(const std::string& addr) {
        auto pos = addr.find(':');
        serverAddress = addr.substr(0, pos);
        port = std::stoi(addr.substr(pos + 1));
        return *this;
    }

    ConnectionBuilder &withSSH(const std::string& cert, const std::string& pkey, const std::string& caf = "asas") {
        useSSH = true;
        sshCertPath = cert;
        sshPKeyPath = pkey;
        sshCAFilePath = caf;
        return *this;
    }

    ConnectionBuilder &addArg(const std::string& arg) {
        args.push_back(arg);
        return *this;
    }

    Connection connect() {
        static_assert(Ready, "你必须指定 addr 参数！");
        int fd = 0;
        // fd = open(serverAddress, port);
        return Connection(fd);
    }
};

Connection c = ConnectionBuilder()
    .withSSH("1", "2") // 如果新增参数, 那么这里就可以报错
    .addArg("asas")
    .addArg("bsbs")
    .withAddressAndPort("localhost:8080")
    .addArg("baba")
    .connect();
```

### 2.3 为管理资源的更高效的Builder模式

举个例子, 对于要管理资源的类, 一般都是使用移动, 而不是拷贝.

其中:

- `[[nodiscard]]`表示如果返回后没有接收, 则编译器会警告 (C++17)

- `returnType Fun() && {}`的`&&`表示这个对象是以右值的形式调用的.(该成员函数只能对右值调用)

```C++
struct [[nodiscard]] Cake {
    int handle;

    Cake() {}

    [[nodiscard]] Cake &&setOrig() && {
        // 构造原味蛋糕
        handle = 0;
        return std::move(*this);
    }

    [[nodiscard]] Cake &&setChoco(double range) && {
        // 构造巧克力蛋糕
        handle = (int)range;
        return std::move(*this);
    }

    [[nodiscard]] Cake &&setMoca(int flavor) && {
        // 构造抹茶味蛋糕
        handle = flavor;
        return std::move(*this);
    }

    Cake(Cake &&) = default;
    Cake(Cake const &) = delete; // 删除拷贝
};

// 声明了两个重载函数, 移动会跟高效些.
void func(Cake &&c);
void func(Cake const &c);

Cake origCake = Cake().setOrig().setChoco(1.0);
Cake chocoCake = Cake().setChoco(1.0);
Cake matchaCake = Cake().setMoca(1);

int main() {
    Cake c;
    Cake().setOrig();
    func(std::move(c)); // 使用移动
}
```

综上例子, 就可以完善如下:

```C++
using namespace std::chrono_literals;

struct Connection {
    int fd;

    explicit Connection(int fd_) : fd(fd_) {
    }

    Connection &read();
};

struct ConnectionBuilderBase {
    std::string serverAddress;
    int port;
    bool useSSH = false;
    std::string sshCertPath = "";
    std::string sshPKeyPath = "";
    std::string sshCAFilePath = "";
    std::string username = "admin";
    std::string password = "password";
    bool enableFastTCPOpen = true;
    int tlsVersion = 1;
    std::chrono::seconds connectTimeout = 10s;
    std::chrono::seconds readTimeout = 5s;
    std::vector<std::string> args;
};

template <bool Ready = false>
struct [[nodiscard]] ConnectionBuilder : ConnectionBuilderBase {
    [[nodiscard]] ConnectionBuilder<true> &&withAddress(std::string addr) && {
        serverAddress = addr;
        return static_cast<ConnectionBuilder<true> &&>(static_cast<ConnectionBuilderBase &&>(*this));
    }

    [[nodiscard]] ConnectionBuilder &&withPort(int p) {
        port = p;
        return *this;
    }

    [[nodiscard]] ConnectionBuilder<true> &&withAddressAndPort(std::string addr) && {
        auto pos = addr.find(':');
        serverAddress = addr.substr(0, pos);
        port = std::stoi(addr.substr(pos + 1));
        return static_cast<ConnectionBuilder<true> &&>(static_cast<ConnectionBuilderBase &&>(*this));
    }

    [[nodiscard]] ConnectionBuilder &&withSSH(std::string cert, std::string pkey, std::string caf) && {
        useSSH = true;
        sshCertPath = cert;
        sshPKeyPath = pkey;
        sshCAFilePath = caf;
        return *this;
    }

    [[nodiscard]] ConnectionBuilder &&addArg(std::string arg) && {
        args.push_back(arg);
        return *this;
    }

    [[nodiscard]] Connection &&connect() && {
        static_assert(Ready, "你必须指定 addr 参数！"); // 通过不同的模版, 来保证某些东西的正确性, 于编译期且是建造者模式
        int fd = 0;
        // fd = open(serverAddress, port);
        return Connection(fd);
    }
};
```

## 三、虚函数到访问者模式
### 3.1 案例
现在有以下代码, 有的可以吃有的可以喝还有的都可以:

```C++
#include <cstdio>

struct EatParams {
    int amount;
    int speed;
};

struct DrinkParams {
    int volume;
    int temperature;
};

struct Food {
    virtual void eat(EatParams eatParams) = 0;
    virtual void drink(DrinkParams drinkParams) = 0;
};

struct Cake : Food {
    void eat(EatParams eatParams) override {
        // 吃蛋糕
    }

    void drink(DrinkParams drinkParams) override {
        // 不能喝蛋糕
    }
};

struct Milk : Food {
    void eat(EatParams eatParams) override {
        // 不能吃牛奶
    }

    void drink(DrinkParams drinkParams) override {
        // 喝牛奶
    }
};

struct Pudding : Food {
    void eat(EatParams eatParams) override {
        // 吃布丁
    }

    void drink(DrinkParams drinkParams) override {
        // 喝布丁
    }
};

void dailyRun(Food& food) {
    food.eat({1, 2});
    food.drink({3, 4});
}

int main() {
    Cake cake;
    Milk milk;
    Pudding pudding;
    dailyRun(cake);
    dailyRun(milk);
    dailyRun(pudding);
    return 0;
}
```

这样不是很麻烦吗? 如果我以后要增加一个功能叫做`lay(拉)`给Food(接口(纯虚类)), 那么 Cake, Milk, Pudding要不要支持呢?

如果支持的话不就违背了[[SOLID]开闭原则](../../../../../001-计佬常識/002-设计模式/005-面向对象设计原则/002-设计原则/003-【SOLID】开闭原则/index.md)了吗? 添加新的代码, 不需要修改之前的代码啊 (对拓展开放, 对修改封闭)~

### 3.2 重新抽象

正确的应该将他们抽象成这样: 而不是笼统的一个Food~~, 松弟你要这样乱抽象怎么不写成是所有东西都继承`东西类`呢?~~

```C++
struct Eatble {
    virtual void eat(EatParams eatParams) = 0;
};

struct Drinkble {
    virtual void drink(DrinkParams drinkParams) = 0;
};

struct Cake : Eatble {
    void eat(EatParams eatParams) override {
        // 吃蛋糕
    }
};

struct Milk : Drinkble {
    void drink(DrinkParams drinkParams) override {
        // 喝牛奶
    }
};

struct Pudding : Eatble, Drinkble { // 不是不能多继承, 接口的多继承是允许的
                                    // 就连Java都可以接口的多继承,
                                    // 不允许的是带有成员变量的多继承, 因为这样就分不清同名变量了, (以及菱形继承, 就要使用虚继承了..)
    void eat(EatParams eatParams) override {
        // 吃布丁
    }

    void drink(DrinkParams drinkParams) override {
        // 喝布丁
    }
};

// 使用重载决定使用哪个类(功能)
void dailyRun(Eatble& food) {
    food.eat({1, 2});
}

void dailyRun(Drinkble& food) {
    food.drink({3, 4});
}

int main() {
    Cake cake;
    Milk milk;
    Pudding pudding;
    dailyRun(cake);
    dailyRun(milk);
    dailyRun(static_cast<Eatble &>(pudding));
    dailyRun(static_cast<Drinkble &>(pudding));
    return 0;
}
```

如果你不想使用上面这种, 重载调用的话, 就希望一起调用, 也可以这样:

```C++
struct Food {
    virtual ~Food() = default; // 如果没有虚函数就不会生成一个typeid来动态类型查找
    // 所以声明一个虚析构来使得它变成一个接口类
};


struct Eatble : virtual Food { // 接口的继承最好使用虚继承
    virtual void eat(EatParams eatParams) = 0;
};

struct Drinkble : virtual Food {
    virtual void drink(DrinkParams drinkParams) = 0;
};

struct Cake : Eatble {
    void eat(EatParams eatParams) override {
        // 吃蛋糕
    }
};

struct Milk : Drinkble {
    void drink(DrinkParams drinkParams) override {
        // 喝牛奶
    }
};

struct Pudding : Eatble, Drinkble {
    void eat(EatParams eatParams) override {
        // 吃布丁
    }

    void drink(DrinkParams drinkParams) override {
        // 喝布丁
    }
};

void dailyRun(Food* food) {
    if (auto eat = dynamic_cast<Eatble *>(food); eat) { // dynamic_cast 转化类型识别返回 0 或者叫做 (void *)0
        eat->eat({1, 2});
    }
    // 如果只是判断布尔值, 那么可以省略后面的 变量, 即 if (auto it = getIt(); it) 的后面的 it, 可以省略
    if (auto drink = dynamic_cast<Drinkble *>(food)) { // <-- 像这样
        drink->drink({3, 4});
    }
}

int main() {
    Cake cake;
    Milk milk;
    Pudding pudding;
    dailyRun(&cake);
    dailyRun(&milk);
    dailyRun(&pudding);
    return 0;
}
```

这里可以稍微说明一下`dynamic_cast`的原理:

```C++
// 前向声明
struct Eatble;
struct Drinkble;

struct Food {
    virtual ~Food() = default;

    virtual Eatble *toEatble() {
        return nullptr;
    }

    virtual Drinkble *toDrinkble() {
        return nullptr;
    }
};

struct Eatble : virtual Food {
    virtual void eat(EatParams eatParams) = 0;

    virtual Eatble *toEatble() override {
        return this;
    }
};

struct Drinkble : virtual Food {
    virtual void drink(DrinkParams drinkParams) = 0;

    virtual Drinkble *toDrinkble() override {
        return this;
    }
};

void dailyRun(Food* food) {
    if (auto eat = food->toEatble(); eat) {
        eat->eat({1, 2});
    }
    if (auto drink = food->toDrinkble()) {
        drink->drink({3, 4});
    }
}
```

当然上面的还是违背了`开闭原则`的, 因为新增功能, 你就可能要修改`dailyRun`并增加一个`if`判断...

### 3.3 [访问者模式](../../../../../001-计佬常識/002-设计模式/009-行为型模式/011-访问者模式/index.md)

```C++
#include <print>

struct EatParams {
    int amount;
    int speed;
};

struct DrinkParams {
    int volume;
    int temperature;
};

// !================================================!
// 声明一个食物访问者基类
struct FoodVisitor {
    // 前向声明, 但可以这样写
    virtual void visit(struct /* <-- */ Eatable *eat) {}
    virtual void visit(struct Drinkable *drink) {} // 这些是空实现, 让需要这个功能的子类覆盖
    virtual ~FoodVisitor() = default;
};

// !================================================!

struct Food {
    // !================================================!
    virtual void accept(FoodVisitor *visitor) = 0; // 传入食物访问者给食物 (倒反天罡)
    // !================================================!
    virtual ~Food() = default;
};

// !================================================!
// 一个宏
#define DEF_FOOD_ACCEPT \
void accept(FoodVisitor *visitor) override { \
    visitor->visit(this); \ // 传入的this是Food及其子类, 
}                           // 然后让访问者重载, goto PengUser; (具体访问者)
// !================================================!

struct Drinkable : virtual Food {
    virtual void drink(DrinkParams drinkParams) = 0;

    // 让'喝'来访问'食物'
    DEF_FOOD_ACCEPT
};

struct Eatable : virtual Food {
    virtual void eat(EatParams eatParams) = 0;

    // 让'吃'来访问'食物'
    DEF_FOOD_ACCEPT
};

struct Cake : Eatable {
    void eat(EatParams eatParams) override {
        std::println("Eating cake...");
        std::println("Amount: {}", eatParams.amount);
        std::println("Speed: {}", eatParams.speed);
    }
};

struct Milk : Drinkable {
    void drink(DrinkParams drinkParams) override {
        std::println("Drinking milk...");
        std::println("Volume: {}", drinkParams.volume);
        std::println("Temperature: {}", drinkParams.temperature);
    }
};

struct Pudding : Eatable, Drinkable {
    void eat(EatParams eatParams) override {
        std::println("Eating pudding...");
        std::println("Amount: {}", eatParams.amount);
        std::println("Speed: {}", eatParams.speed);
    }

    void drink(DrinkParams drinkParams) override {
        std::println("Drinking pudding...");
        std::println("Volume: {}", drinkParams.volume);
        std::println("Temperature: {}", drinkParams.temperature);
    }

    // 既要吃 又要喝
    void accept(FoodVisitor *visitor) override {
        Eatable::accept(visitor);
        Drinkable::accept(visitor);
    }
};

// !================================================!
// 具体访问者的实现
struct PengUser : FoodVisitor {
    void visit(Eatable *eat) override {
        eat->eat({5, 10});
    }

    void visit(Drinkable *drink) override {
        drink->drink({10, 20});
    }
};
// !================================================!

void pengEat(Food *food) { // 吃三次
    PengUser user;
    food->accept(&user);
    food->accept(&user);
    food->accept(&user);
}

int main() {
    Cake cake;
    Milk milk;
    Pudding pudding;
    pengEat(&cake);
    pengEat(&milk);
    pengEat(&pudding);
    return 0;
}
```

所以, 你在QT啊, 什么的地方, 要求你定义一个宏, 可能原理就是这样..