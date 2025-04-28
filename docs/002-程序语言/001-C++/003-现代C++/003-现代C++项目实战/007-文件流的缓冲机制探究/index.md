# 文件流的缓冲机制探究
> 小彭老师指出，stdout和cout中输出内容，并不会立即刷新在屏幕上，而是先进入他们内部的缓冲区。等到用户输出了换行符'\n'，才会调用操作系统API，一次性写入之前积攒的所有字符，清空缓冲区。这种遇到换行符才刷新的缓冲机制，被称为行缓冲(IOLBF)，是出于效率的考虑。如果你需要输出一个不以'\n'结束的字符串，并且立即显示在屏幕上的话，需要fflush(stdout)或者cout << flush。奇怪的是，MSVC似乎没有支持行缓冲，这或许就是为什么MSVC上通过putchar逐个会非常慢，需要先形成一整个字符串后一次性打印，才会快一些的原因。
> 
> 与之相反地，stderr和cerr，禁用了缓冲机制，每次输出内容，都会强制flush，不会缓存起来，这就称为无缓冲(IONBF)。cerr更适合输出错误信息，这样即使程序事后崩溃，也不会有东西积压在缓冲里没及时打印。
>
> 而普通的文件，通过fopen或fostream打开后，就是处于全缓冲(IOFBF)状态，任何输入的字符都会被缓冲起来，直到缓冲区容纳不下了，或者文件被关闭，才会刷新，写入磁盘文件。
>
> 另外，cout不推荐使用cout << endl的原因就是，endl = '\n' + flush，而cout本身就已经是行缓冲的，遇'\n'会自动刷新，并不需要额外一次flush。除非你通过cout.sync_with_stdio(false)让cout进入全缓冲模式，或者你写入一个文件流，并且希望立即同步，那才需要flush。总之，endl是一个历史遗留产物，正常情况下cout << msg << '\n'即可，没必要用endl。
>
> 最后，我们还运用C++虚函数，自己实现了一个简易的流类型，内置一个页对齐的缓冲区，支持行缓冲，无缓冲，全缓冲三种工作模式。还禁用了stdin的CANON和ECHO选项，从而实现密码输入框（输入时显示****，支持退格）。
>
> By: [【C/C++】printf后并不会立即显示？文件流的缓冲机制探究](https://www.bilibili.com/video/BV1r1421r7Ym/)

## 1. 认识缓冲流
### 1.1 从现象切入
考虑以下代码:
```C++
#include <iostream>
#include <thread>

using namespace std;

int main() {
    cout << "Hello,";
    std::this_thread::sleep_for(1s);
    cout << "World\n";
    std::this_thread::sleep_for(1s);
    cout << "Exiting\n";
    return 0;
}
```

如果你在Linux上, 那么看到的输出应该是:

```C++
// 0s 时候

// 1s 时候
Hello,World\n

// 2s 时候
Hello,World\n
Exiting\n
```

而如果你是windows, 则是:

```C++
// 0s 时候
Hello,

// 1s 时候
Hello,World\n

// 2s 时候
Hello,World\n
Exiting\n
```

这是为什么呢?

### 1.2 如何输出?
首先我们要知道, 程序是如何将内容输出到控制台上的:

```C++
printf("loli\n");
std::cout << "kawaii" << '\n';
```

实际上C/C++程序在启动的时候, 默认就会打开三个文件, 他们的`fd`分别是:

```C++
/* Standard streams.  */
extern FILE *stdin;        /* Standard input stream.  */
extern FILE *stdout;        /* Standard output stream.  */
extern FILE *stderr;        /* Standard error output stream.  */
/* C89/C99 say they're macros.  Make them happy.  */
#define stdin stdin
#define stdout stdout
#define stderr stderr

/* Standard file descriptors.  */
#define    STDIN_FILENO    0    /* Standard input.  */
#define    STDOUT_FILENO    1    /* Standard output.  */
#define    STDERR_FILENO    2    /* Standard error output.  */
```

代表着`输入流`, `输出流`和`错误输出流`.

因此, 有下列等式:
```C++
printf("Loli");
// 上下等价
fprintf(stdout, "Loli");
```

### 1.3 不同流的策略
学过C语言的文件操作的同学都知道, 如果我们需要立即写入到文件, 则需要调用`fflush`或者`fclose`. 否则即便写入了, 你打开文件也是看不到的...(同理, 此时只是写入到了缓冲区)

输入/输出流也是文件, 同理也是需要这样操作.

```C++
// Linux:
// BUFSIZ 8192
// stdout: _IOLBF 行缓冲
// stderr: _IONBF 无缓冲
// fopen: _IOFBF 完全缓冲

// std::cout _IOLBF 行缓冲
// std::cerr _IONBF 无缓冲
// std::fstream/std::clog _IOFBF 完全缓冲

// MSVC (windows):
// BUFSIZ 512
// stdout: _IONBF 无缓冲
// stderr: _IONBF 无缓冲
// fopen: _IOFBF 完全缓冲

// C 语言标准要求 stdout 可以行缓冲, stderr 必须无缓冲

// Linux 中, sync false 会导致 cout 变为完全缓冲
// std::ios::sync_with_stdio(false); // (因为这样就不和cstdio的绑定了)
```

此处的`行缓冲`, 就是遇到`\n`, 才会写入文件.

而`无缓冲`就是有内容就立即写入.

最后的`完全缓冲`, 就是只有在 缓冲区满了 或者 调用`fflush`/`fclose`时候才会写入.

因此这也解释了为什么windows上面使用`printf`会这么慢.

> 因为每次printf都要进出内核态(600ms就没了!)

同时也解释了`std::cin`为什么不推荐使用`std::endl`

- 因为`std::endl`等价于:

  ```C++
  std::cin << '\n';
  std::cout << std::flush;
  ```

    而无论win还是linux, 都是遇到`\n`就会刷新, `std::cout.flush()`完全就是多余. <sup>[[除非某些特殊情况](https://www.apiref.com/cpp-zh/cpp/io/manip/endl.html)]</sup>

## 2. 玩转缓冲区
### 2.0 不要当八嘎

Linux上, 如果你写了以下代码:

```C++
#include <cstdio>

int main() {
    printf("初始化 i\n");
    int i = 0;
    printf("对 i 进行操作: ");
    // 假装有很复杂的代码 ...
    *(int *)1 = 0; // 段错误
    printf("啊啊啊~\n");
    return 0;
}
```

然后运行时候输出为:

```C++
初始化 i\n

// 段错误
```

然后你就很苦恼, 为什么`int i = 0;`会报错? 怎么不输出`对 i 进行操作:`?

> 如果你现在百思不得其解, 那么你就是八嘎! 八嘎!

### 2.1 自定义缓冲区大小

```C++
#include <cstdio>
#include <thread>

using namespace std;

static char buf[BUFSIZ];

int main() {
    setvbuf(stdout, buf, _IOFBF, sizeof buf);

    for (int i = 0; i < 65536; i += 8) {
        fprintf(stdout, "[%5d]\n", i);
        this_thread::sleep_for(1ms);
    }

    return 0;
}
```

> What is [setvbuf](https://www.runoob.com/cprogramming/c-function-setvbuf.html)

指定`buf`为缓冲区, 缓冲策略为`_IOFBF`(完全缓冲)

故代上述码, 当且仅当缓冲区满了才会输出内容到控制台.

### 2.2 手搓输入流
#### 2.2.1 操作系统的历史遗留问题

为什么我们输入的时候有显示, 使用退格也可以正常删除, 而不是显示`^?^?^?`呢?, 并且必须要按回车才是输入?

甚至不是因为行缓冲:

```C++
setvbuf(stdin, nullptr,_IONBF, 0);
```

这个确实是操作系统(Linux/win都是)的问题, 需要:

```C++
#include <termios.h>

int _fuckStdin_ = []() {
    saved = false;
    if (isatty(STDIN_FILENO)) {
        struct termios tc;
        tcgetattr(STDIN_FILENO, &tc);
        memcpy(&oldtc, &tc, sizeof(tc));
        saved = true;
        tc.c_lflag &= ~ICANON;
        tc.c_lflag &= ~ECHO;
        tcsetattr(STDIN_FILENO, TCSANOW, &tc);
    }
    return 0;
}();
```

其中

- `tc.c_lflag &= ~ICANON;`
    - 关闭终端的 **规范模式 (canonical mode)**，也就是说输入不会被行缓冲，也不需要按下`Enter`键就能获取输入字符。

- `tc.c_lflag &= ~ECHO;`
    - 关闭 **回显模式 (echo mode)**，也就是输入的字符不会显示在终端上。

#### 2.2.2 模拟输入密码

将输入回显为`*`, 退格正常, 代码如下:

```C++
struct StdinRawify {
    struct termios oldtc;
    bool saved;

    StdinRawify() {
        saved = false;
        if (isatty(STDIN_FILENO)) {
            struct termios tc;
            tcgetattr(STDIN_FILENO, &tc);
            memcpy(&oldtc, &tc, sizeof(tc));
            saved = true;
            tc.c_lflag &= ~ICANON;
            tc.c_lflag &= ~ECHO;
            tcsetattr(STDIN_FILENO, TCSANOW, &tc);
        }
    }

    ~StdinRawify() { // 需要设置回去! 不然你的控制台就变成密码台了!
        if (saved) {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldtc);
        }
    }
};

std::string input_password(const char *prompt, size_t max_size = static_cast<size_t>(-1)) {
    if (prompt) {
        fprintf(stderr, "%s", prompt);
    }
    std::string ret;
    StdinRawify stdinRawifier;
    while (true) {
        int c = getchar();
        if (c == EOF)
            break;
        if (c == '\n' || c == '\r') {
            fputc('\n', stderr);
            break;
        } else if (c == '\b' || c == '\x7f') {
            if (ret.size() > 0) {
                ret.pop_back();
                fprintf(stderr, "\b \b");
            }
        } else {
            if (ret.size() < max_size) {
                ret.push_back(c);
                fputc('*', stderr);
            }
        }
    }
    return ret;
}

int main() {
    auto passwd = input_password("请输入密码：");
    // setvbuf(stdin, nullptr, _IONBF, 0);
    fprintf(stderr, "输入的密码是：%s\n", passwd.c_str());
    return 0;
}
```


#### 2.2.2 手搓输入流
思路:

- 使用`::write` + 设计模式, 自己搞一个

### 2.3 手搓输出流

思路:

- 使用`::write` + 设计模式, 自己搞一个