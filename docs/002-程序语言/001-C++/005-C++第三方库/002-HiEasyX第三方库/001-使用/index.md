# HiEasyX
## github仓库
[HiEasyX](https://github.com/zouhuidong/HiEasyX)

## 需求
我要做一个游戏

## 可能需要的函数

### win32

```C++
// ScreenToClient 函数将屏幕上指定点的屏幕坐标转换为工作区坐标
BOOL ScreenToClient(
  [in] HWND    hWnd,
       LPPOINT lpPoint // 该结构指定要转换的屏幕坐标
);
```


### HiFunc.h
```C++

/**
 * @brief 设置窗口透明度
 * @param[in] HWnd 窗口句柄
 * @param[in] enable 是否启用窗口透明度
 * @param[in] alpha 窗口透明度值 0-255
*/
void SetWindowTransparent(HWND HWnd, bool enable, int alpha = 0xFF);
```

### HiWindows.h
```C++
/**
 * @brief 在创建窗口前设置窗口位置，仅对此操作后首个新窗口生效
 * @param[in] x    位置
 * @param[in] y    位置
*/
void PreSetWindowPos(int x, int y);

/**
 * @brief 获取窗口位置
 * @param[in] hWnd 窗口句柄（为空代表当前活动窗口）
 * @return 窗口位置
*/
POINT GetWindowPos(HWND hWnd = nullptr);

/**
 * @brief 移动窗口
 * @param[in] x        位置
 * @param[in] y        位置
 * @param[in] hWnd    窗口句柄（为空代表当前活动窗口）
*/
void MoveWindow(int x, int y, HWND hWnd = nullptr);

/**
 * @brief 相对移动窗口
 * @param[in] dx    相对位移
 * @param[in] dy    相对位移
 * @param[in] hWnd    窗口句柄（为空代表当前活动窗口）
*/
void MoveWindowRel(int dx, int dy, HWND hWnd = nullptr);

/**
 * @brief 重设窗口大小
 * @param[in] w        窗口宽
 * @param[in] h        窗口高
 * @param[in] hWnd    窗口句柄（为空代表当前活动窗口）
*/
void ResizeWindow(int w, int h, HWND hWnd = nullptr);

/**
 * @brief 设置窗口标题文本
 * @param[in] lpszTitle        新的窗口标题
 * @param[in] hWnd            窗口句柄（为空代表当前活动窗口）
*/
void SetWindowTitle(LPCTSTR lpszTitle, HWND hWnd = nullptr);
```

### 获取屏幕分辨率

```C++
HWND hwd = ::GetDesktopWindow();
HDC hdc = ::GetDC(hwd);
int width = GetDeviceCaps(hdc, DESKTOPHORZRES);
int height = GetDeviceCaps(hdc, DESKTOPVERTRES);
auto data = GetScreenSize();
printf("显示器信息: %d x %d, (%d, %d)是左上角\n", data.w, data.h, data.left, data.top); // 系统的
printf("应该是 %d x %d\n", width, height);// 硬件的 
```

# 功能探究
## 如何获取全部窗口的消息?

可是得到的是相对坐标呀az

```C++
hiex::Window wnd(300, 200);
hiex::Window wnd2(300, 200);
HX::threadPool::ThreadPool pool(2);
ExMessage msg = { 0 };
pool.addTask([&] {
    while (1)
    {
        peekmessage(&msg, 255U, 1, wnd.GetHandle());
        peekmessage(&msg, 255U, 1, wnd2.GetHandle());
    }
    });

while (true)
{
    if (msg.message == WM_CHAR)    // 均为 0 // 原本 if(!(HXMessage::msg.x || HXMessage::msg.y))
        printf("按键\n");
    else if (msg.message == WM_KEYDOWN) // 注意判断的内容是不同的!!! WM_KEYDOWN 与 EX_KEY
        printf("写字\n");

    switch (msg.message)
    {
    case WM_MOUSEMOVE:        // 鼠标移动消息
        printf("%d %d\n", msg.x, msg.y);
        break;

    default:
        break;
    }

    static hiex::tDelayFPS recond;
    hiex::DelayFPS(recond, 24);
}
```

好像可以搞一个全透明的窗口, 然后置于最顶层, 用于获取信息

```C++
// 这个是win32的API
SetWindowPos(wndAll.GetHandle(), HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE); // 其置顶的效果类似于 截屏软件 和 歌词的一样 点击誰誰就是最顶层
```

一个可用的最优的方式
```C++
hiex::Window wndAll(data.w, data.h);

HWND hwnd = wndAll.GetHandle(); // 替换为你的窗口句柄

// 移除窗口的缩放框架和最大化按钮、将窗口置于最顶层、禁止拖动和缩放、移除标题栏并最大化窗口
SetWindowLongPtr(hwnd, GWL_STYLE, GetWindowLongPtr(hwnd, GWL_STYLE) & ~(WS_CAPTION | WS_THICKFRAME | WS_MAXIMIZEBOX | WS_SIZEBOX) & ~WS_EX_DLGMODALFRAME);
SetWindowPos(hwnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE);
SetWindowTransparent(hwnd, 1, 60);
```

还有, 可以使用钩子, 但是有恶意软件的风险, =-=, 得了 消息可能还要自己封装，算了

## 八嘎
### 屏幕抖动问题
> 这个确实挺影响的

每次改变窗口大小的时候, 都有可能使得其绘制的内容发生抖动