# 三维模型的加载与相机控制
## 一、回顾三角形大作
### 1.1 三角形大作存在的问题

之前(第一章)的三角形大作, 顶点信息是写死在代码上的, 如果我们要新增一个点, 就需要重新调用一遍 `glVertex3f`, 非常的不方便! 而且还要重新编译!

```cpp
glBegin(GL_TRIANGLES);
glVertex3f(0.0f, 0.5f, 0.0f);
glVertex3f(-0.5f, -0.5f, 0.0f);
glVertex3f(0.5f, -0.5f, 0.0f);
glEnd();
```

> 如何另数据与程序解耦呢?

### 1.2 动态存储

我们可以把顶点信息存储在 `std::vector` 中, 这样绘制的时候, 就支持动态数量个顶点啦:

```cpp [c12-顶点存储]
struct Vertex {
    float x, y, z;
};

std::vector<Vertex> vertices;
```

```cpp [c12-顶点绘制]
glBegin(GL_TRIANGLES);
for (auto const& v : vertices)
    glVertex3f(v.x, v.y, v.z);
glEnd();
```

这样我们可以实现点击屏幕, 选择3个点, 然后画三角形:

```cpp
#include <check/OpenGL.hpp> // 包括 glad/glad.h
#include <GLFW/glfw3.h>     // 必须放在 glad/glad.h 后面

#include <vector>

struct Vertex {
    float x, y, z;
};

std::vector<Vertex> vertices;

void show() {
    glBegin(GL_TRIANGLES);
    for (auto const& v : vertices)
        glVertex3f(v.x, v.y, v.z);
    CHECK_GL(glEnd());
}

void mouseBtnCb(GLFWwindow* win, int btn, int action, [[maybe_unused]] int mods) {
    // 判断是鼠标 && 左键按下
    if (btn == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        double xpos, ypos;
        int width, height;
        // 获取鼠标坐标
        glfwGetCursorPos(win, &xpos, &ypos);
        // 获取窗口大小
        glfwGetWindowSize(win,  &width, &height);

        // 映射到 [-1, 1], 除以对应的宽、高, 以保证宽高比
        auto x = static_cast<float>(2 * xpos / width - 1);
        // 注意因为窗口坐标是左上原点, y 正轴向下, 所以 转换需要 height - ypos
        auto y = static_cast<float>(2 * (height - ypos) / height - 1);
        // 比如 xpos / width 是把 [0, width] -> [0, 1]
        // *2 - 1 是把 [0, 1] -> [-1, 1]
        vertices.push_back({x, y, 0});
    }
}

int main() {
    if (!glfwInit()) {
        throw std::runtime_error("failed to initialize GLFW");
    }
    auto* window = glfwCreateWindow(960, 720, "Example", NULL, NULL);
    if (!window) {
        glfwTerminate();
        throw std::runtime_error("GLFW failed to create window");
    }
    glfwMakeContextCurrent(window);
    if (!gladLoadGL()) {
        glfwTerminate();  // 由于 glfwInit 在前, 理论上是需要配套的 glfwTerminate 防止泄漏
        throw std::runtime_error("GLAD failed to load GL functions");
    }
    log::hxLog.debug("OpenGL version: ", glGetString(GL_VERSION)); // 初始化完毕, 打印一下版本号

    CHECK_GL(glEnable(GL_POINT_SMOOTH));
    CHECK_GL(glEnable(GL_BLEND));
    CHECK_GL(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
    CHECK_GL(glPointSize(64.0f));

    glfwSetMouseButtonCallback(window, mouseBtnCb); // 设置鼠标事件回调
    while (!glfwWindowShouldClose(window)) {
        show();
        glfwSwapBuffers(window); // 双缓冲
        glfwPollEvents();
    }
    return 0;
}
```

### 1.3 由点到面: 从顶点数组到任意三角形面的数组

```cpp [c13-三角形数组]
struct Vertex {
    float x, y, z;
};

struct Triangle {
    Vertex a, b, c;
};

// 两个三角形组成的正方形
std::vector<Triangle> triangles = {
    {
        {0.5, 0.5, 0},
        {0.5, -0.5, 0},
        {-0.5, 0.5, 0},
    },
    {
        {-0.5, -0.5, 0},
        {-0.5, 0.5, 0},
        {0.5, -0.5, 0},
    },
};
```

```cpp [c13-变动其中一个点]
#include <check/OpenGL.hpp> // 包括 glad/glad.h

#include <vector>

struct Vertex {
    float x, y, z;
};

struct Triangle {
    Vertex a, b, c;
};

std::vector<Triangle> triangles = {
    {
        {0.5, 0.5, 0},
        {0.5, -0.5, 0},
        {-0.5, 0.5, 0},
    },
    {
        {-0.5, -0.5, 0},
        {-0.5, 0.5, 0},
        {0.5, -0.5, 0},
    },
};

void show() {
    glBegin(GL_TRIANGLES);
    for (auto const& v : triangles) {
        glVertex3f(v.a.x, v.a.y, v.a.z);
        glVertex3f(v.b.x, v.b.y, v.b.z);
        glVertex3f(v.c.x, v.c.y, v.c.z);
    }
    CHECK_GL(glEnd());
}

void mouseBtnCb(GLFWwindow* win, int btn, int action, [[maybe_unused]] int mods) {
    // 判断是鼠标 && 左键按下
    if (btn == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        double xpos, ypos;
        int width, height;
        glfwGetCursorPos(win, &xpos, &ypos);
        glfwGetWindowSize(win,  &width, &height);

        auto x = static_cast<float>(2 * xpos / width - 1);
        auto y = static_cast<float>(2 * (height - ypos) / height - 1);
        triangles[0].c = {x, y, 0}; // 重新设置其中一个点
    }
}

int main() {
    auto* window = initOpenGL();
    log::hxLog.debug("OpenGL version: ", glGetString(GL_VERSION));

    CHECK_GL(glEnable(GL_POINT_SMOOTH));
    CHECK_GL(glEnable(GL_BLEND));
    CHECK_GL(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
    CHECK_GL(glPointSize(64.0f));

    glfwSetMouseButtonCallback(window, mouseBtnCb); // 设置鼠标事件回调
    while (!glfwWindowShouldClose(window)) {
        CHECK_GL(glClear(GL_COLOR_BUFFER_BIT)); // 清空画布
        show();
        glfwSwapBuffers(window); // 双缓冲
        glfwPollEvents();
    }
    return 0;
}
```

我们变动其中一个点, 发现:

![变动点 ##w400##](HX_2025-07-31_14-15-00.png)

正方形直接裂开了!, 这可不对, 我们期望的是这两个三角形的顶点都一起动~

### 1.4 优化: 不存储点坐标, 存储点索引

我们希望修改一个点, 然后所有人看到的这个点都修改了. 基于`引用`的思想.

我们可以分别存储`点和三角形`, 三角形的顶点是点数组的 **索引**. 这样改一个点数组的元素, 其关联的三角形的所有点都跟着变啦~

```cpp [c14-数据结构]
struct Vertex {
    float x, y, z;
};

std::vector<Vertex> vertices = {
    {0.5, 0.5, 0},
    {0.5, -0.5, 0},
    {-0.5, 0.5, 0},
    {-0.5, -0.5, 0},
};

struct Triangle {
    std::size_t a, b, c;
};

std::vector<Triangle> triangles = {
    {0, 1, 2},
    {3, 2, 1},
};
```

```cpp [c14-完整代码]
#include <check/OpenGL.hpp> // 包括 glad/glad.h

#include <vector>

struct Vertex {
    float x, y, z;
};

std::vector<Vertex> vertices = {
    {0.5, 0.5, 0},
    {0.5, -0.5, 0},
    {-0.5, 0.5, 0},
    {-0.5, -0.5, 0},
};

struct Triangle {
    std::size_t a, b, c;
};

std::vector<Triangle> triangles = {
    {0, 1, 2},
    {3, 2, 1},
};

void show() {
    glBegin(GL_TRIANGLES);
    for (auto const& v : triangles) {
        glVertex3f(vertices[v.a].x, vertices[v.a].y, vertices[v.a].z);
        glVertex3f(vertices[v.b].x, vertices[v.b].y, vertices[v.b].z);
        glVertex3f(vertices[v.c].x, vertices[v.c].y, vertices[v.c].z);
    }
    CHECK_GL(glEnd());
}

void mouseBtnCb(GLFWwindow* win, int btn, int action, [[maybe_unused]] int mods) {
    // 判断是鼠标 && 左键按下
    if (btn == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        double xpos, ypos;
        int width, height;
        glfwGetCursorPos(win, &xpos, &ypos);
        glfwGetWindowSize(win,  &width, &height);

        auto x = static_cast<float>(2 * xpos / width - 1);
        auto y = static_cast<float>(2 * (height - ypos) / height - 1);
        vertices[2] = {x, y, 0};
    }
}

int main() {
    auto* window = initOpenGL();
    log::hxLog.debug("OpenGL version: ", glGetString(GL_VERSION));

    CHECK_GL(glEnable(GL_POINT_SMOOTH));
    CHECK_GL(glEnable(GL_BLEND));
    CHECK_GL(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
    CHECK_GL(glPointSize(64.0f));

    glfwSetMouseButtonCallback(window, mouseBtnCb); // 设置鼠标事件回调
    while (!glfwWindowShouldClose(window)) {
        CHECK_GL(glClear(GL_COLOR_BUFFER_BIT)); // 清空画布
        show();
        glfwSwapBuffers(window); // 双缓冲
        glfwPollEvents();
    }
    return 0;
}
```

```cpp [c14-小偷懒]
void show() {
    glBegin(GL_TRIANGLES);
    for (auto const& v : triangles) {
        // 从三个参数的 glVertex3f 改成一个指针参数的 glVertex3fv. 少写点代码
        // glVertex3fv 接受一个 float * 指针作为参数, 会加载指针所指向的内存中连续的三个 float
        glVertex3fv(&vertices[v.a].x);
        glVertex3fv(&vertices[v.b].x);
        glVertex3fv(&vertices[v.c].x);
    }
    CHECK_GL(glEnd());
}
```

效果:

![新版本效果 ##w400##](HX_2025-07-31_14-25-23.png)

> [!NOTE]
> "不是保留顶点本身而是保留顶点的索引"--保留了索引, 就维持住了网格的 **拓扑结构**, 不同面的相邻顶点之间就不是一盘散沙, 而是紧密连接在一起~

## 二、OBJ 格式三维模型的加载与绘制
### 2.1 OBJ 是什么?

**OBJ 是一种基于纯文本的 3D 模型存储格式**

> OBJ是一种基于纯文本的3D模型存储格式, 用于存储3D模型的点、面、法线等数据。它的优点是它是一种可读性好的格式, 可以直接被文本编辑器打开和编辑, 也被多种3D软件所支持读取和写入。OBJ格式还支持引用外部MTL文件, 从而支持多种材质和纹理映射。他的文件格式相对简单, 容易读取和导出, 即使是 C 语言初学者也很容易写出他的读取器和导出器。
>
> 然而, OBJ格式的缺点是它不适合存储复杂的3D数据, 因为纯文本格式不能以最小化的方式表示这些数据, 这使得OBJ格式不适合存储需要高精度和超高面数的3D模型。此外, OBJ格式对于大型3D模型的存储也会变得非常慢和低效, 因为它需要读取和写入大量的文本数据。

### 2.2 认识 OBJ 文件格式: 顶点数据

> OBJ 文件包含很多行, 每一行都包含一个 **指令**。指令由英文字母开始, 后面跟着几个数字或字符串作为数据, 典型的是 `v 指令`, 他代表一个顶点, 后面跟着的数据是用空格分割的三个浮点数, 分别代表着顶点的 **x、y、z 坐标**。此外还有 o 指令用于指定当前对象名称, 以 # 开头行的是可选的注释, 注释不会被读取和解析。

```sh
# Blender 3.5.0
# www.blender.org
o Cube
v 1.000000 1.000000 -1.000000
v 1.000000 -1.000000 -1.000000
v 1.000000 1.000000 1.000000
```

### 2.3 顶点解析器

```cpp [c23-顶点解析器]
struct Vertex {
    float x, y, z;
};

struct Triangle {
    std::size_t a, b, c;
};

struct ObjParser {
    void parser(std::string_view path) {
        std::ifstream file{{path.data(), path.size()}};
        if (!file.is_open()) [[unlikely]] {
            log::hxLog.error("打开文件:", path, "失败!");
            throw std::runtime_error{path.data()};
        }
        std::string line;
        while (std::getline(file, line)) {
            if (line.substr(0, 2) == "v ") {
                std::istringstream s{line.substr(2)};
                Vertex v;
                s >> v.x >> v.y >> v.z;
                _vertices.push_back(std::move(v));
            }
        }

        file.close();
        log::hxLog.info("加载:", path, "完成!");
    }

private:
    std::vector<Vertex> _vertices;
};
```

```cpp [c23-完整代码]
#include <check/OpenGL.hpp>

#include <fstream>
#include <stdexcept>
#include <vector>

auto __init__ = []{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    try {
        auto cwd = std::filesystem::current_path();
        log::hxLog.debug("当前工作路径是:", cwd);
        std::filesystem::current_path("../../../src/08-OpenGL");
        log::hxLog.debug("切换到路径:", std::filesystem::current_path());
    } catch (const std::filesystem::filesystem_error& e) {
        log::hxLog.error("Error:", e.what());
    }
    return 0;
}();

namespace HX {

struct Vertex {
    float x, y, z;
};

struct Triangle {
    std::size_t a, b, c;
};

struct ObjParser {
    void parser(std::string_view path) {
        std::ifstream file{{path.data(), path.size()}};
        if (!file.is_open()) [[unlikely]] {
            log::hxLog.error("打开文件:", path, "失败!");
            throw std::runtime_error{path.data()};
        }
        std::string line;
        while (std::getline(file, line)) {
            if (line.substr(0, 2) == "v ") {
                std::istringstream s{line.substr(2)};
                Vertex v;
                s >> v.x >> v.y >> v.z;
                _vertices.push_back(std::move(v));
            }
        }

        file.close();
        log::hxLog.info("加载:", path, "完成!");
    }

    auto& getVertices() const noexcept {
        return _vertices;
    }

private:
    std::vector<Vertex> _vertices;
};

} // namespace HX

void show() {
    glBegin(GL_TRIANGLES);
    static auto vertices = [] {
        ObjParser res;
        res.parser("./obj/monkey.obj");
        return res;
    }();
    for (auto const& v : vertices.getVertices())
        glVertex3f(v.x, v.y, v.z);
    CHECK_GL(glEnd());
}

int main() {
    auto* window = initOpenGL();
    log::hxLog.debug("OpenGL version: ", glGetString(GL_VERSION));

    CHECK_GL(glEnable(GL_POINT_SMOOTH));
    CHECK_GL(glEnable(GL_BLEND));
    CHECK_GL(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
    CHECK_GL(glPointSize(64.0f));

    while (!glfwWindowShouldClose(window)) {
        CHECK_GL(glClear(GL_COLOR_BUFFER_BIT)); // 清空画布
        show();
        glfwSwapBuffers(window); // 双缓冲
        glfwPollEvents();
    }
    return 0;
}
```

非常简单, 读取一整行(`std::getline`), 然后看看是否有 `v ` 顶点数据, 有就直接流式解析~

### 2.4 认识 OBJ 文件格式: 面数据

![认识面数据 ##w500##](HX_2025-07-31_15-20-55.png)

### 2.5 从 OBJ 文件格式, 看多边形网格数据结构

- vertices 是 `vector<array<float, 3>>`, 每个顶点有三个 float, 分别为 x, y, z 表示顶点坐标。

```sh
v -1.000000 1.000000 1.000000
v -1.000000 -1.000000 1.000000
v 1.000000 -1.000000 1.000000
v 1.000000 1.000000 1.000000
```

- faces 是 `vector<vector<int>>`, 每个 n 边形面, 由 n 个顶点的下标(int)构成。

```cpp
f 1 2 3 4   // 由 1、2、3、4 号顶点构成一个四边形面
f 1 2 3     // 由 1、2、3 号顶点构成一个三角形面
f 1 3 4     // 由 1、3、4 号顶点构成一个三角形面
f 1 3       // 由 1 号 和 3 号顶点连成一条线段
```

### 2.6 OBJ 格式的其他数据

OBJ 格式有时候还会包含 `法线(vn)` 和 `纹理坐标(vt)`

![vn and vt ##w300##](HX_2025-07-31_16-25-09.png)

虽说我们不去解析 vn 和 vt 就可以...但可恶的是 f 指令会因此产生变化:

- 原本只有一个整数(v 编号), 会变成由斜杠分割的三个整数(分别为 v、vn、vt 的编号)

> 由于我们只想要 v 的部分, 只需要斜杠分割的第一个数就行, 例如 6/7/4 我们只需要其中的 6

```cpp [c26-解析面]
void parser(std::string_view path) {
    std::ifstream file{{path.data(), path.size()}};
    if (!file.is_open()) [[unlikely]] {
        log::hxLog.error("打开文件:", path, "失败!");
        throw std::runtime_error{path.data()};
    }
    std::string line;
    while (std::getline(file, line)) {
        auto head = line.substr(0, 2);
        if (head == "v ") {
            // 解析顶点
            std::istringstream s{line.substr(2)};
            Vertex v;
            s >> v.x >> v.y >> v.z;
            _vertices.push_back(std::move(v));
        } else if (head == "f ") {
            // 解析面
            std::istringstream s{line.substr(2)};
            std::string a, b, c; // 从 1/2/3 1/2/3 1/2/3 中, 解析出 1, 1, 1
            s >> a >> b >> c;
            Triangle t;
            std::istringstream{a} >> t.a;
            std::istringstream{b} >> t.b;
            std::istringstream{c} >> t.c;
            --t.a, --t.b, --t.c; // obj 索引是从 1 开始的, 换算到 0 开始.
            _trinales.push_back(std::move(t));
        }
    }

    file.close();
    log::hxLog.info("加载:", path, "完成!");
}
```

读取到:

![mk-f3 ##w400##](HX_2025-07-31_16-39-00.png)

> [!TIP]
> 小彭老师说: 这个 OBJ 模型中存在四边形...而我们刚刚的代码只读取了四边形的前三个顶点, 导致显示出来的只是其中一半的三角形
>
> 由于 OpenGL 只支持渲染三角形, 不支持四边形, 我们需要在 load_obj 函数中, 检测读到的面片是否为四边形, 如果是则需要把他拆分成两个三角形。

### 2.7 任意多边形拆分成三角形的原理

![任意多边形拆分成三角形的原理 ##w700##](HX_2025-07-31_16-43-10.png)

```cpp [c27-拆分任意多边形]
std::string line;
while (std::getline(file, line)) {
    auto head = line.substr(0, 2);
    if (head == "v ") {
        // 解析顶点
        std::istringstream s{line.substr(2)};
        Vertex v;
        s >> v.x >> v.y >> v.z;
        _vertices.push_back(std::move(v));
    } else if (head == "f ") {
        // 解析面
        std::istringstream s{line.substr(2)};
        std::vector<std::size_t> idx;
        while (std::getline(s, head, ' ')) {
            std::size_t i;
            std::istringstream{head} >> i;
            idx.push_back(i - 1);
        }
        for (std::size_t i = 2; i < idx.size(); ++i)
            _trinales.push_back({idx[0], idx[i], idx[i - 1]}); // 数据应该保证点之间是有顺序的
    }
}
```

效果:

![mk-fn ##w400##](HX_2025-07-31_16-52-31.png)

### 2.8 使用 glm 封装好的矢量代替自定义的点面类

```cpp [c28-自定义数据]
struct Vertex {
    float x, y, z;
};

struct Triangle {
    std::size_t a, b, c;
};

std::vector<Vertex> _vertices;
std::vector<Triangle> _trinales;
```

```cpp [c28-glm 封装]
#include <glm/vec3.hpp>         // vec3
#include <glm/gtc/type_ptr.hpp> // glm::value_ptr

std::vector<glm::vec3> _vertices;
std::vector<glm::uvec3> _trinales;

// 内部定义, 节选, 可以任意混用 xyz / rgb / zbp, 都是一个东西, 类似于别名:
template<typename T, qualifier Q>
struct vec<3, T, Q> {
    union { T x, r, s; };
    union { T y, g, t; };
    union { T z, b, p; };
};

// 使用 glm::value_ptr 获取指向首个元素的指针
auto& vertices = obj.getVertices();
for (auto const& v : obj.getTriangles()) {
    // glVertex3fv(&vertices[v.x].x);
    // glVertex3fv(&vertices[v.y].x);
    // glVertex3fv(&vertices[v.z].x);
    glVertex3fv(glm::value_ptr(vertices[v.x]));
    glVertex3fv(glm::value_ptr(vertices[v.y]));
    glVertex3fv(glm::value_ptr(vertices[v.x]));
}
```

### 2.9 glm 矢量库中的预定义类型一览

![glm 矢量库中的预定义类型一览 ##w700##](HX_2025-07-31_17-06-51.png)

常用头文件:

```cpp vscode
#include <glm/glm.hpp>                  // 基础类型 vec/mat
#include <glm/gtc/matrix_transform.hpp> // 矩阵变换
#include <glm/gtc/type_ptr.hpp>         // value_ptr
#include <glm/gtx/quaternion.hpp>       // 四元数
```

## 三、复习 OpenGL 中的坐标系和矩阵
对象空间 ➡ 世界矩阵 ➡ 世界空间 ➡ 视角矩阵 ➡ 视角空间 ➡ 投影矩阵 ➡ 裁剪空间 ➡ 光栅化 ➡ 窗口空间

### 3.1 local to world(又称 Model 矩阵)

![Model 矩阵 ##w700##](HX_2025-07-31_17-26-59.png)

### 3.2 world to view(又称 View 矩阵)

![View 矩阵 ##w700##](HX_2025-07-31_17-27-50.png)

### 3.3 view to clip(又称 Projection 矩阵)

![Projection 矩阵 ##w700##](HX_2025-07-31_17-28-45.png)

### 3.4 OpenGL 进行裁剪(clip)操作

![OpenGL 进行裁剪(clip)操作 ##w700##](HX_2025-07-31_17-29-37.png)

### 3.5 NDC to window

![NDC ##w700##](HX_2025-07-31_17-30-24.png)

![预判 ##w700##](HX_2025-07-31_17-32-32.png)

![预判你的预判 ##w700##](HX_2025-07-31_17-33-16.png)

### 3.6 总结: 我们需要考虑的有 Model、View、Projection 矩阵

- **Model** 矩阵负责从对象局域坐标系(物体建模时采用的)转换到世界坐标系(场景布置)。

- **View** 矩阵负责把世界坐标系转换到摄像机为中心, 摄像机方向为 z 轴负方向的视角坐标系。

- **Projection** 矩阵负责把视角坐标系的三维坐标, 转换到带有 w 分量的齐次空间, 同时还对 z 轴进行了翻转(变左手系), 得到的四维坐标存入 OpenGL 内置的特殊变量: gl_Position。

- 最终由 OpenGL 内部取出 gl_Position 执行“Perspective Divide”操作: 将 x、y、z 分量同除以 w。此时如果刚才 Projection 矩阵把 w 设为 z, 就会产生“近大远小”的透视效果。

- 随后由 OpenGL 内部取出 x、y 坐标用于确定屏幕上像素点的位置, z 坐标用于深度测试, 并利用硬件加速的光栅化在屏幕上绘制立体图案。

![##w700##](HX_2025-07-31_17-35-16.png)

### 3.7 Model 矩阵 - 物体局部坐标系到世界坐标系

- OBJ 中的 x、y、z 坐标, 是物体局部坐标系。

- 而多个物体可能包含相同的 x、y、z 坐标, 为了让他们不要叠在一起, 每个物体都有一个 Model 矩阵, 负责把该物体的局部坐标转换为世界坐标。

- Model 矩阵通常包含: 缩放、旋转、平移三部分, 乘法时顺序相反: $M = TRS$。

- `平移`部分 $T$ 控制着物体的`位置`, 可以由 `glm::translate(glm::vec3(x, y, z))` 创建;

- `旋转`部分 $R$ 控制着物体的`朝向`, 可以由 `glm::rotate(angle_radians, axis)` 创建;

- `缩放`部分 $S$ 控制着物体的`大小`, 可以由 `glm::scale(glm::vec3(sx, sy, sz))` 创建。

- Model 矩阵是一个**协变矩阵**, 负责把三维坐标从局部坐标系转换到世界坐标系。

- Model 矩阵是 4x4 的齐次矩阵, 其 $w$ 分量用于乘以平移的偏移量, 当需要把 Model 矩阵作用于绝对矢量(**位置**)时, 请令 $w$ 为 1; 当作用于相对矢量(例如**速度**)时, $w$ 应为 0。

- 当作用于法线时, 需要乘以的不是 Model 而是 Model 中 3x3 部分的逆转置: $(M^{-1})^T$。

- 对于附着在物体上与空间无关的属性, 如**纹理 UV**, 则不用乘 Model 矩阵, 保持不变即可。

### 3.8 `glm::translate` 这些函数的一个坑点

实际上他们不支持 `glm::translate(glm::vec3(x, y, z))` 这样写。

要 `glm::translate(glm::mat4x4(1), glm::vec3(x, y, z))` 这样写。

因为 glm 的作者为了所谓的“便民”, 给 `glm::translate` 强制加上了第一个参数。

然后 `glm::translate` 不是直接返回他得到的平移矩阵, 而是返回这个平移矩阵从左边乘以那该死的第一个参数后的结果, 他们就觉得这样很“便民”,

例如:

```cpp
model = model * glm::translate(glm::vec3(x, y, z));
model = glm::translate(model, glm::vec3(x, y, z));
```

以上两种写法等价。