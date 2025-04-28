# 静态资源映射规则
## 1. 静态资源路径
> [!TIP]
> 注意, 资源需要放在`resources`内, 比如`resources/static/**`

默认情况下，Spring Boot 从类路径中的`/static`（或`/public`、`/resources`或`/META-INF/resources`）目录中提供静态内容。它使用 Spring WebFlux 的 ResourceWebHandler，因此您可以通过添加自己的 WebFluxConfigurer 并重写 addResourceHandlers 方法来修改此行为。

默认情况下，资源映射在`/**`上，但您可以通过设置`spring.webflux.static-path-pattern`属性来调整它。例如，将所有资源重新定位到`/resources/**`可以按照以下方式实现:

```yaml
spring:
  webflux:
    static-path-pattern: "/resources/**"
```

还可以通过使用`spring.web.resources.static-locations`来自定义静态资源位置。这样做将用目录位置列表替换默认值。如果您这样做，默认的欢迎页面检测将切换到您自定义的位置。

## 2. 欢迎界面和图标
在静态资源的根目录, `index.html`为欢迎界面.

`favicon.ico`也一样, 只需要放在根目录即可, 会自动识别...

## 3. Webjars
除了之前列出的“标准”静态资源位置之外，对于 Webjars 内容有一个特殊处理。默认情况下，任何路径在/webjars/**下的资源，如果它们是以 Webjars 格式打包的 jar 文件，则会被提供服务。路径可以通过 spring.webflux.webjars-path-pattern 属性进行自定义。

Webjars也就是把静态资源打成jar包让我们访问。

(誰会这么干?)

## 4. 自定义错误界面

例如，要将 404 映射到静态 HTML 文件，您的目录结构应该是这样的:

```sh
src/
 +- main/
     +- java/
     |   + <source code>
     +- resources/
         +- public/
             +- error/
             |   +- 404.html
             +- <other public assets>
```

其他自己查!