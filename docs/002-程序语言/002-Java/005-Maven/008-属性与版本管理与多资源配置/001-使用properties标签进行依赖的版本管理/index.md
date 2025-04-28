# 使用`properties`标签进行依赖的版本管理
在[maven分模块开发与设计](../../007-maven分模块开发与设计/index.md)的基础上, 我们虽然可以直接在根目录的`pom.xml`直接进行版本管理, 但是`<dependency>`太多, 太长了, 很难找, 那么我们可以使用 **属性来描述，方便维护管理**。

即: (节选)

```xml
<!-- 使用属性来描述 -->
<properties>
    <!-- 这里的标签名称随意 -->
    <javax.servlet-api.version>3.1.0</javax.servlet-api.version>
    <jstl.version>1.2</jstl.version>
</properties>

<dependencyManagement>
    <dependencies>
        <!-- servlet依赖的jar包start -->
        <dependency>
            <groupId>javax.servlet</groupId>
            <artifactId>javax.servlet-api</artifactId>
            <version>${javax.servlet-api.version}</version>
            <scope>provided</scope>
        </dependency>
        <!--jstl标签依赖的jar包start -->
        <dependency>
            <groupId>javax.servlet</groupId>
            <artifactId>jstl</artifactId>
            <version>${jstl.version}</version>
        </dependency>
    </dependencies>
</dependencyManagement>
```

类似于使用一个`宏定义`, 这样可能会方便寻找!