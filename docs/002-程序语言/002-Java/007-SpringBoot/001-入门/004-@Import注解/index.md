# @Import注解
`@Import` 注解是 Spring Framework 提供的一个用于导入配置类或普通类的注解。它的主要作用是将其他的 Spring 配置类或组件引入当前的 Spring 上下文中，从而使得这些被引入的类能够被 Spring 容器管理。

首先我们来件`@Import`源码:

```Java
@Target({ElementType.TYPE})
@Retention(RetentionPolicy.RUNTIME)
@Documented
public @interface Import {
    Class<?>[] value();
}
```

@Import注解源码的定义非常简单，就一个属性 value，而且是一个 Class 类型的数组。

结合上面的注释，对我们了解Import有很大的帮助。

可以同时导入多个`@Configuration`类 、`ImportSelector`和`ImportBeanDefinitionRegistrar`的实现，以及导入普通类（4.2版本开始支持）

`@Import`的功能与 xml 中的 在类级别声明或作为元注释`<import/>`标签等效

如果需要导入XML 或其他非bean 定义资源，请使用`@ImportResource`注解

### 作用
1. **导入配置类**：可以将其他的配置类导入当前的配置类中，这样就可以使用其中定义的 Bean。
2. **导入普通类**：除了配置类，`@Import` 还可以用来导入普通的类，这些类也可以作为 Bean 被管理。
3. **模块化配置**：可以通过 `@Import` 将不同的模块分开，增强代码的可读性和可维护性。

### 应用场景
1. **分离配置**：在大型项目中，可以将不同的模块配置分开，使用 `@Import` 组合在一起，方便管理。
2. **条件导入**：可以与 `@Conditional` 注解结合使用，基于某些条件导入特定的配置或类。
3. **第三方库集成**：在集成第三方库时，可能需要导入特定的配置类来初始化该库的相关组件。

### 示例
```java
@Configuration
@Import(value = {OtherConfig.class, AnotherConfig.class})
public class MainConfig {
    // 主配置类
}
```

在这个示例中，`MainConfig` 会导入 `OtherConfig` 和 `AnotherConfig` 中定义的所有 Bean。