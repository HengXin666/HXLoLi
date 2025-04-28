# lombok简化开发
lombok是一个插件，用途是使用注解给你类里面的字段，自动的加上属性，构造器，ToString方法，Equals方法等等,比较方便的一点是，你在更改字段的时候，lombok会立即发生改变以保持和你代码的一致性。

## 1. 安装

1. 安装lombok插件
2. 注意: 安装完后一定要重启idea。
3. 引入lombok依赖

```xml
<dependency>
    <groupId>org.projectlombok</groupId>
    <artifactId>lombok</artifactId>
</dependency>
```

## 2. 使用
### 2.1 简化开发

```Java
@Data               // 省略定义get set toString方法
@NoArgsConstructor  // 不用写无参数的构造函数
@AllArgsConstructor // 不用定义带所有参数的构造函数
public class User { // ...
```

### 2.2 简化日志开发
```Java
@Slf4j // 这个注解
public class MyTestClass {
    static {
        log.info("日志输出!!");
    }
}
```