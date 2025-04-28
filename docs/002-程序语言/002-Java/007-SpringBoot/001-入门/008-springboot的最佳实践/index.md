# springboot的最佳实践
### 1. 引入场景依赖
- **使用 Starter 依赖**：Spring Boot 提供了许多 Starter 依赖，简化了项目的依赖管理。通过添加相应的 Starter，可以快速引入所需的功能模块。
- **示例**：
  ```xml
  <dependency>
      <groupId>org.springframework.boot</groupId>
      <artifactId>spring-boot-starter-web</artifactId>
  </dependency>
  ```

### 2. 查看自动配置
- **自动配置报告**：可以通过设置 `debug=true` 来查看哪些自动配置生效或不生效。报告分为 Positive（生效）和 Negative（不生效）。
- **分析自动配置**：参考 Spring Boot 的官方文档，了解特定场景下引入的 Starter 所对应的自动配置，评估是否需要修改配置项。

### 3. 配置文件设置
- **配置文件示例**：
  ```properties
  spring.banner.image.location=classpath:abc.jpg
  debug=true
  ```
- **修改配置项**：参照官方文档，适当修改配置项以满足项目需求。

### 4. 自定义配置绑定
- **使用 `@ConfigurationProperties`**：可以将配置文件中的属性绑定到 Java 类上，以便于管理和使用。
- **示例**：
  ```java
  @ConfigurationProperties(prefix = "app")
  public class AppProperties {
      private String name;
      private String version;
      // getters and setters
  }
  ```

### 5. 自定义组件
- **使用 `@Bean` 和 `@Component`**：可以创建自定义的 Bean 或组件，以扩展 Spring Boot 的功能。
- **示例**：
  ```java
  @Bean
  public MyService myService() {
      return new MyService();
  }
  ```

### 6. 自定义器
- **使用 Customizer**：可以创建自定义器来对某个 Bean 进行定制化配置。
- **示例**：
  ```java
  public class MyCustomizer implements SomeCustomizer {
      @Override
      public void customize(SomeBean someBean) {
          // 定制化逻辑
      }
  }
  ```

### 结论
- 在使用 Spring Boot 开发时，遵循这些最佳实践可以提高开发效率和应用的可维护性。通过合理利用自动配置、场景依赖和自定义组件，可以快速构建出高质量的应用程序。

# 附件: 自定义启动Logo
在 Spring Boot 中自定义启动图标（logo）通常涉及到修改项目的 Banner 设置。以下是如何自定义 Spring Boot 启动图标的步骤：

### 1. 准备图标文件
将你想要使用的图标文件（如 JPG、PNG 或 GIF 格式）放在项目的 `resources` 目录下。比如，你可以创建一个名为 `static` 的文件夹，并将图标文件放入其中：

```
src/main/resources/static/abc.jpg
```

### 2. 配置文件设置
在 `application.properties` 或 `application.yml` 中，设置图标的路径。例如，如果你的图标是 `abc.jpg`，你可以这样配置：

**application.properties**:
```properties
spring.banner.image.location=classpath:static/abc.jpg
```

**application.yml**:
```yaml
spring:
  banner:
    image:
      location: classpath:static/abc.jpg
```

### 3. 自定义 Banner
如果你还想自定义文本 Banner，可以在 `resources` 目录中创建一个 `banner.txt` 文件，内容可以是 ASCII 艺术或其他文本。

### 4. 启动应用
运行你的 Spring Boot 应用，启动时将显示自定义的图标和 Banner 文本。

### 5. 检查效果
启动应用后，可以在控制台中查看是否显示了自定义的图标和 Banner。如果图标没有显示，可以检查路径是否正确，确保文件格式支持，并查看日志中是否有相关错误信息。

### 其他注意事项
- 确保图标文件的大小合适，以避免在控制台中显示不清晰或影响启动速度。
- 可以在不同环境中使用不同的图标，通过使用 `application-{profile}.properties` 文件来针对不同的环境配置。

通过以上步骤，你可以成功地自定义 Spring Boot 应用的启动图标。