# 1.4 Spring对bean的管理方式
## 1.4.1 Spring实例化bean的方式
### 1.4.1.1 无参数构造函数的实例化方式
无参数构造函数实例化方式是Spring默认的bean的实例化方式。

```java
public class AccountDaoImpl implements AccountDao {
    AccountDaoImpl() {
        System.out.println("AccountDaoImpl 构造了");
    }
    @Override
    public void addAccount() {
        System.out.println("[Dao]: 新增账户的方法实现了....");
    }
}
```

```xml
<bean id="accountDao" class="com.HX.dao.impl.AccountDaoImpl"></bean>
```

```java
public class Test {
    public static void main(String[] args) {
        ApplicationContext context = new ClassPathXmlApplicationCo
        AccountDao accountDao = (AccountDaoImpl) context.getBean("accountDao");
        accountDao.addAccount();
    }
}
```

### 1.4.1.2 使用工厂中的普通方法实例化对象
我们设想一个场景: 我们要使用别人写好的代码，别人写好的代码通常是封装在一个jar包中的，我们要怎么创建jar包中字节码文件的对象呢？（我们无法通过修改源码的方式来提供默认构造方法）我们接下来模拟一个工厂类（它可能是存在于jar包中的），通常jar包都会暴露一个工厂类，给调用者创建对象。

创建接口以及其实现

假设下面是我们需要用到的类
```java
public interface PersonDao {
    public void addPerson();
}
```

```java
public class PersonDaoImpl implements PersonDao {
    @Override
    public void addPerson() {
        System.out.println("新增addPerson方法...");
    }
}
```

创建一个工厂类

```java
public class PersonFactory {
    // 方法的返回值就是需要管理的bean的类型
    public PersonDao getPerson() {
        return new PersonDaoImpl();
    }
}
```

配置文件
```xml
<bean id="personFactory" class="com.HX.factory.PersonFactory"></bean>
<bean id="personDao" class="com.HX.dao.impl.PersonDaoImpl" factory-bean="personFactory" factory-method="getPerson"></bean>
```

使用示例
```java
public class Test {
    public static void main(String[] args) {
        ApplicationContext context = new ClassPathXmlApplicationContext("bean.xml");
        PersonDao accountService = (PersonDaoImpl) context.getBean("personDao");
        accountService.addPerson();
    }
}
```

### 1.4.1.3 使用工厂中的静态方法实例化对象

直接在上面的基础上:
```java
public class PersonFactory {
    // 方法的返回值就是需要管理的bean的类型
    public PersonDao getPerson() {
        return new PersonDaoImpl();
    }

    // 静态工厂方法
    public static PersonDao staticGetPerson() {
        return new PersonDaoImpl();
    }
}
```

配置文件
```xml
<bean id="staticPersonDao" class="com.HX.factory.PersonFactory" factory-method="staticGetPerson"></bean>
```

使用示例:
```java
public class Test {
    public static void main(String[] args) {
        ApplicationContext context = new ClassPathXmlApplicationContext("bean.xml");
        PersonDao accountService = (PersonDaoImpl) context.getBean("staticPersonDao");
        accountService.addPerson();
    }
}
```
