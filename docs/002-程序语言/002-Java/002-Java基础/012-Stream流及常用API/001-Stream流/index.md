# Stream流
## 管道

```java
管道就是一系列的聚合操作。

管道包含以下组件：
[源]：可以是集合，数组，生成器函数或I / O通道。
[零个或多个中间操作]: 诸如过滤器之类的中间操作产生新的流。
[终结操作/终端操作]: (例如forEach)会产生非流结果，例如原始值（如双精度值），集合，或者在forEach的情况下根本没有任何值。
```

流

```java
流是一系列元素。与集合不同，它不是存储元素的数据结构。取而代之的是，流通过管道携带来自源的值。

筛选器操作返回一个新流，该流包含与筛选条件（此操作的参数）匹配的元素。
```

## 如何获取 Stream 流
### Collection 接口

```java
default Stream<E> stream();
```

### Stream 接口

```java
// 获取流
public static<T> Stream<T> of(T... values);
// 将两个流拼接形成一个新的流
public static <T> Stream<T> concat(Stream<? extends T> a, Stream<? extends T> b);
```

示例

```java
public class Main {
    public static void main(String[] args) {
        Stream<String> s1 = Arrays.asList("张三", "李四", "老八").stream();
        Stream<String> s2 = Stream.of("张三", "李四", "老八");
        Stream<String> s3 = Stream.concat(s1, s2); // 合成
        
        // 流只能单用, 不支持键值对, 但是可以将键值对视为整体
        Stream<Map.Entry<String, Integer>> s4 = new HashMap<String, Integer>().entrySet().stream();
    }
}
```

## Stream 流中间聚合操作
Stream 接口

这里可以链式调用即返回值还是`Stream<T>`对象, 可以继续调用方法, 类似于: [建造者模式](../../../../../001-计佬常識/002-设计模式/007-创建型模式/005-建造者模式/index.md)

```java
Stream<T> filter(Predicate<? super T> predicate);             // 根据给定的条件过滤流中的元素
<R> Stream<R> map(Function<? super T, ? extends R> mapper);   // 将流中元素进行类型转换
Stream<T> distinct();                                         // 去重
Stream<T> sorted();                                           // 排序，如果存储元素没有实现Comparable或者相关集合没有提供Comparator将抛出异常
Stream<T> limit(long maxSize);                                // 根据给定的上限，获取流中的元素
Stream<T> skip(long n);                                       // 跳过给定数量的元素
IntStream mapToInt(ToIntFunction<? super T> mapper);          // 将流中元素全部转为整数
LongStream mapToLong(ToLongFunction<? super T> mapper);       // 将流中元素全部转为长整数
DoubleStream mapToDouble(ToDoubleFunction<? super T> mapper); // 将流中元素全部转为双精度浮点数
```

示例

```java
public class Main {
    public static void main(String[] args) {
        Stream<String> s2 = Stream.of("张三", "李四", "老八", "老八");
        System.out.println(s2.distinct().filter(s -> !s.equals("张三")).count()); // 去重后 再去除 "张三" 再计数
    }
}
```

## Stream 流终结操作

```java
void forEach(Consumer<? super T> action);               // 遍历操作流中元素
<A> A[] toArray(IntFunction<A[]> generator);            // 将流中元素按照给定的转换方式转换为数组
<R, A> R collect(Collector<? super T, A, R> collector); // 将流中的元素按照给定的方式搜集起来
Optional<T> min(Comparator<? super T> comparator);      // 根据给定的排序方式获取流中最小元素
Optional<T> max(Comparator<? super T> comparator);      // 根据给定的排序方式获取流中最大元素
Optional<T> findFirst();                                // 获取流中第一个元素
long count();                                           // 获取流中元素数量
boolean anyMatch(Predicate<? super T> predicate);       // 检测流中是否存在给定条件的元素
boolean allMatch(Predicate<? super T> predicate);       // 检测流中元素是否全部满足给定条件
boolean noneMatch(Predicate<? super T> predicate);      // 检测流中元素是否全部不满足给定条件
```

示例

```java
public class Main {
    public static void main(String[] args) {
        Stream<String> s2 = Stream.of("张三", "李四", "老八", "老八");
        s2.distinct().filter(s -> !s.equals("张三")).forEach(System.out::println); // 使用终结操作后, 流就被清空了

        // 需要重新赋值
        s2 = Stream.of("张三", "李四", "老八", "老八");
        Optional<String> s2Find = s2.findFirst();
        System.out.println(s2Find.get());

        System.out.println(Stream.of(1, 2, 3, 4, 5).anyMatch(integer -> integer < 3)); // 存在满足
        System.out.println(Stream.of(1, 2, 3, 4, 5).allMatch(integer -> integer < 3)); // 全部满足
        System.out.println(Stream.of(1, 2, 3, 4, 5).noneMatch(integer -> integer < 3)); // 全部不满足
    }
}
```

## Stream 流聚合操作与迭代器的区别
> 聚合操作（如forEach）似乎像迭代器。但是，它们有几个基本差异：
>
> 它们使用内部迭代：聚合操作不包含诸如`next`的方法来指示它们处理集合的下一个元素。使用内部委托，你的应用程序确定要迭代的集合，而JDK确定如何迭代该集合。通过外部迭代，你的应用程序既可以确定要迭代的集合，又可以确定迭代的方式。但是，外部迭代只能顺序地迭代集合的元素。内部迭代没有此限制。它可以更轻松地利用并行计算的优势，这涉及将问题分为子问题，同时解决这些问题，然后将解决方案的结果组合到子问题中。
>
> 它们处理流中的元素：聚合操作从流中而不是直接从集合中处理元素。因此，它们也称为流操作。
>
> 它们支持将行为作为参数：你可以将`lambda`表达式指定为大多数聚合操作的参数。这使你可以自定义特定聚合操作的行为。

## 面试题
使用`Stream`流将一个基本数据类型的数组转换为包装类型的数组，再将包装类型的数组转换基本数据类型数组。

```java
public class Main {
    public static void main(String[] args) {
        // 使用`Stream`流将一个基本数据类型的数组转换为包装类型的数组，再将包装类型的数组转换基本数据类型数组。
        int[] arr = new int[]{1, 1, 4, 5, 1, 4};
        // 两种方法
        Integer[] arrClass = Arrays.stream(arr).mapToObj(value -> value).toArray(Integer[]::new);
        Integer[] arrClass2 = Arrays.stream(arr).boxed().toArray(Integer[]::new);

        int[] arr2 = Arrays.stream(arrClass).mapToInt(value -> value).toArray();
    }
}
```
