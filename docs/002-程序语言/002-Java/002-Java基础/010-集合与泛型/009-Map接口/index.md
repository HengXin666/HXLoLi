# Map 接口
## 特性描述
> Map集合是将键映射到值的对象。映射不能包含重复的键：每个键最多可以映射到一个值。
>
> Java平台包含三个常用Map的实现：HashMap，TreeMap和LinkedHashMap。

**`Map`接口常用方法**

```java
int size();                                   // 获取集合的大小
boolean isEmpty();                            // 判断集合是否为空
boolean containsKey(Object key);              // 判断集合中是否包含给定的键
boolean containsValue(Object value);          // 判断集合中是否包含给定的值
V get(Object key);                            // 获取集合中给定键对应的值
V put(K key, V value);                        // 将一个键值对放入集合中
V remove(Object key);                         // 将给定的键从集合中移除
void putAll(Map<? extends K, ? extends V> m); // 将给定的集合添加到集合中
void clear();                                 // 清除集合中所有元素
Set<K> keySet();                              // 获取集合中键的集合
Collection<V> values();                       // 获取集合中值的集合
Set<Map.Entry<K, V>> entrySet();              // 获取集合中键值对的集合
```

**`Entry`接口常用方法**

```java
K getKey();               // 获取键
V getValue();             // 获取值
V setValue(V value);      // 设置值
boolean equals(Object o); // 比较是否是同一个对象
int hashCode();           // 获取哈希码
```

<b style="color:red">`Map`接口中的内部接口`Entry`就是`map`存储的数据项，一个`Entry`就是一个键值对(类似于C++STL的`std::pair`)。</b>

## 手搓map

```java
class MyHashMap<K, V> {
    private int size;
    private float loadFactor = 0.75f; // 负载因子
    private MyEntry<K, V>[] arr;

    MyHashMap(int size) {
        this.size = 0;
        this.arr = new MyEntry[size];
    }

    int getHashVal(K key) {
        return key.hashCode() & (this.arr.length - 1);
    }

    void add(K key, V val) {
        int new_size = size + 1;
        if (new_size > arr.length * loadFactor) {
            // 进行扩展... 略
        }

        int index = this.getHashVal(key);
        if (this.arr[index] == null) {
            // 直接添加
            this.arr[index] = new MyEntry<>(key, val, null);
        } else {
            // 变成链表 [头插]
            this.arr[index] = new MyEntry<>(key, val, this.arr[index]);
        }
        ++size;
    }

    V getVal(K key) {
        MyEntry<K, V> res = this.arr[this.getHashVal(key)];
        while (res != null && !res.getKey().equals(key))
            res = res.getNext();
        return res.getVal();
    }
}

class MyEntry<K, V> {
    private K key;
    private V val;
    private MyEntry<K, V> next;

    MyEntry(K key, V val, MyEntry<K, V> next) {
        this.key = key;
        this.val = val;
        this.next = next;
    }

    public MyEntry<K, V> getNext() {
        return next;
    }

    public void setNext(MyEntry<K, V> next) {
        this.next = next;
    }

    public K getKey() {
        return key;
    }

    public void setKey(K key) {
        this.key = key;
    }

    public V getVal() {
        return val;
    }

    public void setVal(V val) {
        this.val = val;
    }
}

public class MyHashMapText {
    public static void main(String[] args) {
        MyHashMap<Integer, String> map = new MyHashMap<>(17);
        map.add(1, "Boss");
        map.add(2, "ZaZa");
        map.add(3, "qwq");
        map.add(18, "awa");
        System.out.println(map.getVal(18));
    }
}
```

## HashMap
> 基于哈希表的Map接口的实现。此实现提供所有可选的映射操作，并允许空值和空键。（HashMap类与Hashtable大致等效，不同之处在于它是不同步的，并且允许为null。）该类不保证映射的顺序。特别是，它不能保证顺序会随着时间的推移保持恒定。

`HashMap`存储的是一组无序的键值对。存储时是根据键的哈希码来计算存储的位置，因为对象的哈希码是不确定的，因此`HashMap`存储的元素是无序的。

采用数组+链表+红黑树设计

平常是数组映射, 冲突则使用链表, 链表过长(超过8), 就变为[红黑树](../../../../../001-计佬常識/001-数据结构与算法/008-【数据结构】高阶搜索树/002-红黑树/001-红黑树丶/index.md), 过短(小于6)就退化为链表.


```java
public class MyHashMapText {
    public static void main(String[] args) {
        HashMap<String, String> map = new HashMap<>();
        // 插入
        map.put("张三", "法外狂徒");
        map.put("李四", "???");
        map.put("老八", "美食家");
        // 查找
        System.out.println(map.get("老八"));
        // 删除
        map.remove("李四");
        // 遍历
        for (Map.Entry it : map.entrySet())
            System.out.println(it.getKey() + " -> " + it.getValue());
        // 单独遍历键
        map.keySet();
        // 单独遍历值
        map.values();
    }
}
```

## TreeMap
> 基于红黑树的`NavigableMap`实现。根据集合存储的键的自然排序或在映射创建时提供的`Comparator`来对键进行排序，具体取决于所使用的构造方法。

使用同`HashMap`, 注意内部会排序, 需要一个`比较器`

示例1: String内部有实现比较器接口
```java
public class MyHashMapText {
    public static void main(String[] args) {
        TreeMap<String, String> map = new TreeMap<>();
        // 插入
        map.put("张三", "法外狂徒");
        map.put("李四", "???");
        map.put("老八", "美食家");
        // 查找
        System.out.println(map.get("老八"));
        // 删除
        map.remove("李四");
        // 遍历
        for (Map.Entry it : map.entrySet())
            System.out.println(it.getKey() + " -> " + it.getValue());
        // 单独遍历键
        map.keySet();
        // 单独遍历值
        map.values();
    }
}
```

示例2: 匿名比较器外部实现

```java
class Loli {
    private String name;
    private int arg;

    Loli(String name, int arg) {
        this.name = name;
        this.arg = arg;
    }

    public String getName() {
        return name;
    }

    public int getArg() {
        return arg;
    }
}

public class MyHashMapText {
    public static void main(String[] args) {
        Comparator<Loli> c = (o1, o2) -> o1.getArg() - o2.getArg();
        TreeMap<Loli, String> map = new TreeMap<>(c);
        // 插入
        map.put(new Loli("ラストオーダー", 9), "zsjj");
        map.put(new Loli("五河琴里", 14), "imouto");
        map.put(new Loli("愛", 10), "ll");
        map.put(new Loli("yujiann", 7), "xgks");
        // 遍历
        for (Map.Entry it : map.entrySet())
            System.out.println(((Loli)it.getKey()).getName() + "[" + ((Loli)it.getKey()).getArg() + "]" + " -> " + it.getValue());
    }
}
```

## LinkedHashMap
> Map接口的哈希表和链表实现，具有可预测的迭代顺序。此实现与HashMap的不同之处在于，它维护一个贯穿其所有条目的双向链表。此链表定义了迭代顺序，通常是将键插入映射的顺序（插入顺序）。请注意，如果将键重新插入到映射中，则插入顺序不会受到影响。

遍历顺序按照插入顺序来, 插入的键相同则视作修改该元素.

示例:

```C++
public class MyHashMapText {
    public static void main(String[] args) {
        LinkedHashMap<String, String> map = new LinkedHashMap<>();
        map.put("钻石", "铁镐");
        map.put("红石", "铁镐");
        map.put("石头", "赤手空拳");
        map.put("钻石", "钻石镐");
        for (Map.Entry it : map.entrySet())
            System.err.println(it.getKey() + " -> " + it.getValue());
    }
}
```
