# 二维数组
## 声明

`数据类型[][] 数组名 = new 数据类型[数组的长度][数组的长度];`

```java
public class demo_4 {
    public static void main(String[] arg) {
        int[][] map = new int[5][5];
        for (int i = 0; i < map.length; ++i)
            for (int j = 0; j < map[i].length; ++j)
                System.out.print(map[i][j] + (j == map[i].length - 1 ? "\n" : " "));

        System.out.println("");
        int[][] arr = new int[5][]; // 可以只声明外层, 内层可以变长度
        for (int i = 0; i < arr.length; ++i) {
            arr[i] = new int[i + 1];
            for (int j = 0; j < i + 1; ++j) {
                arr[i][j] = j;
            }
        }

        for (int i = 0; i < arr.length; ++i)
            for (int j = 0; j < arr[i].length; ++j)
                System.out.print(arr[i][j] + (j == arr[i].length - 1 ? "\n" : " "));
    }
}
```

可以理解为 数组里面的元素是数组, 学过C++的`std::vector`dddd