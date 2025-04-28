# 栈
## 手撕

```java
import java.util.ArrayList;

class MyStack<T> extends ArrayList<T> {
    public void push(T t) {
        super.add(t);
    }

    public void pop() {
        super.remove((T)this.top());
    }

    public T top() {
        if (super.size() == 0) {
            throw new IllegalArgumentException("栈没有数据了");
        }
        return super.get(super.size() - 1);
    }
}

public class MyStackText {
    public static void main(String[] args) {
        MyStack<Integer> stack = new MyStack<>();
        stack.add(1);
        stack.push(2); // add 是 父类 ArrayList 有的
        stack.add(3);
        while (true) {
            System.out.println(stack.top()); // 最后会报错
            stack.pop();
        }
    }
}
```

## 自带的

```java
public class MyStackText {
    public static void main(String[] args) {
        Stack<Integer> stack = new Stack<>();
        stack.add(1);
        stack.push(2); // add 是 父类 ArrayList 有的
        stack.add(3);
        while (!stack.empty()) {
            System.out.println(stack.pop());
        }
    }
}
```
