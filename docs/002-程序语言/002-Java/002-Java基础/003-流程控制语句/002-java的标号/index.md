# java的标号

类似于goto但是到达的地方是有限的

*使用标记可能会导致代码变得复杂且难以理解，因此在日常的Java编程中，应该避免过度依赖标记，而是尽量采用结构化的编程风格，编写清晰易懂的代码，提高代码的可读性和可维护性。*

(我不能用goto, 你最好也不要用标号!(java没有`goto`))

```java
emoji: while (true) {
    abc: while (true) {
        while (true) {

            break emoji; // 指定跳出的是 emoji后面的while
            // 直接就退出了这个三层嵌套的while了
        }
        continue abc; // 指定 重新执行的是 abd 后面的while
    }
}
```
