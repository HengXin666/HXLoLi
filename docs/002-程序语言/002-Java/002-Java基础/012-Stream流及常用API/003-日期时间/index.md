# 时间 Date 类
## 常用方法

```java
public Date();                    // 无参构造，表示计算机系统当前时间，精确到毫秒
public Date(long date);           // 带参构造，表示根据给定的时间数字来构建一个日期对象，精确到毫秒

public long getTime();            // 获取日期对象中的时间数字，精确到毫秒
public boolean before(Date when); // 判断当前对象表示的日期是否在给定日期之前
public boolean after(Date when);  // 判断当前对象表示的日期是否在给定日期之后
```

示例

```java
import java.util.Date;

public class MyDate {
    public static void main(String[] args) {
        Date date = new Date();
        System.out.println(date);

        Date senjitsu = new Date(System.currentTimeMillis() - 24 * 60 * 60 * 1000); // 获取昨天时间
        System.out.println(senjitsu);

        System.out.println(date.getTime() - senjitsu.getTime());
        System.out.println(date.after(senjitsu));
        System.out.println(date.before(senjitsu));
    }
}
```

- 思考：打印的日期不好看懂，能否按照我们熟悉的方式来打印？

    当然可以，首先我们需要将日期按照我们熟悉的日期格式转换为一个字符串日期，然后再打印。

## SimpleDateFormat 类
常用方法

```java
public SimpleDateFormat(String pattern);                // 根据给定的日期格式构建一个日期格式化对象
public final String format(Date date);                  // 将给定日期对象进行格式化
public Date parse(String source) throws ParseException; // 将给定的字符串格式日期解析为日期对象
```

常用日期格式

|字母|含义|说明|
|:-:|:-:|:-:|
|y|年year|不区分大小写，一般用小写|
|M|月month|区分大小写，只能使用大写|
|d|日day|区分大小写，只能使用小写|
|H|时hour|不区分大小写，一般用大写|
|m|分minute|区分大小写，只能使用小写|
|s|秒second|区分大小写，只能使用小写|

示例

```java
public class MyDate {
    public static void main(String[] args) {
        Date date = new Date();
        SimpleDateFormat format = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
        System.out.println(format.format(date));
        try {
            System.out.println(format.parse("2024-3-9 20:26:28"));
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }
}
```

# 日历 Calendar 类
## 常用静态字段
|字段值|含义|
|:-:|:-|
|YEAR|年|
|MONTH|月，从0开始|
|DAY_OF_MONTH|月中第几天|
|HOUR|时（12小时制）|
|HOUR_OF_DAY|时（24小时制）|
|MINUTE|分|
|SECOND|秒|
|DAY_OF_WEEK|周中第几天，一周的第一天是周日，因此周日是1|

## 常用方法

```java
public static Calendar getInstance();    // 获取日历对象
public final Date getTime();             // 获取日历表示日期对象
public final void setTime(Date date);    // 设置日历表示的日期对象

public int get(int field);               // 获取给定的字段的值
public void set(int field, int value);   // 设置给定字段的值
public void roll(int field, int amount); // 根据给定的字段和更改数量滚动日历
public int getActualMaximum(int field);  // 获取给定字段的实际最大数量
```

示例
```java
public class MyDate {
    public static void main(String[] args) {
        Calendar instance = Calendar.getInstance();
        System.out.println(instance.getTime());
        System.out.println(instance.get(Calendar.DAY_OF_WEEK) + " " + instance.getActualMaximum(Calendar.DAY_OF_MONTH));

        instance.roll(Calendar.MONTH, 2); // 往后滚动2个月
        System.out.println(instance.getTime());
    }
}
```

## 练习
制作一个万年历

```java
import java.util.Calendar;

interface PrintfColor {
    /**
     * 31红, 32绿, 33黄, 34深蓝, 35紫, 36亮蓝
     * @param code 颜色代号：背景颜色代号(41-46)；前景色代号(31-36)
     * @param content 要打印的内容
     */
    static void printSingleColor(int code, String content){
        System.out.printf("\033[%d;0m%s\33[0m", code, content);
    }
}

class MyDateMoNo {
    private static final String[] WEEK_STR = {"一", "二", "三", "四", "五", "六", "日"};

    public static void show() {
        Calendar c = Calendar.getInstance();
        // 1. 计算本月的第一天是星期几[星期日 1是开始], 但是我需要星期一是开始 d + 6 % 7 [0 + 1是星期一]
        c.set(Calendar.DAY_OF_MONTH, 1);
        int now_mon_hajime_day_week = (c.get(Calendar.DAY_OF_WEEK) + 6) % 7;
        System.out.println(now_mon_hajime_day_week);
        int now_mon_max_day = c.getActualMaximum(Calendar.DAY_OF_MONTH); //本月有多少天
        // 2. 计算上一个月有多少天
        c.roll(Calendar.MONTH, -1);
        int mae_mon_max_day = c.getActualMaximum(Calendar.DAY_OF_MONTH);


        for (int i = 0; i < 5; ++i) {
            PrintfColor.printSingleColor(33,MyDateMoNo.WEEK_STR[i] + "  ");
        }

        for (int i = 5; i < 7; ++i) {
            PrintfColor.printSingleColor(32, MyDateMoNo.WEEK_STR[i] + "  ");
        }
        System.out.println();

        for (int i = 1; i <= 42; ++i) {
            if (i < now_mon_hajime_day_week) {
                PrintfColor.printSingleColor(34, ((Integer) (mae_mon_max_day - (now_mon_hajime_day_week - i) + 1)).toString() + " ");
            }
            else if (i < now_mon_max_day + now_mon_hajime_day_week) {
                PrintfColor.printSingleColor(32, ((Integer) (i - now_mon_hajime_day_week + 1)).toString() + " ");
            }
            else {
                PrintfColor.printSingleColor(35, ((Integer) (i - now_mon_hajime_day_week - now_mon_max_day + 1)).toString() + " ");
            }

            if (i % 7 == 0) {
                System.out.println();
            }
        }
    }
}

public class Demo {
    public static void main(String[] args) {
        MyDateMoNo.show();
    }
}
```
