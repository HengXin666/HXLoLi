# Date 类

| 方法 | 说明 |
| --- | --- |
| `getDate()` | 获取当前对象表示的日期是一个月中的第几天 |
| `getDay()` | 获取当前对象表示的日期是一周中的第几天 |
| `getHours()` | 获取当前对象表示的日期中的小时数 |
| `getMinutes()` | 获取当前对象表示的日期中的分钟数 |
| `getSeconds()` | 获取当前对象表示的日期中的秒数 |
| `getMonth()` | 获取当前对象表示的日期中的月份 |
| `getFullYear()` | 获取当前对象表示的日期中的年份 |
| `getTime()` | 获取当前日期对象对应的毫秒数 |

示例
```js
let now = new Date(); //创建一个日期对象，默认时间为系统当前时间
let year = now.getFullYear(); //获取年份
let month = now.getMonth() + 1; //获取月份，月份在0~11之间
let date = now.getDate(); //获取日期是当前月的第几天
let hour = now.getHours();//获取小时数
let minute = now.getMinutes(); //获取分钟数
let second = now.getSeconds(); //获取秒数
let time = year + "-" + zerofill(month, 2) + "-" + zerofill(date, 2) + " " +
    zerofill(hour, 2) + ":" + zerofill(minute, 2) + ":" + zerofill(second, 2);
console.log(time);

let weekday = now.getDay(); //获取当前日期是一周的第几天：一周的开始是周日，值为0
console.log(weekday);
now.setMonth(month);
now.setDate(0);
console.log(now.getDate());//需要注意的是：在取当前月最大天数时，需要将月份重新设置，日期设置为0即可

function zerofill(num, targetLen) {
    let str = num + "";
    while (str.length < targetLen) {
        str = "0" + str;
    }
    return str;
}
```