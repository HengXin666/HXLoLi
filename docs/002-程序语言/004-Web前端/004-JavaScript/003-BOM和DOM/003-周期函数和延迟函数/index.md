# 周期函数和延迟函数

| 函数 | 说明 |
| --- | --- |
| `setInterval(函数,间隔时间)` | 按照给定的间隔时间重复执行给定的函数 |
| `clearInterval(周期函数)` | 清除给定的周期函数 |
| `setTimeout(函数，延迟时间）` | 在给定的延迟事件后执行一次给定的函数 |
| `clearTimeout(延迟函数)` | 清除给定的延迟函数 |

示例
```html
<body>
    <p id="hx">当前时间</p>
</body>
<script>
    var count = 0;

    let f = setInterval(function() {
        let now = new Date(); //创建一个日期对象，默认时间为系统当前时间
        let year = now.getFullYear(); //获取年份
        let month = now.getMonth() + 1; //获取月份，月份在0~11之间
        let date = now.getDate(); //获取日期是当前月的第几天
        let hour = now.getHours();//获取小时数
        let minute = now.getMinutes(); //获取分钟数
        let second = now.getSeconds(); //获取秒数
        let time = year + "-" + zerofill(month, 2) + "-" + zerofill(date, 2) + " " +
            zerofill(hour, 2) + ":" + zerofill(minute, 2) + ":" + zerofill(second, 2);
        document.getElementById("hx").innerText = time;
        ++count;
        if (count > 5)
            clearInterval(f);
    }, 1000);
    
    function zerofill(num, targetLen) {
        let str = num + "";
        while (str.length < targetLen) {
            str = "0" + str;
        }
        return str;
    }
</script>
```

当然这样也可以 (上面类似于匿名函数)
```html
<script>

function funname(params) {
  
}

setTimeout(funname, 1000); // 这样传参不用 括号

setTimeout("funname()", 1000); // 字符串形式需要括号
  
</script>
```