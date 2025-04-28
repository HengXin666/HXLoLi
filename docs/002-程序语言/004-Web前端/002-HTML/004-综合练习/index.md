# 综合练习
## 完成登录表单

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>综合练习</title>
</head>
<body>
    <form action="" method="get">
        <div> <!-- 使用<label>行级标签, 可以使得点击"账号"这俩个字的时候, 变成等价于点击输入框 -->
            <label>账号:<input type="text" name = "username" maxlength="5" placeholder="请输入账号"></label>
        </div>
        <div>
            <label>密码:<input type="password" name="password" placeholder="请输入密码"></label>
        </div>
        <input type="submit" value="登录"> <!-- 这样才是提交 -->
    </form>
</body>
</html>
```

## 完成注册表单

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>综合练习</title>
</head>
<body>
    <form action="" method="get">
        <div>
            <label>账号:<input type="text" name = "username" maxlength="5" placeholder="请输入账号"></label>
        </div>
        <div>
            <label>密码:<input type="password" name="password" placeholder="请输入密码"></label>
        </div>
        <div>
            <label>确认密码:<input type="password" name="password" placeholder="请输入密码"></label>
        </div>
        <div>
            性别
            <input type="radio" name="sex" value="0">男
            <input type="radio" name="sex" value="1">女
            <input type="radio" name="sex" value="2" checked>未知
        </div>
        <div>
            <label for="country">国家</label>
            <select id="country" name="country">
                <option value="">请选择</option>
                <option value="CN">中国</option>
                <option value="JP">小日子</option>
                <option value="?">外国</option>
            </select>
        </div>
        <input type="submit" value="登录">
    </form>
</body>
</html>
```

## 完成查询界面

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>综合练习</title>
</head>
<body>
    姓名<input type="text"> <label for="country">性别</label>
    <select id="country" name="country">
        <option value="">请选择</option>
        <option value="B">男</option>
        <option value="G">女</option>
        <option value="?">其他</option>
    </select>
    <input type="submit" value="提交">
    <div>
        <table border="2">
            <thead>
                <tr>
                    <td>姓名</td>
                    <td>性别</td>
                    <td>年龄</td>
                    <td>成绩</td>
                </tr>
            </thead>
            <tbody>
                <tr>
                    <td>李四</td>
                    <td>男</td>
                    <td>22</td>
                    <td>66</td>
                </tr>
                <tr>
                    <td>张三</td>
                    <td>男</td>
                    <td>25</td>
                    <td>67</td>
                </tr>
                <tr>
                    <td>老六</td>
                    <td>男</td>
                    <td>38</td>
                    <td>88</td>
                </tr>
            </tbody>
        </table>
    </div>
</body>
</html>
```
