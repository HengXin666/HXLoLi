# jQuery validate
## jQuery validate 简介
jQuery Validate 插件为表单提供了强大的验证功能，让客户端表单验证变得更简单，同时提供了大量的定制选项，满足应用程序各种需求。

| 规则         | 描述                             |
|--------------|----------------------------------|
| required:true | 必须输入的字段。                 |
| remote       | 使用 AJAX 方法验证输入值。       |
| email:true   | 必须输入正确格式的电子邮件。     |
| url:true     | 必须输入正确格式的网址。         |
| date:true    | 必须输入正确格式的日期。日期校验 IE6 出错，慎用。 |
| dateISO:true | 必须输入正确格式的日期（ISO），例如：2021-10-05。只验证格式，不验证有效性。 |
| number:true  | 必须输入合法的数字（负数，小数）。 |
| digits:true  | 必须输入整数。                   |
| creditcard   | 必须输入合法的信用卡号。         |
| equalTo:"#ID" | 输入值必须和 #ID 相同。         |
| accept       | 输入拥有合法后缀名的字符串（上传文件的后缀）。 |
| maxlength:15 | 输入长度最多是15的字符串（汉字算一个字符）。 |
| minlength:8  | 输入长度最小是8的字符串（汉字算一个字符）。  |
| rangelength:[8,15] | 输入长度必须介于8和15之间的字符串（汉字算一个字符）。 |
| range:[8,20] | 输入值必须介于8和20之间。         |
| max:100      | 输入值不能大于100。              |
| min:0        | 输入值不能小于0。                |

<div style="margin-top: 80px;">

---
</div>

## jQuery validate 使用
```html
<body>
    <form class="registerForm" id="registerForm" method="post" action="">
        <div>
            <label for="username">账号</label>
            <input id="username" name="username" minlength="8" maxlength="15" type="text" required>
        </div>
        <div>
            <label for="password">密码</label>
            <input id="password" name="password" minlength="8" maxlength="20" type="password" required>
        </div>
        <div>
            <label for="password">确认密码</label>
            <input id="confirm" name="confirm" type="password">
        </div>
        <div>
            <label for="email">邮箱</label>
            <input id="email" type="email" name="email" required>
        </div>
        <div>
            <label for="url">个人主页</label>
            <input id="url" type="url" name="url">
        </div>
        <div>
            <input class="submit" type="submit" value="注册">
        </div>
    </form>
</body>

<!-- 注意导入的顺序 -->
<script type="text/javascript" src="./js/jquery-3.7.1.js"></script>
<script type="text/javascript" src="./js/jquery.validate.js"></script>
<!-- jQuery validate 提供了国际化的支持，可以将 messages_zh.js 引入，以支持中文。 -->
<script type="text/javascript" src="./js/messages_zh.js"></script>

<script type="text/javascript">
    // 使用默认的验证提示
    $.validator.setDefaults({
        // 提交表单的事件处理
        submitHandler: function () {
            // 表单验证成功后执行的后续操作
            alert("表单验证通过,可以将数据发送至服务器了");
        }
    });
    $(function () {
        // 表单验证
        $("#registerForm").validate();
    })
</script>
```

<div style="margin-top: 80px;">

---
</div>

## 自定义校验规则

示例
```html
<body>
    <form class="registerForm" id="registerForm" method="get" action="">
        <div>
            <label for="username">账号</label>
            <input id="username" name="username" type="text">
        </div>
        <div>
            <label for="password">密码</label>
            <input id="password" name="password" type="password">
        </div>
        <div>
            <label for="password">确认密码</label>
            <input id="confirm" name="confirm" type="password">
        </div>
        <div>
            <label for="email">邮箱</label>
            <input id="email" type="email" name="email">
        </div>
        <div>
            <label for="url">个人主页</label>
            <input id="url" type="url" name="url">
        </div>
        <div>
            <input class="submit" type="submit" value="注册">
        </div>
    </form>
</body>


<script type="text/javascript" src="./js/jquery-3.7.1.js"></script>
<script type="text/javascript" src="./js/jquery.validate.js"></script>
<script type="text/javascript" src="./js/messages_zh.js"></script>

<script type="text/javascript">
    $(() => {
        //表单验证
        $("#registerForm").validate({
            // 自定义规则
            rules: {
                username: { // 用户名
                    required: true,
                    maxlength: 10,
                    minlength: 4
                },
                password: { // 密码
                    required: true,
                    maxlength: 18,
                    minlength: 8
                },
                confirm: { // 确认密码
                    equalTo: "#password"
                },
                email: { // 邮箱
                    required: true,
                    email: true
                },
                url: { // 网址
                    required: true,
                    url: true
                }
            },
            messages: { // 效验规则对应的提示信息
                username: { // 用户名
                    required: "请输入用户名!",
                    maxlength: "名称最大只能10个字",
                    minlength: "太短了这个名称"
                },
                password: { // 密码
                    required: "你不设置密码吗",
                    maxlength: "这么长记得住吗",
                    minlength: "这么短, 马上就被破解了"
                },
                confirm: { // 确认密码
                    required: "再输入一次!",
                    equalTo: "刚刚输入的密码就忘记了?"
                },
                email: { // 邮箱
                    required: "喂, 邮箱",
                    email: "这个不是邮箱吧"
                },
                url: { // 网址
                    required: "留个网站啊!",
                    url: "这是网站吗?"
                }
            },
            submitHandler: form => form.submit()
        });
    })
</script>
```