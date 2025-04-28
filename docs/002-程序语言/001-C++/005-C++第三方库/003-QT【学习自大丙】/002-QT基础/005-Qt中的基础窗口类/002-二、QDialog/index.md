# 二、QDialog
## 2.1 常用API
> 对话框类是QWidget类的子类, 处理继承自父类的属性之外, 还有一些自己所特有的属性, 常用的一些API函数如下:

```C++
// 构造函数
QDialog::QDialog(QWidget *parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());

// 模态显示窗口
[virtual slot] int QDialog::exec();
// 隐藏模态窗口, 并且解除模态窗口的阻塞, 将 exec() 的返回值设置为 QDialog::Accepted
[virtual slot] void QDialog::accept();
// 隐藏模态窗口, 并且解除模态窗口的阻塞, 将 exec() 的返回值设置为 QDialog::Rejected
[virtual slot] void QDialog::reject();
// 关闭对话框并将其结果代码设置为r。finished()信号将发出r;
// 如果r是QDialog::Accepted 或 QDialog::Rejected，则还将分别发出accept()或Rejected()信号。
[virtual slot] void QDialog::done(int r);

[signal] void QDialog::accepted();
[signal] void QDialog::rejected();
[signal] void QDialog::finished(int result);
```

## 2.2 常用使用方法

```C++
场景介绍:
    1. 有两个窗口, 主窗口和一个对话框子窗口
    2. 对话框窗口先显示, 根据用户操作选择是否显示主窗口
```

- 关于对话框窗口类的操作:

```C++
void TestDialog::on__acceptBtn_clicked() {
    this->accept();
}


void TestDialog::on__rejectBtn_clicked() {
    this->reject();
}


void TestDialog::on__donBtn_clicked() {
    this->done(666);
}
```

- 根据用户针对对话框窗口的按钮操作, 进行相应的逻辑处理:

```C++
// 创建对话框对象
TestDialog dlg;
int ret = dlg.exec(); // 返回值..
if(ret == QDialog::Accepted) {
    qDebug() << "accept button clicked...";
    // 显示主窗口code...
} else if(ret == QDialog::Rejected) {
    qDebug() << "reject button clicked...";
    // 不显示主窗口code...
} else {
    // ret == 666
    qDebug() << "done button clicked...: " << ret;
    // 根据需求进行逻辑处理
}
```
