
```sh
tmux new -s mc # 新建一个会话

tmux attach -t mc   #与之前建立的名为mc的会话窗口重新建立会话
```


## 与mc服务端交互的命令  

```sh
# 给玩家授予op权限
op <name>


# 解除授予给玩家的op权限
deop <name>


# 保存存档
save-all


# 关闭服务端
stop
```

## 自动备份

```sh
tail -f mc.fifo | bash run.sh > log.log
```
