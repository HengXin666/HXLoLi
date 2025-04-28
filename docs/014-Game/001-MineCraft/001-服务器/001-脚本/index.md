# say

```json
tellraw @a [{"text":"2","color":"#5d2156"},{"text":"2","color":"#6e2867"},{"text":"2","color":"#772f6f"},{"text":"2","color":"#85387d"},{"text":"2","color":"#914389"},{"text":"2","color":"#9f4e96"},{"text":"2","color":"#ad5aa4"}]

{"text": "", "color": ""}

tellraw @a [{"text": "[ロリっ子]: ", "color": "gold"}, {"text": "働きましょう~", "color": "green"}]

tellraw @a [{"text": "[ロリっ子]: ", "color": "gold"}, {"text": "还有", "color": "green"}, {"text": "10 分钟", "color": "yellow"}, {"text": "就清理掉落物啦~", "color": "green"}]
```

# 定时清理掉落物

```sh
#!/bin/bash

fifo_file="mc.fifo"

echo "say §a 已经启动扫地小萝莉~" > $fifo_file

savestate=0
tag=0

while true; do
    echo "list" > mc.fifo
    latest=$(tail -1 log.log)
    if [[ ${latest:33:11} == "There are 0" ]]; then
        savestate=0
        tag=0
        echo "say §dおやすみニャーさい~" > $fifo_file
    else
        if [ $tag -eq 0 ]; then
        echo "say §dいらっニャーいませ~ 掃除ロリっ子やる気満々~" > $fifo_file
            tag=1
        fi
        savestate=1
    fi

    # 有人, 才会清理垃圾
    if [ $savestate -eq 1 ]; then
        echo "say §e还有 §b10 分钟 §e就清理掉落物啦~" > $fifo_file
        sleep 10m # 每10分钟进行一次清理

        sleep 60 # 1 min
        echo "say §e还有 §b1 分钟 §e就清理掉落物啦~" > $fifo_file # 发送通知, §a 表示绿色

        for ((i=10; i>0; i--)); do
            echo "say §a还有 §e$i §a就清理掉落物啦~" > $fifo_file # 发送通知, §a 表示绿色
            sleep 1 # 1 s
        done

        # 执行清理掉落物命令
        echo "say §c正在清理掉落物..." > $fifo_file # 发送清理开始的通知，§c 表示红色
        echo "kill @e[type=item]" > $fifo_file # 清理掉落物
        echo "say §a清理掉落物结束!" > $fifo_file # 清理完成的通知，§a 表示绿色
    else
        sleep 10m # 间隔10 min检查是否有人
    fi
done
```

# 定时备份

```sh
mkdir backup
backup_num=6 # 保留6个备份
savestate=0
while true; do
    sleep 5m # 每五分钟检查一次是否有人
    echo "list" > mc.fifo
    latest=$(tail -1 log.log)
    if [[ ${latest:33:11} == "There are 0" ]]; then
        haspeople=0
    else
        haspeople=1
        savestate=1
        echo "say §aご機嫌よ, 检测到玩家, 启动备份模式~" > mc.fifo
    fi
    if [ $savestate -eq 1 ]; then
        {
            echo "say §e开始备份 {"
            echo save-off
            echo save-all
            tar -cf backup/$(date +%Y%m%d_%H%M%S).tar.gz world
            echo save-on
            echo "say §e} §a// 备份结束"
        } > mc.fifo # 一次备份
        cd backup
        dir_num=$(ls -l | wc -l)
        if [ $dir_num -gt $backup_num ]; then # 删除过旧的备份
            num=$(expr $dir_num - $backup_num)
            ls -tr | head -$num | xargs -i -n1 rm {}
        fi
        cd ..
        if [ $haspeople -eq 0 ]; then # 若没人则停止备份
            savestate=0
            echo "say §eご苦労様でした, 没人我下班啦~ 停止备份~" > mc.fifo
        fi
        sleep 15m # 每次备份至少间隔 20分钟
    fi
done
```
