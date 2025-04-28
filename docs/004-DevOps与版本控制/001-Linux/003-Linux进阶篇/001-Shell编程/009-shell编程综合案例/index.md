# shell编程综合案例
需求:
- 每天凌晨2:30备份数据库day01到/data/backup/db;
- 备份开始和备份结束的时候，要给出相应的提示信息；
- 备份后的文件要以备份时间为文件名，并打包成tar.gz的格式，比如: 2024-4-20_230201.tar.gz;
- 在备份的同时，要检查是否有10天前备份的数据库文件，如果有就将其删除。

实践:
1. 把脚本放到`/usr/sbin `, 为什么要放在sbin目录，因为这个目录是root用户具备执行的权限。

代码:
```bash
#!\bin\bash
# 定义备份目录
BACKUP=~/data/backup/db

# 获取当前的时间
DATETIME=$(date +%Y-%m-%d_%H%M%S)

# 看一下日期
# echo $DATETIME

# 数据库主机地址
HOST=localhost

# 数据库用户名
DB_USER=root
# 数据库密码
DP_PW=root
# 需要备份的数据库
DATABASE=day01

# 创建备份目录, 如果不存在则创建
[ ! -d "${BACKUP}/${DATETIME}" ] && mkdir -p "${BACKUP}/${DATETIME}"

# 备份数据库
mysqldump -u${DB_USER} -p${DB_PW} --host=${HOST} -q -R --databases ${DATABASE} |
    gzip > ${BACKUP}/${DATETIME}/$DATETIME.sql.gz

# 打包成 tar.gz
cd ${BACKUP}
tar -zcvf $DATETIME.tar.gz ${DATETIME}
# 删除对应的备份目录
rm -rf ${BACKUP}/${DATETIME}

# 删除10天前的备份文件
find ${BACKUP} -atime +10 -name "*.tar.gz" -exec rm -rf {} \;
echo "数据库${DATABASE}备份成功!"
```

然后是写入定时任务:

- `crontab -e`

```bash
30 2 * * * /usr/sbin/mysql_db_backup.sh
```
