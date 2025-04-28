# 550. 游戏玩法分析 IV
题目: [550. 游戏玩法分析 IV](https://leetcode.cn/problems/game-play-analysis-iv/description/)

中等

```sql
Table: Activity

+--------------+---------+
| Column Name  | Type    |
+--------------+---------+
| player_id    | int     |
| device_id    | int     |
| event_date   | date    |
| games_played | int     |
+--------------+---------+
（player_id，event_date）是此表的主键（具有唯一值的列的组合）。
这张表显示了某些游戏的玩家的活动情况。
每一行是一个玩家的记录，他在某一天使用某个设备注销之前登录并玩了很多游戏（可能是 0）。
 

编写解决方案，报告在首次登录的第二天再次登录的玩家的 比率，四舍五入到小数点后两位。换句话说，你需要计算从首次登录日期开始至少连续两天登录的玩家的数量，然后除以玩家总数。

结果格式如下所示：

示例 1：

输入：
Activity table:
+-----------+-----------+------------+--------------+
| player_id | device_id | event_date | games_played |
+-----------+-----------+------------+--------------+
| 1         | 2         | 2016-03-01 | 5            |
| 1         | 2         | 2016-03-02 | 6            |
| 2         | 3         | 2017-06-25 | 1            |
| 3         | 1         | 2016-03-02 | 0            |
| 3         | 4         | 2018-07-03 | 5            |
+-----------+-----------+------------+--------------+
输出：
+-----------+
| fraction  |
+-----------+
| 0.33      |
+-----------+
解释：
只有 ID 为 1 的玩家在第一天登录后才重新登录，所以答案是 1/3 = 0.33
```

# 题解
我的代码

```sql
-- 求 首次登录日期开始至少连续两天登录的玩家的数量
-- 然后 除以玩家总数

-- 先找最早登录表, ROUND

select
    ROUND(
        COUNT(a.event_date) / (
                                select 
                                    COUNT(g.res) 
                                from (
                                    select 
                                        d.player_id as res
                                    from 
                                        Activity as d 
                                    group by 
                                        d.player_id
                                    ) as g
                                )
            , 2) as fraction
from
    Activity as a
join
    (select 
        MIN(b.event_date) as minDate,
        b.player_id
    from
        Activity as b
    group by
        b.player_id
    ) as c
on
    c.player_id = a.player_id
where 
    TIMESTAMPDIFF(DAY, c.minDate, a.event_date) = 1;
```

妙解:

先过滤出每个用户的首次登陆日期，然后左关联，筛选次日存在的记录的比例

```sql
select round(avg(a.event_date is not null), 2) fraction
from 
    (select player_id, min(event_date) as login
    from activity
    group by player_id) p 
left join activity a 
on p.player_id=a.player_id and datediff(a.event_date, p.login)=1

-- 作者：小数志
-- 链接：https://leetcode.cn/problems/game-play-analysis-iv/solutions/221603/mysqlleft-joinji-ke-by-luanz/
-- 来源：力扣（LeetCode）
-- 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```
