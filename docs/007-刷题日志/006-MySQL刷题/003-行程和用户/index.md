# 262. 行程和用户
原题: [262. 行程和用户](https://leetcode.cn/problems/trips-and-users/description/)

## 答案

```sql
-- 求 取消率 Cancellation Rate 需要四舍五入保留 两位小数 
-- 取消率 = (被司机或乘客取消的非禁止用户生成的订单数量) / 
--          (非禁止用户生成的订单总数)

-- 非禁止用户（乘客和司机都必须未被禁止）即 banned 为 No 的用户

-- 1. 先求当日的非禁止用户生成的订单总数
-- 2. 顺便统计 当日的被司机或乘客取消的非禁止用户生成的订单数量
-- 3. 做商即可 四舍五入保留 两位小数
select 
    t.request_at as `Day`,
    ROUND(COUNT(IF(u1.banned = 'No' AND u2.banned = 'No' AND t.status != 'completed', 1, null)) / COUNT(IF(u1.banned = 'No' AND u2.banned = 'No', 1, null)), 2) as `Cancellation Rate`
from Trips as t
left join Users as u1
on t.client_id = u1.users_id
left join Users as u2
on t.driver_id = u2.users_id
where ABS(TIMESTAMPDIFF(DAY, t.request_at, '2013-10-02')) <= 1
group by t.request_at
having `Cancellation Rate` IS NOT NULL;
```
