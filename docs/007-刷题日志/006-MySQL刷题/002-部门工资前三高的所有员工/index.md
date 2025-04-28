# 185. 部门工资前三高的所有员工
困难
```sql
表:Employee

+--------------+---------+
| Column Name  | Type    |
+--------------+---------+
| id           | int     |
| name         | varchar |
| salary       | int     |
| departmentId | int     |
+--------------+---------+
id 是该表的主键列(具有唯一值的列)。
departmentId 是 Department 表中 ID 的外键（reference 列）。
该表的每一行都表示员工的ID、姓名和工资。它还包含了他们部门的ID。

表: Department

+-------------+---------+
| Column Name | Type    |
+-------------+---------+
| id          | int     |
| name        | varchar |
+-------------+---------+
id 是该表的主键列(具有唯一值的列)。
该表的每一行表示部门ID和部门名。

公司的主管们感兴趣的是公司每个部门中谁赚的钱最多。一个部门的 高收入者 是指一个员工的工资在该部门的 不同 工资中 排名前三 。

编写解决方案，找出每个部门中 收入高的员工 。

以 任意顺序 返回结果表。

返回结果格式如下所示。
```

## 示例 1:
```
输入: 
Employee 表:
+----+-------+--------+--------------+
| id | name  | salary | departmentId |
+----+-------+--------+--------------+
| 1  | Joe   | 85000  | 1            |
| 2  | Henry | 80000  | 2            |
| 3  | Sam   | 60000  | 2            |
| 4  | Max   | 90000  | 1            |
| 5  | Janet | 69000  | 1            |
| 6  | Randy | 85000  | 1            |
| 7  | Will  | 70000  | 1            |
+----+-------+--------+--------------+
Department  表:
+----+-------+
| id | name  |
+----+-------+
| 1  | IT    |
| 2  | Sales |
+----+-------+
输出: 
+------------+----------+--------+
| Department | Employee | Salary |
+------------+----------+--------+
| IT         | Max      | 90000  |
| IT         | Joe      | 85000  |
| IT         | Randy    | 85000  |
| IT         | Will     | 70000  |
| Sales      | Henry    | 80000  |
| Sales      | Sam      | 60000  |
+------------+----------+--------+
解释:
在IT部门:
- Max的工资最高
- 兰迪和乔都赚取第二高的独特的薪水
- 威尔的薪水是第三高的

在销售部:
- 亨利的工资最高
- 山姆的薪水第二高
- 没有第三高的工资，因为只有两名员工
```

## 题解
### 噢滴呆马

5% (3000ms+)

```sql
-- Write your MySQL query statement below
-- 查询前三, 按部门 分组 排序 展示前三 (下面代码非常lj, 效率非常低下, 只是能过qwq)
select b.name as Department,
a.name as Employee,
a.salary as Salary
from Employee as a
join Department as b on a.departmentId = b.id
where a.salary >= (
if ((
select DISTINCT x.salary from Employee as x
join Department as y on x.departmentId = y.id
where y.id = b.id
order by x.salary desc
limit 2, 1) is null, 0, (
select DISTINCT x.salary from Employee as x
join Department as y on x.departmentId = y.id
where y.id = b.id
order by x.salary desc
limit 2, 1))) ;
```

说明: 对每一个员工的工资进行匹对

匹对它所在部门的第三高的工资, 如果大于等于, 则输出

对于部门不满3人等情况下, 会不存在第三高的工资, 所以需要 判断是否为null

由于我不太会在这里使用临时变量, 并且临时变量如果遇到null, 也是不会赋值的, 所以查询了两次qwq...

## 自定义变量解

41.78% (954ms)

```sql
SELECT dep.Name Department, emp.Name Employee, emp.Salary
FROM (-- 自定义变量RANK, 查找出 每个部门工资前三的排名
        SELECT te.DepartmentId, te.Salary,
               CASE 
                    WHEN @pre = DepartmentId THEN @rank:= @rank + 1
                    WHEN @pre := DepartmentId THEN @rank:= 1
               END AS `RANK`
        FROM (SELECT @pre:=null, @rank:=0)tt,
             (-- (部门,薪水)去重,根据 部门(升),薪水(降) 排序
                 SELECT DepartmentId,Salary
                 FROM Employee
                 GROUP BY DepartmentId,Salary
                 ORDER BY DepartmentId,Salary DESC
             )te
       )t
INNER JOIN Department dep ON t.DepartmentId = dep.Id
INNER JOIN Employee emp ON t.DepartmentId = emp.DepartmentId and t.Salary = emp.Salary and t.`RANK` <= 3
ORDER BY t.DepartmentId, t.Salary DESC-- t 结果集已有序,根据该集合排序


-- 作者：gaazau
-- 链接：https://leetcode.cn/problems/department-top-three-salaries/solutions/7696/mysql-zi-ding-yi-bian-liang-jie-fa-by-zill-2/
-- 来源：力扣（LeetCode）
-- 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```

## 窗口函数

62.4% (818ms)

```sql
select
    b.name as Department,
    c.name as Employee,
    c.salary as Salary
from 
    (
        select
            a.departmentId,
            a.name,
            a.salary,
            dense_rank() over (partition by departmentId
                                order by salary desc) as sort
        from Employee as a
    ) as c
left join
    Department as b
on c.departmentId = b.id
where c.sort <= 3;
```

## 纯语法实现

建议直接看题解qwq

```sql
SELECT
    Department.NAME AS Department,
    e1.NAME AS Employee,
    e1.Salary AS Salary 
FROM
    Employee AS e1,Department 
WHERE
    e1.DepartmentId = Department.Id 
    AND 3 > (SELECT  count( DISTINCT e2.Salary ) 
             FROM    Employee AS e2 
             WHERE    e1.Salary < e2.Salary     AND e1.DepartmentId = e2.DepartmentId     ) 
ORDER BY Department.NAME,Salary DESC;

-- 作者：肥猫布里奇高
-- 链接：https://leetcode.cn/problems/department-top-three-salaries/solutions/11461/185-bu-men-gong-zi-qian-san-gao-de-yuan-gong-by-li/
-- 来源：力扣（LeetCode）
-- 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```
