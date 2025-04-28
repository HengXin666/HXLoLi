# 更新容器

## 1. 修改.yaml配置文件
```Bash
vim [文件]
```

```修改版本号
services:
  blossom:
    image: jasminexzzz/blossom:1.12.0

```

## 2. 更新
```Bash
docker compose -f blossom-mysql8.yaml up -d --remove-orphans
```

## 3. 启动

```Bash
docker logs blossom-backend
```

```
版本: [1.12.0-SNAPSHOT]
```