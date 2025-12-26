#!/bin/bash
# 为了方便, 你可以给权限:
# chmod 777 ./run.sh
# chmod 777 ./push.sh

# 先 git pull 拉取 main
echo "正在拉取 main 分支..."
git pull origin main

echo "正在启动..."
npm run start