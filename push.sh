#!/bin/bash

# 设置 UTF-8 编码 (有点多余)
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

# 执行构建侧边栏的 Node 脚本
node ./scripts/generateSidebar.js

# 检查是否传入了参数
if [ $# -eq 0 ]; then
    git add .
    npx ts-node ./scripts/count-md-words.ts "Codeing..."
    echo "Please enter the submission information and call in the format of: ./push.sh 'commitMsg'"
else
    commit_message="$1"
    echo "commit: $commit_message"
    git add .
    npx ts-node ./scripts/count-md-words.ts "$commit_message"
    git add .
    git commit -m "$commit_message"
    git push origin
fi
