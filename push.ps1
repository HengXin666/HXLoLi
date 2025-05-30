# 设置编码为 UTF-8
$OutputEncoding = New-Object -typename System.Text.UTF8Encoding

# 构建目录文件
node .\scripts\generateSidebar.js

# 检查是否有输入参数
if ($args.Length -eq 0) {
    git add .
    npx ts-node .\scripts\count-md-words.ts "Codeing..."
    # 请输入提交信息, 调用格式为 .\push.ps1 "提交信息"
    Write-Host "Please enter the submission information and call in the format of: .\push.ps1 'commitMsg'"
} else {
    $commit_message = $args[0]
    Write-Host "commit: $commit_message"
    git add .
    npx ts-node .\scripts\count-md-words.ts "$commit_message"
    git add .
    git commit -m "$commit_message"
    # 生成提交信息, 下次统计...
    git push origin
}
