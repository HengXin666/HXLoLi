# 设置编码为 UTF-8
$OutputEncoding = New-Object -typename System.Text.UTF8Encoding

# 构建目录文件
node .\scripts\generateSidebar.js

# 检查是否有输入参数
if ($args.Length -eq 0) {
    git add .
    # 请输入提交信息, 调用格式为 .\push.ps1 "提交信息"
    Write-Host "Please enter the submission information and call in the format of: .\push.ps1 'commitMsg'"
} else {
    $commit_message = $args[0]
    Write-Host "commit: $commit_message"
    git add .
    git commit -m "$commit_message"
    git push origin
}
