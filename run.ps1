git pull origin main

Write-Host ""
Write-Host "请选择启动模式:"
Write-Host "  1) 默认启动"
Write-Host "  2) 同步私有仓库并启动 (明文预览, 退出时自动清理)"
Write-Host "  3) 同步私有仓库并加密启动 (测试插件解密, 退出时自动清理)"
Write-Host ""
$choice = Read-Host "请输入 [1/2/3] (默认 1)"

function Invoke-Cleanup {
    Write-Host ""
    Write-Host "🧹 正在清理私有页面文件..."
    try { npm run clean:private } catch { Write-Host "⚠️  清理警告: $_" }
    Write-Host "✅ 清理完成!"
}

if ($choice -eq "2") {
    Write-Host ""
    Write-Host "🔓 同步私有仓库并启动 (明文模式)..."
    try {
        npm run dev:private
    } finally {
        Invoke-Cleanup
    }
} elseif ($choice -eq "3") {
    Write-Host ""
    Write-Host "🔐 同步私有仓库并加密启动 (插件测试模式)..."

    # 查找公钥
    $PubKey = $null
    $candidates = @(
        "..\HXLoLi-imouto-public-key.pem",
        "..\hxloli_public.pem",
        ".\hxloli_public.pem"
    )
    foreach ($c in $candidates) {
        if (Test-Path $c) {
            $PubKey = $c
            break
        }
    }

    if (-not $PubKey) {
        Write-Host ""
        Write-Host "⚠️  未找到 RSA 公钥文件!"
        Write-Host "   请将公钥放在以下任一位置:"
        Write-Host "     ..\HXLoLi-imouto-public-key.pem"
        Write-Host "     ..\hxloli_public.pem"
        Write-Host "     .\hxloli_public.pem"
        Write-Host ""
        Write-Host "   或从私钥导出公钥:"
        Write-Host "     openssl rsa -in ..\HXLoLi-imouto\private-key.pem -pubout -o ..\HXLoLi-imouto-public-key.pem"
        exit 1
    }

    Write-Host "🔑 使用公钥: $PubKey"

    $PrivateRepoPath = "..\HXLoLi-imouto"
    if (Test-Path $PrivateRepoPath) {
        Write-Host "📥 拉取私有仓库最新内容..."
        try { Push-Location $PrivateRepoPath; git pull; Pop-Location } catch {}
    } else {
        Write-Host "❌ 私有仓库不存在: $PrivateRepoPath"
        exit 1
    }

    # 以 deploy 模式加密
    Write-Host "🔒 以 deploy 模式加密私有页面..."
    node scripts/encrypt-private.mjs --source $PrivateRepoPath --mode deploy --pubkey $PubKey

    # 重新生成 sidebar
    node scripts/generateSidebar.js

    Write-Host ""
    Write-Host "🚀 启动开发服务器 (加密模式)..."
    Write-Host "   请确保已安装 HXLoLi-NaGaMe 浏览器插件并完成 GitHub 授权"
    Write-Host ""

    try {
        npm run start
    } finally {
        Invoke-Cleanup
    }
} else {
    Write-Host ""
    Write-Host "正在启动..."
    npm run start
}
