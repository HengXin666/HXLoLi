#!/bin/bash
# 为了方便, 你可以给权限:
# chmod 777 ./run.sh
# chmod 777 ./push.sh

# 先 git pull 拉取 main
echo "正在拉取 main 分支..."
git pull origin main

echo ""
echo "请选择启动模式:"
echo "  1) 默认启动"
echo "  2) 同步私有仓库并启动 (明文预览, 退出时自动清理)"
echo "  3) 同步私有仓库并加密启动 (测试插件解密, 退出时自动清理)"
echo ""
read -rp "请输入 [1/2/3] (默认 1): " choice

CLEANED=0
cleanup() {
    if [ "$CLEANED" -eq 0 ]; then
        CLEANED=1
        echo ""
        echo "🧹 正在清理私有页面文件..."
        npm run clean:private || true
        echo "🔄 重新生成 sidebar (移除私有分类)..."
        node scripts/generateSidebar.js || true
        echo "✅ 清理完成!"
    fi
}

case "${choice:-1}" in
    2)
        echo ""
        echo "🔓 同步私有仓库并启动 (明文模式)..."
        trap cleanup EXIT INT TERM HUP
        npm run dev:private
        ;;
    3)
        echo ""
        echo "🔐 同步私有仓库并加密启动 (插件测试模式)..."

        # 查找公钥: 优先项目同级目录, 其次 HXLoLi-imouto 同级
        PUBKEY=""
        for candidate in \
            "../HXLoLi-imouto-public-key.pem" \
            "../hxloli_public.pem" \
            "./hxloli_public.pem"; do
            if [ -f "$candidate" ]; then
                PUBKEY="$candidate"
                break
            fi
        done

        if [ -z "$PUBKEY" ]; then
            echo ""
            echo "⚠️  未找到 RSA 公钥文件!"
            echo "   请将公钥放在以下任一位置:"
            echo "     ../HXLoLi-imouto-public-key.pem"
            echo "     ../hxloli_public.pem"
            echo "     ./hxloli_public.pem"
            echo ""
            echo "   或从私钥导出公钥:"
            echo "     openssl rsa -in ../HXLoLi-imouto/private-key.pem -pubout -o ../HXLoLi-imouto-public-key.pem"
            exit 1
        fi

        echo "🔑 使用公钥: $PUBKEY"

        trap cleanup EXIT INT TERM HUP

        # Step 1: 同步私有仓库
        PRIVATE_REPO_PATH="../HXLoLi-imouto"
        if [ -d "$PRIVATE_REPO_PATH" ]; then
            echo "📥 拉取私有仓库最新内容..."
            (cd "$PRIVATE_REPO_PATH" && git pull) || true
        else
            echo "❌ 私有仓库不存在: $PRIVATE_REPO_PATH"
            exit 1
        fi

        # Step 2: 以 deploy 模式加密
        echo "🔒 以 deploy 模式加密私有页面..."
        node scripts/encrypt-private.mjs \
            --source "$PRIVATE_REPO_PATH" \
            --mode deploy \
            --pubkey "$PUBKEY"

        # Step 3: 重新生成 sidebar
        node scripts/generateSidebar.js

        # Step 4: 启动 dev server
        echo ""
        echo "🚀 启动开发服务器 (加密模式)..."
        echo "   请确保已安装 HXLoLi-NaGaMe 浏览器插件并完成 GitHub 授权"
        echo ""
        npm run start
        ;;
    *)
        echo ""
        echo "正在启动..."
        npm run start
        ;;
esac
