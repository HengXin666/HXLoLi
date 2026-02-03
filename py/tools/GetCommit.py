import subprocess

def generate_commit_ts(output_file='./data/gitVersion.ts'):
    try:
        # 使用 git 命令获取最新的 short commit ID
        commit_id = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'],
            stderr=subprocess.STDOUT
        ).decode('utf-8').strip()

        # 构建 TypeScript 文件内容
        content = f"export const LATEST_COMMIT_ID = '{commit_id}';\n"

        # 写入文件
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(content)

        print(f"✅ 成功! 最新的 Commit ID [{commit_id}] 已保存至 {output_file}")

    except subprocess.CalledProcessError:
        print("❌ 错误: 无法获取 Commit ID。请确保当前目录是一个 Git 仓库。")
    except Exception as e:
        print(f"❌ 发生未知错误: {e}")

if __name__ == "__main__":
    # 当前目录应该是项目根目录
    generate_commit_ts()
