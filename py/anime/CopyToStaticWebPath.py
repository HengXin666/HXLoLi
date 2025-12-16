import os
import shutil # 导入 shutil 库, 它提供了高级的文件操作

def main() -> None:
    """
    将文件从数据目录【拷贝】到静态web路径下。
    这个脚本可以从任何位置安全地运行。
    """

    # 1. 获取当前脚本文件所在的目录
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # 2. 以脚本目录为基准, 构建源和目标路径
    data_root_path = os.path.join(script_dir, "data")

    # 假设你的目录结构是 HXLoLi/py/anime/your_script.py
    # 我们需要找到 HXLoLi/static/anime
    # 从脚本目录 (py/anime) 向上回溯两层到项目根目录 (HXLoLi)
    # 这里的路径计算需要根据你的实际目录结构来确定
    # 如果脚本在 HXLoLi/py/anime/ 下
    project_root = os.path.dirname(os.path.dirname(script_dir))
    static_path = os.path.join(project_root, "static", "anime")

    print(f"源目录: {data_root_path}")
    print(f"目标目录: {static_path}")

    # 3. 确保目标目录存在
    if not os.path.exists(static_path):
        os.makedirs(static_path)
        print(f"已创建目标目录: {static_path}")

    # 4. 检查源目录是否存在
    if not os.path.exists(data_root_path):
        print(f"错误: 源目录 '{data_root_path}' 不存在。")
        return

    # 5. 拷贝文件
    files_copied = 0
    for file_name in os.listdir(data_root_path):
        if file_name.endswith(".json"):
            source_file = os.path.join(data_root_path, file_name)
            destination_file = os.path.join(static_path, file_name)

            print(f"正在拷贝: {source_file} -> {destination_file}")

            # 使用 shutil.copy2() 来执行拷贝操作
            # copy2 会同时拷贝文件内容和元数据(如时间戳)
            # 如果你只关心内容, 也可以用 shutil.copy()
            shutil.copy2(source_file, destination_file)
            files_copied += 1

    if files_copied > 0:
        print(f"成功拷贝了 {files_copied} 个 .json 文件。")
    else:
        print("在源目录中没有找到 .json 文件进行拷贝。")

if __name__ == "__main__":
    main()
