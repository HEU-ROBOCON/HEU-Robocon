import os
import shutil

def merge_datasets(dataset1_path, dataset2_path, output_path):
    """
    合并两个数据集，保持文件连续编号
    
    参数:
        dataset1_path: 第一个数据集路径
        dataset2_path: 第二个数据集路径
        output_path: 合并后输出路径
    """
    # 创建输出目录
    os.makedirs(os.path.join(output_path, 'images'), exist_ok=True)
    os.makedirs(os.path.join(output_path, 'labels'), exist_ok=True)
    
    # 复制dataset1内容
    print("正在复制dataset1...")
    for folder in ['images', 'labels']:
        src = os.path.join(dataset1_path, folder)
        dst = os.path.join(output_path, folder)
        shutil.copytree(src, dst, dirs_exist_ok=True)
    
    # 获取dataset1中最大的文件编号
    max_num = 0
    for f in os.listdir(os.path.join(dataset1_path, 'images')):
        if f.startswith('loop') and f.endswith('.jpg'):
            num = int(f[4:-4])
            if num > max_num:
                max_num = num
    
    # 复制并重命名dataset2文件
    print(f"正在合并dataset2，从loop{max_num+1}开始...")
    for folder in ['images', 'labels']:
        src_dir = os.path.join(dataset2_path, folder)
        dst_dir = os.path.join(output_path, folder)
        
        for f in os.listdir(src_dir):
            if f.startswith('loop') and (f.endswith('.jpg') or f.endswith('.txt')):
                # 提取原始编号
                orig_num = int(f[4:-4])
                # 计算新编号
                new_num = orig_num + max_num + 1
                # 构建新文件名
                new_name = f"loop{new_num}{os.path.splitext(f)[1]}"
                # 复制文件
                shutil.copy2(
                    os.path.join(src_dir, f),
                    os.path.join(dst_dir, new_name))
    
    # 合并classes.txt
    print("合并classes.txt...")
    with open(os.path.join(output_path, 'classes.txt'), 'w') as f_out:
        with open(os.path.join(dataset1_path, 'classes.txt'), 'r') as f_in:
            f_out.write(f_in.read())
    
    print(f"合并完成！输出目录: {output_path}")
    print(f"总图像数量: {len(os.listdir(os.path.join(output_path, 'images')))}")
    print(f"总标签数量: {len(os.listdir(os.path.join(output_path, 'labels')))}")

if __name__ == "__main__":
    # 配置路径
    dataset1_path = "dataset1"
    dataset2_path = "dataset2"
    output_path = "dataset"
    
    # 执行合并
    merge_datasets(dataset1_path, dataset2_path, output_path)
