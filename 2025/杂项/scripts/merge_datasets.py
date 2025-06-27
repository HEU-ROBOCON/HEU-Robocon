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
        if f.lower().endswith(('.jpg', '.png', '.jpeg')):
            # 支持多种文件名格式
            base_name = os.path.splitext(f)[0]
            if base_name.startswith(('loop', 'image')):
                try:
                    num = int(''.join(filter(str.isdigit, base_name)))
                    if num > max_num:
                        max_num = num
                except ValueError:
                    continue
    
    # 复制并重命名dataset2文件
    print(f"正在合并dataset2，从{max_num+1}开始...")
    
    # 先处理图片文件
    image_files = []
    src_image_dir = os.path.join(dataset2_path, 'images')
    for f in os.listdir(src_image_dir):
        if f.lower().endswith(('.jpg', '.png', '.jpeg')):
            base_name, ext = os.path.splitext(f)
            try:
                new_num = max_num + 1
                new_name = f"loop{new_num}{ext}"
                max_num += 1
                # 复制图片文件
                shutil.copy2(
                    os.path.join(src_image_dir, f),
                    os.path.join(output_path, 'images', new_name))
                image_files.append(base_name)
            except ValueError:
                pass
    
    # 再处理对应的标签文件
    src_label_dir = os.path.join(dataset2_path, 'labels')
    for f in os.listdir(src_label_dir):
        if f.endswith('.txt'):
            base_name = os.path.splitext(f)[0]
            if base_name in image_files:  # 只复制有对应图片的标签
                try:
                    # 找到对应的新编号
                    idx = image_files.index(base_name)
                    new_name = f"loop{max_num - len(image_files) + idx + 1}.txt"
                    # 复制标签文件
                    shutil.copy2(
                        os.path.join(src_label_dir, f),
                        os.path.join(output_path, 'labels', new_name))
                except ValueError:
                    pass
    
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
