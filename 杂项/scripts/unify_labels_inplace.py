import os

def unify_labels_inplace(dataset_path, target_class_order=['red', 'green']):
    """
    在原目录中统一标签类别编号
    
    参数:
        dataset_path: 数据集路径
        target_class_order: 目标类别顺序 [red, green]表示红0绿1
    """
    # 读取原classes.txt
    classes_file = os.path.join(dataset_path, 'classes.txt')
    with open(classes_file, 'r') as f:
        current_classes = [line.strip() for line in f.readlines() if line.strip()]
    
    # 确定类别映射关系
    class_mapping = {}
    for i, cls in enumerate(current_classes):
        if cls in target_class_order:
            class_mapping[str(i)] = str(target_class_order.index(cls))
    
    # 更新classes.txt
    with open(classes_file, 'w') as f:
        f.write('\n'.join(target_class_order) + '\n')
    
    # 处理标签文件
    labels_dir = os.path.join(dataset_path, 'labels')
    for label_file in os.listdir(labels_dir):
        if label_file.endswith('.txt'):
            label_path = os.path.join(labels_dir, label_file)
            
            # 读取并修改标签
            with open(label_path, 'r') as f:
                lines = f.readlines()
            
            with open(label_path, 'w') as f:
                for line in lines:
                    parts = line.strip().split()
                    if len(parts) >= 5:
                        old_class = parts[0]
                        new_class = class_mapping.get(old_class, old_class)
                        f.write(f"{new_class} {' '.join(parts[1:])}\n")
    
    print(f"完成 {dataset_path} 的标签统一")
    print(f"新类别顺序: {target_class_order} (0: {target_class_order[0]}, 1: {target_class_order[1]})")

if __name__ == "__main__":
    # 配置路径和目标顺序 (红0绿1)
    target_order = ['red', 'green']
    
    print("正在统一dataset1标签 (原: green=0, red=1 → 红0绿1)")
    unify_labels_inplace("dataset1", target_order)
    
    print("\n正在统一dataset2标签 (原: red=0, green=1 → 红0绿1)") 
    unify_labels_inplace("dataset2", target_order)
    
    print("\n所有数据集标签已统一为: 红0绿1")
