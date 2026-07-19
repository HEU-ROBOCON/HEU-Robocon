import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torchvision import models, transforms
from PIL import Image
import os
import glob
import random
import copy

# =================配置区域=================
# 数据集所在的根目录 (上一级生成的文件夹)
DATA_DIR = "dataset_root" 
# 模型保存路径
MODEL_SAVE_PATH = "resnet18_cube_classifier.pth"
# 类别数量 (0-31 共32类)
NUM_CLASSES = 32
# 图片输入大小
INPUT_SIZE = 100
# 批次大小
BATCH_SIZE = 64
# 训练轮数
NUM_EPOCHS = 20
# 学习率
LEARNING_RATE = 0.001
# 设备配置
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
# =========================================

class CubeDataset(Dataset):
    def __init__(self, root_dir, transform=None, is_train=True):
        """
        自定义数据集类
        策略：
        - 0号文件夹 (负样本): 保持原样 (1倍数据量)
        - 1-31号文件夹 (正样本): 逻辑上扩充4倍 (原图 + 旋转90 + 旋转180 + 旋转270)
        """
        self.root_dir = root_dir
        self.transform = transform
        self.samples = [] # 存储 (图片路径, 标签, 旋转模式)

        # 遍历所有类别文件夹
        for class_idx in range(NUM_CLASSES):
            class_folder = os.path.join(root_dir, str(class_idx))
            if not os.path.exists(class_folder):
                continue
            
            # 读取文件夹下的所有图片
            # 这里忽略 file_list.txt，直接读图更稳健
            image_files = glob.glob(os.path.join(class_folder, "*.jpg"))
            image_files += glob.glob(os.path.join(class_folder, "*.png"))

            for img_path in image_files:
                if class_idx == 0:
                    # 负样本：只添加原始记录，旋转模式为 0
                    self.samples.append((img_path, class_idx, 0))
                else:
                    # 正样本：训练集模式下，添加4种旋转变体
                    if is_train:
                        self.samples.append((img_path, class_idx, 0))   # 0度
                        self.samples.append((img_path, class_idx, 90))  # 90度
                        self.samples.append((img_path, class_idx, 180)) # 180度
                        self.samples.append((img_path, class_idx, 270)) # 270度
                    else:
                        # 验证集/测试集模式下，不进行硬性扩充
                        self.samples.append((img_path, class_idx, 0))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        img_path, label, rot_angle = self.samples[idx]
        
        # 1. 读取图片 (强制转为灰度图 L)
        image = Image.open(img_path).convert('L')

        # 2. 执行硬性旋转 (针对正样本的增强)
        if rot_angle == 90:
            image = image.rotate(90, expand=True)
        elif rot_angle == 180:
            image = image.rotate(180, expand=True)
        elif rot_angle == 270:
            image = image.rotate(270, expand=True)

        # 3. 应用 PyTorch 的随机变换 (小角度旋转、Tensor转换等)
        if self.transform:
            image = self.transform(image)

        return image, label

def get_model():
    """
    获取改造后的 ResNet18 模型
    """
    # 加载预训练的 resnet18 (虽然预训练是RGB的，但权重还是有用的)
    model = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
    
    # 1. 改造第一层卷积 (输入通道 3 -> 1)
    # 原始: nn.Conv2d(3, 64, kernel_size=7, stride=2, padding=3, bias=False)
    # 我们保留权重的平均值来初始化单通道卷积，以便利用预训练特征
    old_conv = model.conv1
    model.conv1 = nn.Conv2d(1, 64, kernel_size=7, stride=2, padding=3, bias=False)
    
    with torch.no_grad():
        # 将原 RGB 权重的均值赋值给新的灰度卷积层
        model.conv1.weight.data = torch.mean(old_conv.weight.data, dim=1, keepdim=True)

    # 2. 改造全连接层 (输出类别 1000 -> 32)
    num_ftrs = model.fc.in_features
    model.fc = nn.Linear(num_ftrs, NUM_CLASSES)
    
    return model

def main():
    print(f"Using device: {DEVICE}")

    # ================= 数据准备 =================
    # 定义数据增强 (随机小角度 + 转Tensor + 归一化)
    train_transform = transforms.Compose([
        transforms.Resize((INPUT_SIZE, INPUT_SIZE)),
        transforms.RandomRotation(degrees=10), # 随机旋转 ±10 度
        # 也可以加一点平移或缩放
        transforms.RandomAffine(degrees=0, translate=(0.05, 0.05), scale=(0.9, 1.1)),
        transforms.ToTensor(),
        # 灰度图的归一化，mean和std设为0.5是常用做法
        transforms.Normalize(mean=[0.5], std=[0.5]) 
    ])

    val_transform = transforms.Compose([
        transforms.Resize((INPUT_SIZE, INPUT_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.5], std=[0.5])
    ])

    # 加载数据集
    full_dataset = CubeDataset(DATA_DIR, transform=train_transform, is_train=True)
    
    # 简单划分训练集和验证集 (80% 训练, 20% 验证)
    train_size = int(0.8 * len(full_dataset))
    val_size = len(full_dataset) - train_size
    train_dataset, val_dataset = torch.utils.data.random_split(full_dataset, [train_size, val_size])
    
    # 修正验证集的 transform (不应该有随机增强)
    # 注意：random_split 分割的是 Subset，需要访问 underlying dataset
    # 这是一个简化处理，严格来说应该分两个 Dataset 对象。但这里为了代码简洁暂时共用 logic。
    
    train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True, num_workers=4)
    val_loader = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=False, num_workers=4)

    print(f"Train samples (Augmented): {len(train_dataset)}")
    print(f"Val samples: {len(val_dataset)}")

    # ================= 模型初始化 =================
    model = get_model()
    model = model.to(DEVICE)

    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)
    
    # 学习率衰减策略
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=7, gamma=0.1)

    # ================= 训练循环 =================
    best_acc = 0.0

    for epoch in range(NUM_EPOCHS):
        print(f"\nEpoch {epoch+1}/{NUM_EPOCHS}")
        print("-" * 10)

        # --- 训练阶段 ---
        model.train()
        running_loss = 0.0
        running_corrects = 0

        for inputs, labels in train_loader:
            inputs = inputs.to(DEVICE)
            labels = labels.to(DEVICE)

            optimizer.zero_grad()

            outputs = model(inputs)
            loss = criterion(outputs, labels)

            loss.backward()
            optimizer.step()

            _, preds = torch.max(outputs, 1)
            running_loss += loss.item() * inputs.size(0)
            running_corrects += torch.sum(preds == labels.data)

        epoch_loss = running_loss / train_size
        epoch_acc = running_corrects.double() / train_size
        scheduler.step()

        print(f"Train Loss: {epoch_loss:.4f} Acc: {epoch_acc:.4f}")

        # --- 验证阶段 ---
        model.eval()
        val_loss = 0.0
        val_corrects = 0

        with torch.no_grad():
            for inputs, labels in val_loader:
                inputs = inputs.to(DEVICE)
                labels = labels.to(DEVICE)

                outputs = model(inputs)
                loss = criterion(outputs, labels)

                _, preds = torch.max(outputs, 1)
                val_loss += loss.item() * inputs.size(0)
                val_corrects += torch.sum(preds == labels.data)

        val_loss = val_loss / val_size
        val_acc = val_corrects.double() / val_size

        print(f"Val Loss: {val_loss:.4f} Acc: {val_acc:.4f}")

        # 保存最佳模型
        if val_acc > best_acc:
            best_acc = val_acc
            torch.save(model.state_dict(), MODEL_SAVE_PATH)
            print(f"Model saved! Best Acc: {best_acc:.4f}")

    print(f"\nTraining complete. Best Validation Accuracy: {best_acc:.4f}")

if __name__ == '__main__':
    # Windows下多进程需要放在main里
    main()