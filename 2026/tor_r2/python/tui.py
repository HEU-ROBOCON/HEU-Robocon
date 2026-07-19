import torch
import torch.nn as nn
import torchvision.models as models
import torchvision.transforms as transforms
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import os
import json
import warnings
warnings.filterwarnings('ignore')

# 定义全局变量
NUM_CLASSES = 32  # 根据你的实际类别数修改
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def get_model():
    """
    获取改造后的 ResNet18 模型
    """
    # 加载预训练的 resnet18
    model = models.resnet18(weights=None)  # 不加载预训练权重，只获取结构
    
    # 1. 改造第一层卷积 (输入通道 3 -> 1)
    old_conv = model.conv1
    model.conv1 = nn.Conv2d(1, 64, kernel_size=7, stride=2, padding=3, bias=False)
    
    # 2. 改造全连接层 (输出类别 1000 -> 32)
    num_ftrs = model.fc.in_features
    model.fc = nn.Linear(num_ftrs, NUM_CLASSES)
    
    return model

def load_trained_model(model_path, device=None):
    """
    加载训练好的模型权重
    """
    if device is None:
        device = DEVICE
    
    # 1. 创建模型结构
    model = get_model()
    
    # 2. 加载权重
    try:
        # 尝试不同的加载方式
        if model_path.endswith('.pth'):
            checkpoint = torch.load(model_path, map_location=device, weights_only=False)
        else:
            checkpoint = torch.load(model_path, map_location=device)
    except Exception as e:
        print(f"❌ 模型文件加载失败: {e}")
        raise
    
    # 3. 检查检查点类型
    if isinstance(checkpoint, dict):
        if 'state_dict' in checkpoint:
            # 包含优化器状态等的完整检查点
            state_dict = checkpoint['state_dict']
        elif 'model_state_dict' in checkpoint:
            # 包含模型权重的检查点
            state_dict = checkpoint['model_state_dict']
        else:
            # 直接是state_dict
            state_dict = checkpoint
    else:
        # 可能是直接的模型对象
        try:
            model = checkpoint
            model.to(device)
            model.eval()
            return model
        except:
            # 如果上面失败，尝试作为state_dict处理
            state_dict = checkpoint
    
    # 4. 处理键名不匹配的问题
    new_state_dict = {}
    for k, v in state_dict.items():
        # 去掉可能的'module.'前缀
        if k.startswith('module.'):
            new_key = k[7:]
        else:
            new_key = k
        
        # 尝试加载权重
        try:
            # if 'fc.weight' in new_key or 'fc.bias' in new_key:
            #     # 跳过全连接层权重（因为输入输出维度可能不匹配）
            #     continue
            new_state_dict[new_key] = v
        except:
            pass
    
    # 5. 加载权重
    try:
        missing_keys, unexpected_keys = model.load_state_dict(new_state_dict, strict=False)
        if missing_keys:
            print(f"⚠️ 缺少的键: {missing_keys}")
        if unexpected_keys:
            print(f"⚠️ 意外的键: {unexpected_keys}")
    except Exception as e:
        print(f"❌ 权重加载失败: {e}")
        print(f"尝试使用strict=False加载...")
        model.load_state_dict(new_state_dict, strict=False)
    
    model.to(device)
    model.eval()
    
    print(f"✅ 模型加载成功，来自: {model_path}")
    print(f"📊 总参数量: {sum(p.numel() for p in model.parameters()):,}")
    print(f"🚀 使用设备: {device}")
    
    return model

def preprocess_image(image_path, img_size=100):
    """
    预处理图像（灰度）
    """
    # 图像预处理
    transform = transforms.Compose([
        transforms.Resize((img_size, img_size)),
        transforms.Grayscale(num_output_channels=1),  # 转为灰度
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.5], std=[0.5])  # 灰度图归一化
    ])
    
    # 加载图像
    if not os.path.exists(image_path):
        raise FileNotFoundError(f"❌ 图像不存在: {image_path}")
    
    try:
        img = Image.open(image_path).convert('RGB')  # 先以RGB加载
        img_tensor = transform(img)
        img_tensor = img_tensor.unsqueeze(0)  # 添加batch维度
        return img_tensor
    except Exception as e:
        raise ValueError(f"❌ 图像加载失败: {e}")

def get_default_font():
    """
    获取默认字体
    """
    try:
        # 尝试几种常见字体
        font_paths = [
            "arial.ttf",
            "arialbd.ttf",
            "C:/Windows/Fonts/arial.ttf",
            "C:/Windows/Fonts/simhei.ttf",  # 中文
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",  # Linux
        ]
        
        for font_path in font_paths:
            if os.path.exists(font_path):
                try:
                    return ImageFont.truetype(font_path, 20)
                except:
                    continue
        
        # 如果都没有找到，使用默认字体
        return ImageFont.load_default()
    except:
        return ImageFont.load_default()

def visualize_prediction(image_path, results, save_path=None, show=True):
    """
    可视化预测结果
    """
    # 确保图片存在
    if not os.path.exists(image_path):
        print(f"❌ 图片不存在: {image_path}")
        return None
    
    try:
        # 加载并转换图像为RGB模式
        img = Image.open(image_path).convert('RGB')
    except Exception as e:
        print(f"❌ 无法打开图片: {e}")
        return None
    
    # 创建绘图对象
    draw = ImageDraw.Draw(img)
    
    # 获取字体
    font = get_default_font()
    
    # 计算文本大小
    try:
        # 获取字体高度
        if hasattr(font, 'getsize'):
            text_height = font.getsize('A')[1]
        else:
            text_height = 20
    except:
        text_height = 20
    
    # 显示预测结果
    text_y = 10
    
    for result in results:
        if 'rank' in result:
            text = f"{result['rank']}. {result['class_name']}: {result.get('percentage', '0%')}"
        else:
            text = f"{result.get('class_name', 'Unknown')}: {result.get('confidence_percentage', '0%')}"
        
        # 绘制文本背景
        try:
            if hasattr(font, 'getsize'):
                text_width = font.getsize(text)[0]
            else:
                text_width = len(text) * 10
        except:
            text_width = len(text) * 10
            
        # 绘制半透明背景
        try:
            draw.rectangle([(5, text_y-2), (5+text_width+10, text_y+text_height+2)], 
                          fill=(0, 0, 0, 128))
        except:
            # 如果不支持RGBA，使用纯色
            draw.rectangle([(5, text_y-2), (5+text_width+10, text_y+text_height+2)], 
                          fill=(0, 0, 0))
        
        # 绘制文本
        try:
            # 尝试使用元组颜色
            draw.text((10, text_y), text, fill=(255, 255, 255), font=font)
        except:
            try:
                # 如果失败，尝试整数颜色
                draw.text((10, text_y), text, fill=255, font=font)
            except:
                # 如果还失败，使用最简单的文本绘制
                draw.text((10, text_y), text, fill="white")
        
        text_y += 30
    
    # 显示图片
    if show:
        try:
            img.show()
        except:
            print("⚠️ 无法显示图片，但已生成结果")
    
    # 保存图片
    if save_path:
        try:
            # 确保保存目录存在
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            img.save(save_path)
            print(f"📸 结果已保存到: {save_path}")
        except Exception as e:
            print(f"❌ 保存图片失败: {e}")
    
    return img

def predict_single_image(model, image_path, class_names=None, topk=3, return_probs=True):
    """
    单张图像推理
    """
    # 预处理图像
    try:
        img_tensor = preprocess_image(image_path)
    except Exception as e:
        print(f"❌ 图像预处理失败: {e}")
        return None if return_probs else None
    
    # 移动到设备
    img_tensor = img_tensor.to(DEVICE)
    
    # 推理
    with torch.no_grad():
        try:
            logits = model(img_tensor)
        except Exception as e:
            print(f"❌ 模型推理失败: {e}")
            return None if return_probs else None
        
        if return_probs:
            # 计算softmax概率
            probs = torch.nn.functional.softmax(logits, dim=1)
            topk_probs, topk_indices = torch.topk(probs, min(topk, NUM_CLASSES), dim=1)
            
            # 转换为numpy
            topk_probs = topk_probs.cpu().numpy()[0]
            topk_indices = topk_indices.cpu().numpy()[0]
        else:
            # 只返回类别
            predictions = torch.argmax(logits, dim=1)
            return predictions.cpu().numpy()
    
    # 准备结果
    results = []
    for i in range(len(topk_indices)):
        idx = topk_indices[i]
        prob = float(topk_probs[i])
        
        if class_names and isinstance(class_names, (list, tuple, dict)):
            if isinstance(class_names, dict) and idx in class_names:
                class_name = class_names[idx]
            elif isinstance(class_names, (list, tuple)) and idx < len(class_names):
                class_name = class_names[idx]
            else:
                class_name = f"Class_{idx}"
        else:
            class_name = f"Class_{idx}"
        
        results.append({
            'rank': i + 1,
            'class_id': int(idx),
            'class_name': class_name,
            'probability': prob,
            'percentage': f"{prob*100:.2f}%"
        })
    
    return results

def main():
    """
    主函数：推理程序
    """
    import argparse
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='ResNet18 灰度图像分类推理')
    parser.add_argument('--model_path', type=str, required=True, 
                       help='.pth 模型文件路径')
    parser.add_argument('--image_path', type=str, required=True,
                       help='单张图像路径')
    parser.add_argument('--class_names', type=str, default=None,
                       help='类别名称列表，用逗号分隔')
    parser.add_argument('--class_file', type=str, default=None,
                       help='类别名称文件 (txt或json)')
    parser.add_argument('--topk', type=int, default=3,
                       help='显示前K个预测结果')
    parser.add_argument('--save_path', type=str, default='prediction_result.jpg',
                       help='结果保存路径')
    parser.add_argument('--no_visualize', action='store_true',
                       help='不显示可视化结果')
    
    args = parser.parse_args()
    
    # 1. 加载类别名称
    class_names = None
    
    if args.class_names:
        # 从命令行参数加载类别名称
        class_names = [name.strip() for name in args.class_names.split(',')]
        print(f"📋 从命令行加载了 {len(class_names)} 个类别")
    
    elif args.class_file and os.path.exists(args.class_file):
        # 从文件加载类别名称
        try:
            if args.class_file.endswith('.json'):
                with open(args.class_file, 'r', encoding='utf-8') as f:
                    class_data = json.load(f)
                    if isinstance(class_data, dict):
                        class_names = {int(k): v for k, v in class_data.items()}
                    else:
                        class_names = {i: name for i, name in enumerate(class_data)}
            elif args.class_file.endswith('.txt'):
                with open(args.class_file, 'r', encoding='utf-8') as f:
                    lines = [line.strip() for line in f if line.strip()]
                    class_names = {i: name for i, name in enumerate(lines)}
            
            if class_names:
                if isinstance(class_names, dict):
                    print(f"📋 从文件加载了 {len(class_names)} 个类别")
                else:
                    print(f"📋 从文件加载了 {len(class_names)} 个类别")
        except Exception as e:
            print(f"⚠️ 加载类别文件失败: {e}")
            class_names = None
    
    # 2. 加载模型
    try:
        model = load_trained_model(args.model_path)
    except Exception as e:
        print(f"❌ 模型加载失败: {e}")
        return
    
    # 3. 推理
    print(f"\n🔍 处理单张图像: {args.image_path}")
    results = predict_single_image(
        model, args.image_path, 
        class_names=class_names, 
        topk=args.topk
    )
    
    if results is None:
        print("❌ 推理失败")
        return
    
    # 4. 显示结果
    print("\n📊 预测结果:")
    for result in results:
        print(f"  {result['rank']}. {result['class_name']} (ID:{result['class_id']}): {result['percentage']}")
    
    # 5. 可视化
    if not args.no_visualize:
        visualize_prediction(
            args.image_path, 
            results, 
            save_path=args.save_path,
            show=True
        )

def simple_inference(model_path, image_path, class_names=None):
    """
    简化版推理函数
    """
    # 加载模型
    model = load_trained_model(model_path)
    
    # 推理
    results = predict_single_image(
        model, 
        image_path, 
        class_names=class_names,
        topk=3
    )
    
    if results:
        print("\n📊 预测结果:")
        for r in results:
            print(f"  {r['rank']}. {r['class_name']}: {r['percentage']}")
        
        # 可视化
        visualize_prediction(
            image_path, 
            results, 
            save_path=f"result_{os.path.basename(image_path)}",
            show=True
        )
    else:
        print("❌ 推理失败")
    
    return results

if __name__ == "__main__":
    # 示例用法
    import sys
    
    if len(sys.argv) > 1:
        # 命令行方式
        main()
    else:
        # 直接在代码中调用
        print("🚀 运行简单推理示例...")
        
        # 设置参数
        model_path = r".\resnet18_cube_classifier.pth"
        image_path = r".\0123_7.jpg"
        
        # 定义类别名称（根据你的实际类别修改）
        class_names = [
            f"Class_{i}" for i in range(NUM_CLASSES)
        ]
        # 或者从文件中加载
        # class_names = {0: "Cube_A", 1: "Cube_B", 2: "Cube_C", ...}
        
        # 运行推理
        results = simple_inference(
            model_path=model_path,
            image_path=image_path,
            class_names=class_names
        )