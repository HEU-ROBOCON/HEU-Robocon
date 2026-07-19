import torch
import torch.nn as nn
from torchvision import models

class CustomResNet18(nn.Module):
    def __init__(self):
        super(CustomResNet18, self).__init__()
        base_model = models.resnet18(weights=None)
        
        self.conv1 = nn.Conv2d(1, 64, kernel_size=7, stride=2, padding=3, bias=False)
        self.bn1 = base_model.bn1
        self.relu = base_model.relu
        self.maxpool = base_model.maxpool

        self.layer1 = base_model.layer1
        self.layer2 = base_model.layer2
        self.layer3 = base_model.layer3
        self.layer4 = base_model.layer4

        self.avgpool = base_model.avgpool
        self.fc = nn.Linear(base_model.fc.in_features, 32)

    def forward(self, x):
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.relu(x)
        x = self.maxpool(x)

        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)

        x = self.avgpool(x)
        
        # ==========================================
        # 终极修复：绝对不要用 x.size(0)，也不用 flatten
        # 用 -1 强制告诉 ONNX 这里是动态的！
        # ==========================================
        x = x.view(-1, 512)
        
        x = self.fc(x)
        return x

def export_onnx():
    model = CustomResNet18()
    model_path = "resnet18_cube_classifier.pth"
    
    try:
        model.load_state_dict(torch.load(model_path, map_location="cpu"), strict=False)
        print("权重加载成功")
    except Exception as e:
        print(f"权重加载警告: {e}")

    model.eval()
    dummy_input = torch.randn(2, 1, 100, 100)
    output_onnx = "cube_model.onnx"

    torch.onnx.export(
        model, 
        dummy_input, 
        output_onnx,
        input_names=['input'],
        output_names=['output'],
        # 配置 dynamic_shapes 让输入也支持动态
        dynamic_shapes={'x': {0: 'batch_size'}}, 
        opset_version=18 
    )
    print(f"✅ 模型已成功导出为: {output_onnx}")

if __name__ == "__main__":
    export_onnx()