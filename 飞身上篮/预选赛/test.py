'''
文件名：YOLO-Circle-Position.py
功能：使用YOLO进行目标检测，并获取检测框中心点的三维坐标（毫米）
相机：Azure Kinect DK
'''
import cv2
import numpy as np
import sys
import torch
from ultralytics import YOLO
import pyk4a
from pyk4a import Config, PyK4A

model = YOLO('飞身上篮/预选赛/circle_position/weights/yolo11n_cu_cir_2.pt')

def get_3d_coordinates(box, depth_image, depth_scale):
    """获取检测框中心点的三维坐标（毫米）"""
    mid_x = int((box[0] + box[2]) // 2)
    mid_y = int((box[1] + box[3]) // 2)
    
    # 确保坐标在图像范围内
    if (mid_x >= depth_image.shape[1] or mid_y >= depth_image.shape[0] or
        mid_x < 0 or mid_y < 0):
        return None
        
    depth = depth_image[mid_y, mid_x]
    if depth == 0:
        return None
        
    # Azure Kinect的深度值已经是毫米单位,无需转换
    x = (mid_x - depth_image.shape[1]/2) * depth * depth_scale
    y = (mid_y - depth_image.shape[0]/2) * depth * depth_scale
    z = depth
    
    return [round(x, 1), round(y, 1), round(z, 1)]

def dectshow(org_img, boxs, depth_image, depth_scale):
    """优化后的显示函数"""
    img = org_img.copy()
    for box in boxs:
        # 绘制检测框
        cv2.rectangle(img, (int(box[0]), int(box[1])), 
                    (int(box[2]), int(box[3])), (0, 255, 0), 2)
        
        # 获取三维坐标
        coordinates = get_3d_coordinates(box, depth_image, depth_scale)
        if not coordinates:
            continue
            
        x, y, z = coordinates
        text = [f"X:{x}mm", f"Y:{y}mm", f"Z:{z}mm"]
        
        # 计算显示位置
        text_x = int((box[0] + box[2])//2)
        text_y = int(box[1]) - 10
        
        if text_y < 30:
            text_y = int(box[3]) + 20
        
        # 逐行绘制坐标
        font_scale = 0.5
        line_height = 20
        for i, line in enumerate(text):
            y_pos = text_y + i*line_height
            cv2.putText(img, line, 
                    (text_x - 50, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    font_scale, (0,255,255), 1, 
                    lineType=cv2.LINE_AA)

    cv2.imshow('Detection', img)

if __name__ == "__main__":
    # 初始化Azure Kinect
    k4a = PyK4A(
        Config(
            color_resolution=pyk4a.ColorResolution.RES_720P,
            depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
            synchronized_images_only=True,
        )
    )
    k4a.start()

    # 计算深度比例因子
    depth_scale = 1/1000.0  # Azure Kinect深度单位已经是毫米

    try:
        while True:
            # 获取帧
            capture = k4a.get_capture()
            if capture.color is None or capture.depth is None:
                continue
                
            # 获取彩色图像和深度图像
            color_image = capture.color
            depth_image = capture.depth

            # 将彩色图像从BGRA转为BGR格式
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2BGR)
            
            # YOLO检测
            results = model(color_image)
            boxes = results[0].boxes.xyxy.cpu().tolist()

            # 显示检测结果
            dectshow(color_image, boxes, depth_image, depth_scale)

            # 退出控制
            if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
                break
                
    finally:
        k4a.stop()
        cv2.destroyAllWindows()