'''
文件名：YOLO-Circle-Position-rs.py
功能：使用YOLO进行目标检测，并获取检测框中心点的三维坐标（毫米）
测试使用的相机：Intel RealSense D435i
'''
import pyrealsense2 as rs
import numpy as np
import cv2
import math
import torch
import sys
import random
from ultralytics import YOLO

model = YOLO('飞身上篮/预选赛/models/weights/yolo11n_cu_cir_2.pt') 

# 对齐对象和彩色相机内参
align_to = rs.stream.color
align = rs.align(align_to)
color_intrinsics = None  # 将使用对齐后的彩色相机内参

def get_3d_coordinates(box, depth_frame, intrinsics, sample_radius=3, sample_points=9):
    """
    获取检测框中心点的三维坐标（毫米）
    对深度进行邻域多点采样平均

    参数：
        box: 检测框坐标 [x1, y1, x2, y2]
        depth_frame: RealSense深度帧
        intrinsics: 相机内参
        sample_radius: 采样半径（像素）
        sample_points: 采样点数量
    返回:
        [x, y, z]: 三维坐标（毫米），None表示无效
    """
    # 计算检测框中心点
    mid_x = int((box[0] + box[2]) // 2)
    mid_y = int((box[1] + box[3]) // 2)
    
    # 确保采样半径不会超出检测框
    box_width = int(box[2] - box[0])
    box_height = int(box[3] - box[1])
    sample_radius = min(sample_radius, box_width // 4, box_height // 4)

    # 坐标边界检查
    if (mid_x < 0 or mid_x >= intrinsics.width or 
        mid_y < 0 or mid_y >= intrinsics.height):
        return None
    
    # 收集有效深度值
    valid_depths = []
    for _ in range(sample_points):
        # 在圆形区域内随机采样
        angle = random.uniform(0, 2 * np.pi)
        r = random.uniform(0, sample_radius)
        sample_x = int(mid_x + r * np.cos(angle))
        sample_y = int(mid_y + r * np.sin(angle))
        
        # 确保采样点在检测框内
        if (box[0] <= sample_x <= box[2] and 
            box[1] <= sample_y <= box[3]):
            depth = depth_frame.get_distance(sample_x, sample_y)
            if 0.1 < depth < 6.0:  # 过滤无效值和远距离噪声
                valid_depths.append(depth)
    
    # 如果没有有效深度值，返回None
    if not valid_depths:
        return None
    
    # 计算平均深度
    avg_depth = np.mean(valid_depths)
    
    # 使用中心点坐标和平均深度进行3D反投影
    point_3d = rs.rs2_deproject_pixel_to_point(
        intrinsics, [mid_x, mid_y], avg_depth
    )

    return [round(coord*1000, 1) for coord in point_3d] #米转毫米

def dectshow(org_img, boxs, depth_frame, intrinsics):
    """
    显示函数

    参数：
        org_img: 原始图像
        boxs: 检测框列表，每个框为[x1, y1, x2, y2]
        depth_frame: RealSense深度帧
        intrinsics: 相机内参
    """
    img = org_img.copy()
    height, width = img.shape[:2]  # 获取图像尺寸

    for box in boxs:
        # 绘制检测框
        cv2.rectangle(img, (int(box[0]), int(box[1])), 
                    (int(box[2]), int(box[3])), (0, 255, 0), 2)
        
        # 获取三维坐标
        coordinates = get_3d_coordinates(box, depth_frame, intrinsics)
        if not coordinates:
            continue
            
        x, y, z = coordinates
        text = [f"X:{x}mm", f"Y:{y}mm", f"Z:{z}mm"]

        # 文本显示参数
        font_scale = 0.5
        line_height = 20
        text_width = max([cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 
                                        font_scale, 1)[0][0] for line in text])
        text_height = line_height * len(text)
        
        # 计算初始显示位置
        text_x = int((box[0] + box[2]) // 2)
        text_y = int(box[1] - 10)  # 默认显示在框上方
        
        # 水平方向边界检查
        if text_x - text_width//2 < 0:  # 太靠左
            text_x = text_width//2 + 5
        elif text_x + text_width//2 > width:  # 太靠右
            text_x = width - text_width//2 - 5
            
        # 垂直方向边界检查
        if text_y - text_height < 0:  # 上方空间不足
            text_y = int(box[3] + text_height)  # 显示在框下方
        
        # 如果下方也没有足够空间
        if text_y + text_height > height:
            text_y = height - text_height - 5
        
        # 绘制文本
        for i, line in enumerate(text):
            y_pos = text_y + i*line_height
            cv2.putText(img, line,
                        (text_x - text_width//2, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        font_scale, (0, 255, 255), 1,
                        lineType=cv2.LINE_AA)
            
    cv2.imshow('Detection', img)

if __name__ == "__main__":
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
    profile = pipeline.start(config)
    
    # 获取对齐后的彩色相机内参
    color_profile = profile.get_stream(rs.stream.color)
    color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()

    try:
        while True:
            frames = pipeline.wait_for_frames()

            # 执行深度到彩色图的对齐
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            
            # YOLO检测
            results = model(color_image)
            boxes = results[0].boxes.xyxy.cpu().tolist()

            # 显示检测结果
            dectshow(color_image, boxes, depth_frame, color_intrinsics)

            # 退出控制
            if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
                break
                
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()