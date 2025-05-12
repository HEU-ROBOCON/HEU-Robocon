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
from ultralytics import YOLO

model = YOLO('飞身上篮/预选赛/weights/yolo11n_cu_cir_2.pt') 

# 新增对齐对象和彩色相机内参
align_to = rs.stream.color
align = rs.align(align_to)
color_intrinsics = None  # 将使用对齐后的彩色相机内参

def get_3d_coordinates(box, depth_frame, intrinsics):
    """获取检测框中心点的三维坐标（毫米）"""
    mid_x = (box[0] + box[2]) // 2
    mid_y = (box[1] + box[3]) // 2
    
    # 坐标边界检查
    if (mid_x < 0 or mid_x >= intrinsics.width or 
        mid_y < 0 or mid_y >= intrinsics.height):
        return None
        
    depth = depth_frame.get_distance(int(mid_x), int(mid_y))
    if depth <= 0.1 or depth > 6.0:  # 过滤无效值和远距离噪声
        return None
    
    # 使用彩色相机内参反投影
    point_3d = rs.rs2_deproject_pixel_to_point(
        intrinsics, [mid_x, mid_y], depth
    )
    return [round(coord*1000, 1) for coord in point_3d] #米转毫米

def dectshow(org_img, boxs, depth_frame, intrinsics):
    """优化后的显示函数"""
    img = org_img.copy()
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
        
        # 计算显示位置
        text_x = int((box[0] + box[2])//2)  # 框中心X坐标
        text_y = int(box[1]) - 10          # 初始Y坐标（框上方）
        
        # 边界检测（窗口高度480）
        if text_y < 30:  # 如果接近顶部，显示在框下方
            text_y = int(box[3]) + 20
        
        # 逐行绘制坐标
        font_scale = 0.5
        line_height = 20
        for i, line in enumerate(text):
            y_pos = text_y + i*line_height
            cv2.putText(img, line, 
                    (text_x - 50, y_pos),  # 水平居中
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    font_scale, (0,255,255), 1, 
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

            # 显示优化后的检测结果
            dectshow(color_image, boxes, depth_frame, color_intrinsics)

            # 退出控制
            if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
                break
                
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()