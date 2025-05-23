'''
文件名：YOLO-Circle-Position-dual.py
功能：使用YOLO进行目标检测，并获取检测框中心点的三维坐标（毫米）
测试使用的相机：Azure Kinect DK和Intel RealSense D435i
'''
import cv2
import multiprocessing as mp
import numpy as np
import pyrealsense2 as rs
import pyk4a
from pyk4a import Config, PyK4A
from ultralytics import YOLO
import random

def kinect_process():
    """Kinect检测进程"""
    # 独立模型实例
    model = YOLO('飞身上篮/预选赛/weights/yolo11n_cu_cir_2.pt', verbose=False)
    
    # Kinect初始化
    k4a = PyK4A(Config(
        color_resolution=pyk4a.ColorResolution.RES_720P,
        depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
        synchronized_images_only=True,
    ))
    k4a.start()
    
    try:
        cv2.namedWindow('Kinect Detection', cv2.WINDOW_AUTOSIZE)
        while True:
            capture = k4a.get_capture()
            if capture.color is None or capture.depth is None:
                continue

            # 图像处理
            color_img = cv2.cvtColor(capture.color, cv2.COLOR_BGRA2BGR)
            depth_img = capture.transformed_depth

            # YOLO检测
            results = model(color_img)
            boxes = results[0].boxes.xyxy.cpu().tolist()

            # 可视化
            display_img = color_img.copy()
            for box in boxes:
                mid_x = int((box[0]+box[2])//2)
                mid_y = int((box[1]+box[3])//2)
                
                # 坐标转换
                depth = get_average_depth_kinect(depth_img, mid_x, mid_y)
                if depth == 0: continue
                x = (mid_x - 640) * depth / 1000  # 1280x720分辨率
                y = (mid_y - 360) * depth / 1000
                z = depth

                # 绘制图形
                cv2.rectangle(display_img, (int(box[0]), int(box[1])), 
                            (int(box[2]), int(box[3])), (0,255,0), 2)
                text_y = int(box[1])-20 if int(box[1]) > 50 else int(box[3])+20
                for i, txt in enumerate([f"X:{x:.1f}mm", f"Y:{y:.1f}mm", f"Z:{z}mm"]):
                    cv2.putText(display_img, txt, (mid_x-80, text_y+i*25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

            cv2.imshow('Kinect Detection', display_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        k4a.stop()
        cv2.destroyAllWindows()

def realsense_process():
    """RealSense检测进程"""
    # 独立模型实例
    model = YOLO('飞身上篮/预选赛/weights/yolo11n_cu_cir_2.pt', verbose=False)
    
    # RealSense初始化
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    try:
        cv2.namedWindow('RealSense Detection', cv2.WINDOW_AUTOSIZE)
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # 图像处理
            color_img = np.asanyarray(color_frame.get_data())

            # YOLO检测
            results = model(color_img)
            boxes = results[0].boxes.xyxy.cpu().tolist()

            # 可视化
            display_img = color_img.copy()
            for box in boxes:
                mid_x = int((box[0]+box[2])//2)
                mid_y = int((box[1]+box[3])//2)
                
                # 坐标转换
                depth = get_average_depth_realsense(depth_frame, mid_x, mid_y)
                if depth < 0.1 or depth > 4.0: continue
                point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [mid_x, mid_y], depth)
                x, y, z = [coord*1000 for coord in point_3d]

                # 绘制图形
                cv2.rectangle(display_img, (int(box[0]), int(box[1])), 
                            (int(box[2]), int(box[3])), (0,255,0), 2)
                text_y = int(box[1])-20 if int(box[1]) > 50 else int(box[3])+20
                for i, txt in enumerate([f"X:{x:.1f}mm", f"Y:{y:.1f}mm", f"Z:{z:.1f}mm"]):
                    cv2.putText(display_img, txt, (mid_x-80, text_y+i*25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

            cv2.imshow('RealSense Detection', display_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

def get_average_depth_kinect(depth_img, center_x, center_y, sample_radius=3, sample_points=9):
    """
    在中心点周围采样多个点计算平均深度值 (Kinect版本)
    
    参数:
        depth_img: 深度图像
        center_x, center_y: 中心点坐标
        sample_radius: 采样半径
        sample_points: 采样点数量
    """
    height, width = depth_img.shape
    depths = []
    
    for _ in range(sample_points):
        # 在圆形区域内随机采样
        angle = random.uniform(0, 2 * np.pi)
        r = random.uniform(0, sample_radius)
        x = int(center_x + r * np.cos(angle))
        y = int(center_y + r * np.sin(angle))
        
        # 确保采样点在图像范围内
        x = max(0, min(x, width - 1))
        y = max(0, min(y, height - 1))
        
        depth = depth_img[y, x]
        if depth > 0:  # 只计入有效深度值
            depths.append(depth)
    
    return np.mean(depths) if depths else 0

def get_average_depth_realsense(depth_frame, center_x, center_y, sample_radius=3, sample_points=9):
    """
    在中心点周围采样多个点计算平均深度值 (RealSense版本)
    """
    depths = []
    
    for _ in range(sample_points):
        angle = random.uniform(0, 2 * np.pi)
        r = random.uniform(0, sample_radius)
        x = int(center_x + r * np.cos(angle))
        y = int(center_y + r * np.sin(angle))
        
        depth = depth_frame.get_distance(x, y)
        if 0.1 < depth < 4.0:  # RealSense的有效深度范围
            depths.append(depth)
    
    return np.mean(depths) if depths else 0

if __name__ == '__main__':
    # 创建并启动进程
    processes = [
        mp.Process(target=kinect_process),
        mp.Process(target=realsense_process)
    ]
    
    for p in processes:
        p.start()

    # 等待进程结束
    for p in processes:
        p.join()