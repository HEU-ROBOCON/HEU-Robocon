'''
文件名：Dual_Camera_Detection_Fixed.py
功能：修复双相机显示问题，优化线程控制
'''
import cv2
import numpy as np
import threading
import pyrealsense2 as rs
from ultralytics import YOLO
import pyk4a
from pyk4a import Config, PyK4A

# 使用Event进行线程控制
exit_event = threading.Event()

# 初始化YOLO模型（建议使用绝对路径）
model = YOLO('飞身上篮/预选赛/weights/yolo11n_cu_cir_2.pt', verbose=False)

def kinect_thread():
    """处理Azure Kinect相机的线程"""
    k4a = PyK4A(Config(
        color_resolution=pyk4a.ColorResolution.RES_720P,
        depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
        synchronized_images_only=True,
    ))
    try:
        k4a.start()
        cv2.namedWindow('Kinect Detection', cv2.WINDOW_AUTOSIZE)
        
        while not exit_event.is_set():
            capture = k4a.get_capture()
            if capture.color is None or capture.depth is None:
                continue

            # 确保颜色格式正确转换
            color_image = cv2.cvtColor(capture.color, cv2.COLOR_BGRA2BGR)
            if color_image.size == 0:
                continue

            # YOLO检测
            results = model(color_image)
            boxes = results[0].boxes.xyxy.cpu().tolist()

            # 绘制结果
            img = color_image.copy()
            for box in boxes:
                # 框中心计算
                mid_x = int((box[0]+box[2])//2)
                mid_y = int((box[1]+box[3])//2)
                
                # 获取深度值（Kinect原始深度单位为毫米）
                depth = capture.transformed_depth[mid_y, mid_x]
                if depth == 0:
                    continue
                
                # 坐标转换
                x = (mid_x - 640) * depth / 1000  # 假设彩色图像宽度为1280
                y = (mid_y - 360) * depth / 1000  # 假设彩色图像高度为720
                z = depth

                # 绘制检测框
                cv2.rectangle(img, (int(box[0]), int(box[1])), 
                            (int(box[2]), int(box[3])), (0,255,0), 2)
                
                # 文本位置调整
                text_y = int(box[1])-10 if int(box[1]) > 40 else int(box[3])+20
                for i, text in enumerate([f"X:{x:.1f}mm", f"Y:{y:.1f}mm", f"Z:{z}mm"]):
                    cv2.putText(img, text, (mid_x-80, text_y+i*20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 1)

            cv2.imshow('Kinect Detection', img)
            if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
                exit_event.set()

    finally:
        k4a.stop()
        cv2.destroyWindow('Kinect Detection')

def realsense_thread():
    """处理RealSense相机的线程"""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 降低帧率
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        profile = pipeline.start(config)
        align = rs.align(rs.stream.color)
        intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        cv2.namedWindow('RealSense Detection', cv2.WINDOW_AUTOSIZE)

        while not exit_event.is_set():
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            aligned_frames = align.process(frames)
            
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # 获取彩色图像
            color_image = np.asanyarray(color_frame.get_data())

            # YOLO检测
            results = model(color_image)
            boxes = results[0].boxes.xyxy.cpu().tolist()

            # 绘制结果
            img = color_image.copy()
            for box in boxes:
                mid_x = int((box[0]+box[2])//2)
                mid_y = int((box[1]+box[3])//2)
                
                # 获取深度值（单位：米）
                depth = depth_frame.get_distance(mid_x, mid_y)
                if depth < 0.1 or depth > 4.0:  # 限制有效范围
                    continue
                
                # 坐标转换
                point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [mid_x, mid_y], depth)
                x, y, z = [coord*1000 for coord in point_3d]

                # 绘制检测框
                cv2.rectangle(img, (int(box[0]), int(box[1])), 
                            (int(box[2]), int(box[3])), (0,255,0), 2)
                
                # 文本位置调整
                text_y = int(box[1])-10 if int(box[1]) > 40 else int(box[3])+20
                for i, text in enumerate([f"X:{x:.1f}mm", f"Y:{y:.1f}mm", f"Z:{z:.1f}mm"]):
                    cv2.putText(img, text, (mid_x-80, text_y+i*20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 1)

            cv2.imshow('RealSense Detection', img)
            if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
                exit_event.set()

    finally:
        pipeline.stop()
        cv2.destroyWindow('RealSense Detection')

if __name__ == "__main__":
    # 启动线程
    threads = [
        threading.Thread(target=kinect_thread),
        threading.Thread(target=realsense_thread)
    ]
    
    for t in threads:
        t.start()

    # 等待线程结束
    for t in threads:
        t.join()

    cv2.destroyAllWindows()