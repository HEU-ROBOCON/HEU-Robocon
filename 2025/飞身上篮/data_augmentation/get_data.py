import cv2
import os
import time

def capture_circles(output_dir="识别圆"):
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    # 初始化摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return
    
    print("摄像头已启动，按以下键操作:")
    print("空格键 - 拍摄照片")
    print("s键 - 保存当前照片")
    print("ESC键 - 退出程序")
    
    captured_img = None
    counter = 1
    
    while True:
        # 读取摄像头画面
        ret, frame = cap.read()
        if not ret:
            print("无法获取画面")
            break
        
        # 显示操作提示
        cv2.putText(frame, "Press SPACE to Capture", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "Press 's' to Save", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "Press ESC to Exit", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 显示当前拍摄状态
        if captured_img is not None:
            cv2.putText(frame, f"Captured! Press 's' to save as {counter}.jpg", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # 显示画面
        cv2.imshow("Basketball Circle Marker Capture", frame)
        
        # 按键检测
        key = cv2.waitKey(1) & 0xFF
        
        if key == 27:  # ESC键退出
            break
        elif key == 32:  # 空格键拍摄
            captured_img = frame.copy()
            print(f"已捕获图像 {counter}, 按's'保存")
        elif key == ord('s') and captured_img is not None:  # s键保存
            filename = os.path.join(output_dir, f"circle_{time.strftime('%Y%m%d_%H%M%S')}_{counter}.jpg")
            cv2.imwrite(filename, captured_img)
            print(f"已保存: {filename}")
            counter += 1
            captured_img = None
    
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("程序结束")

if __name__ == "__main__":
    capture_circles()
