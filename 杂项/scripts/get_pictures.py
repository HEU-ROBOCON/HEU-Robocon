import cv2  # 导入OpenCV库，用于处理图像和视频

def get_pictures(camera_id, save_path):
    """
    从指定的摄像头捕获图像并保存到指定路径。

    参数:
    camera_id (int): 摄像头的ID。
    save_path (str): 保存图片的路径。
    """
    # 打开摄像头
    cap = cv2.VideoCapture(camera_id)
    
    # 设置摄像头的分辨率和帧率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # 设置宽度为1280像素
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # 设置高度为720像素
    cap.set(cv2.CAP_PROP_FPS, 30)  # 设置帧率为30帧每秒

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("Camera is not opened")  # 如果摄像头未打开，打印错误信息
        exit()  # 退出程序

    count = 0  # 初始化图片计数器，从0开始

    # 循环捕获图像，直到按下'q'键退出
    while True:
        ret, frame = cap.read()  # 读取一帧图像
        if ret:  # 如果成功读取到图像
            cv2.imshow("frame", frame)  # 显示当前帧
            press = cv2.waitKey(0)  # 等待键盘输入
            if press == ord('s'):  # 如果按下's'键
                # 保存当前帧为图片文件
                cv2.imwrite(save_path + "loop" + str(count) + ".jpg", frame)
                print("loop" + str(count) + ".jpg" + "  saved")  # 打印保存成功信息
                count += 1  # 图片计数器加1
            elif press == ord('q'):  # 如果按下'q'键
                break  # 退出循环
            else:
                continue  # 如果按下其他键，继续循环

if __name__ == "__main__":
    # 设置摄像头ID和图片保存路径
    camera_id = 4  # 摄像头ID
    save_path = "/home/water/桌面/Robocon/Robocon2025/imgs/"  # 图片保存路径

    # 调用函数开始捕获图片
    get_pictures(camera_id, save_path)