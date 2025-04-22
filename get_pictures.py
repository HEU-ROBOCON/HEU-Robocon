import cv2

def get_pictures(camera_id, save_path):
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        print("Camera is not opened")
        exit()
    count = 37
    while cv2.waitKey(33) != ord('q'):
        ret ,frame = cap.read()
        if ret:
            cv2.imshow("frame",frame)
            press = cv2.waitKey(0)
            if press == ord('s'):
                cv2.imwrite(save_path + "circle" + str(count) + ".jpg", frame)
                print("circle" + str(count) + ".jpg" + "  saved")
                count += 1
            else:
                continue

if __name__ == "__main__":
    camera_id = 6
    save_path = "/home/lh/桌面/2025RC/get_pictures/pictures/"
    get_pictures(camera_id, save_path)

            

