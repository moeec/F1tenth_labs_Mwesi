import cv2

def show_camera():
    cap = cv2.VideoCapture(4, cv2.CAP_V4L2)

    if not cap.isOpened():
        print("Cannot open camera")
        return

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to grab frame")
            break

        cv2.imshow("D435", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

show_camera()
