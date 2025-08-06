import cv2

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
else:
    ret, frame = cap.read()
    if ret:
        cv2.imwrite("capture.jpg", frame)
        print("Captured image saved")
    else:
        print("Failed to grab frame")
cap.release()
