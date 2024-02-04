import cv2

cam = cv2.VideoCapture(0)

ret, frame = cam.read()

cam.release()

cv2.imwrite("captured_img.jpg", frame)