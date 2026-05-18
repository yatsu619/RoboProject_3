import cv2

cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

for _ in range(10):
    cap.read()

ret, frame = cap.read()
cv2.imwrite('/home/yatheesh/debug3.jpg', frame)
cap.release()
print('gespeichert')