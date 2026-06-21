import cv2
import numpy as np

punkte = np.array([[280, 285], [1485, 265], [1472, 620], [288, 690]])

cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.polylines(frame, [punkte], isClosed=True, color=(0, 255, 0), thickness=2)
    cv2.imshow("ROI Check", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()