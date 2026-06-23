import cv2
import numpy as np

TRAPEZ = np.array([[280, 285], [1485, 265], [1472, 620], [288, 690]])
coords = (0, 0)

def maus(event, x, y, flags, param):
    global coords
    coords = (x, y)

cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

cv2.namedWindow("Bild")
cv2.setMouseCallback("Bild", maus)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.polylines(frame, [TRAPEZ], isClosed=True, color=(255, 0, 0), thickness=2)
    cv2.putText(frame, f'x={coords[0]}, y={coords[1]}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Bild", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()