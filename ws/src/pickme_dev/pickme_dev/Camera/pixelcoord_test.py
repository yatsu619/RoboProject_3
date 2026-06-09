import cv2

coords = (0, 0)

def show_coords(event, x, y, flags, param):
    global coords
    coords = (x, y)

cap = cv2.VideoCapture(3)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cv2.namedWindow("Live")
cv2.setMouseCallback("Live", show_coords)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.putText(frame, f'x={coords[0]}, y={coords[1]}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Live", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()