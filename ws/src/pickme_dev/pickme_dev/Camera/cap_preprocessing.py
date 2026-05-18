import cv2
import numpy as np

# Zeitstempel muss dann auch im topic verschickt werden
# Randpixel weg bis nur eins übrig bleibt, erosion

cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

for _ in range(10):
    cap.read()

def detect_aruco_markers(frame):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    frame = cv2.convertScaleAbs(frame, alpha=2.0, beta=50)  # aufhellen
    corners, ids, _ = detector.detectMarkers(frame)
    return corners, ids                                     # 67--69
                                                            # 187--420

# --- ROI aus ArUco Markern berechnen (einmalig) ---
#ret, first_frame = cap.read()
#corners, ids = detect_aruco_markers(first_frame)

while True:
    ret, first_frame = cap.read()
    corners, ids = detect_aruco_markers(first_frame)
    if ids is not None and all(m in ids.flatten() for m in [67, 69, 187, 420]):
        break
    print('Warte auf alle 4 Marker...')

for i, marker_id in enumerate(ids.flatten()):
    if marker_id == 67:
        punkt_67  = (int(corners[i][0][:, 0].mean()), int(corners[i][0][:, 1].mean()))
    if marker_id == 69:
        punkt_69  = (int(corners[i][0][:, 0].mean()), int(corners[i][0][:, 1].mean()))
    if marker_id == 187:
        punkt_187 = (int(corners[i][0][:, 0].mean()), int(corners[i][0][:, 1].mean()))
    if marker_id == 420:
        punkt_420 = (int(corners[i][0][:, 0].mean()), int(corners[i][0][:, 1].mean()))

print(f'punkt_67: {punkt_67}')
print(f'punkt_69: {punkt_69}')
print(f'punkt_187: {punkt_187}')
print(f'punkt_420: {punkt_420}')

ROI_X_MIN = min(punkt_67[0],  punkt_187[0])
ROI_X_MAX = max(punkt_69[0],  punkt_420[0])
ROI_Y_MIN = min(punkt_67[1],  punkt_69[1])
ROI_Y_MAX = max(punkt_187[1], punkt_420[1])

print(f'ROI: ({ROI_X_MIN}, {ROI_Y_MIN}) bis ({ROI_X_MAX}, {ROI_Y_MAX})')

# --- Hauptloop ---
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    grey = cv2.medianBlur(grey, 5)
    binary = cv2.threshold(grey, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

    kernel = np.ones((7, 7), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    roi_binary = binary[ROI_Y_MIN:ROI_Y_MAX, ROI_X_MIN:ROI_X_MAX]

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(roi_binary)

    biggest_label = -1
    biggest_area = 0
    for i in range(1, num_labels):
        area   = stats[i, cv2.CC_STAT_AREA]
        w_comp = stats[i, cv2.CC_STAT_WIDTH]
        h_comp = stats[i, cv2.CC_STAT_HEIGHT]
        print(f'Label {i}: area={area}, w={w_comp}, h={h_comp}')
        if area > 2000 and area < 80000 and w_comp < 400 and h_comp > 150 and h_comp < 500 and area > biggest_area:
            biggest_area = area
            biggest_label = i

    if biggest_label != -1:
        x = stats[biggest_label, cv2.CC_STAT_LEFT]
        y = stats[biggest_label, cv2.CC_STAT_TOP]
        w = stats[biggest_label, cv2.CC_STAT_WIDTH]
        h = stats[biggest_label, cv2.CC_STAT_HEIGHT]
        center_x = x + w // 2
        center_y = y + h // 2
        print(f'Bounding Box: {x} {y} {w} {h} | Zentrum: {center_x} {center_y}')

        cv2.rectangle(frame, (x + ROI_X_MIN, y + ROI_Y_MIN), (x + w + ROI_X_MIN, y + h + ROI_Y_MIN), (0, 255, 0), 2)
        cv2.circle(frame, (center_x + ROI_X_MIN, center_y + ROI_Y_MIN), 5, (0, 0, 255), -1)

    cv2.rectangle(frame, (ROI_X_MIN, ROI_Y_MIN), (ROI_X_MAX, ROI_Y_MAX), (255, 0, 0), 2)
    cv2.imshow('Bild', frame)
    cv2.imshow('ROI Binary', roi_binary)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()