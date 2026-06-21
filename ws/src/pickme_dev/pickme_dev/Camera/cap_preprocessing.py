import cv2
import numpy as np
import time
from pickme_dev.Camera.PW_coord_transform import calibrate, pixel_to_world

TRAPEZ = np.array([[280, 285], [1485, 265], [1472, 620], [288, 690]])

def detect_aruco_markers(frame):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _ = detector.detectMarkers(frame)
    return corners, ids

def setup(cap):
    while True:
        ret, first_frame = cap.read()
        corners, ids = detect_aruco_markers(first_frame)
        if ids is not None and all(m in ids.flatten() for m in [67, 69, 187, 420]):
            break
        print('Warte auf alle 4 Marker...')
    H = calibrate(corners, ids)
    print('Kalibrierung erfolgreich!')
    return H

def process_frame(frame, H):
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    grey = cv2.medianBlur(grey, 5)
    binary = cv2.threshold(grey, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    kernel = np.ones((7, 7), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    maske = np.zeros(binary.shape, dtype=np.uint8)
    cv2.fillPoly(maske, [TRAPEZ], 255)
    roi_binary = cv2.bitwise_and(binary, binary, mask=maske)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(roi_binary)
    biggest_label = -1
    biggest_area = 0
    for i in range(1, num_labels):
        area   = stats[i, cv2.CC_STAT_AREA]
        w_comp = stats[i, cv2.CC_STAT_WIDTH]
        h_comp = stats[i, cv2.CC_STAT_HEIGHT]
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
        world_x, world_y = pixel_to_world(center_x, center_y, H)
        timestamp = time.time()
        return world_x, world_y, timestamp
    return None, None, None

if __name__ == '__main__':
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    for _ in range(10):
        cap.read()
    H = setup(cap)
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        world_x, world_y, timestamp = process_frame(frame, H)
        if world_x is not None:
            print(f'Weltkoordinaten: ({world_x:.4f}m, {world_y:.4f}m) | Timestamp: {timestamp:.3f}')
        cv2.polylines(frame, [TRAPEZ], isClosed=True, color=(255, 0, 0), thickness=2)
        cv2.imshow('Bild', frame)
        if cv2.waitKey(1) == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()