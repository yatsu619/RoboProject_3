import cv2
import numpy as np
import time
from pickme_dev.Camera.PW_coord_transform import calibrate, pixel_to_world
#from PW_coord_transform import calibrate, pixel_to_world
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
    for i, marker_id in enumerate(ids.flatten()):
        if marker_id == 67:
            punkt_67 = (int(corners[i][0][:, 0].mean()), int(corners[i][0][:, 1].mean()))
        if marker_id == 69:
            punkt_69 = (int(corners[i][0][:, 0].mean()), int(corners[i][0][:, 1].mean()))
        if marker_id == 187:
            punkt_187 = (int(corners[i][0][:, 0].mean()), int(corners[i][0][:, 1].mean()))
        if marker_id == 420:
            punkt_420 = (int(corners[i][0][:, 0].mean()), int(corners[i][0][:, 1].mean()))
    print(f'Marker 67: {punkt_67}')
    print(f'Marker 69: {punkt_69}')
    print(f'Marker 187: {punkt_187}')
    print(f'Marker 420: {punkt_420}')
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

    object_coord = []
    for i in range(1, num_labels):
        area  = stats[i, cv2.CC_STAT_AREA]
        w_comp = stats[i, cv2.CC_STAT_WIDTH]
        h_comp = stats[i, cv2.CC_STAT_HEIGHT]
        if area > 2000 and area < 80000 and w_comp < 400 and h_comp > 50 and h_comp < 500:
            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            if x < 288 or x + w > 1470:
                continue
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            world_x, world_y = pixel_to_world(center_x, center_y, H)
            object_coord.append((world_x, world_y, time.time()))

    object_coord.sort(key=lambda o: o[0], reverse=True)
    return object_coord

if __name__ == '__main__':
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    for _ in range(10):
        cap.read()
    H = setup(cap)
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        object_coord = process_frame(frame, H)
        for world_x, world_y, timestamp in object_coord:
            print(f'Weltkoordinaten: ({world_x:.4f}m, {world_y:.4f}m) | Timestamp: {timestamp:.3f}')
        cv2.polylines(frame, [TRAPEZ], isClosed=True, color=(255, 0, 0), thickness=2)
        cv2.imshow('Bild', frame)
        if cv2.waitKey(1) == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()