import cv2
import numpy as np
import time
from pickme_dev.Camera.PW_coord_transform import calibrate, pixel_to_world

"""
Bildvorverarbeitung und Objekterkennung für die Positionsbestimmung.
 
Enthält die Kalibrierung über ArUco-Marker sowie die Verarbeitung jedes
einzelnen Kamerabildes: Vorverarbeitung, Objekterkennung innerhalb der
Trapez-Maske und Umrechnung der gefundenen Objektpositionen in
Weltkoordinaten.
"""

TRAPEZ = np.array([[280, 285], [1485, 265], [1472, 620], [288, 690]])

def detect_aruco_markers(frame):
    """
    Sucht im übergebenen Bild nach ArUco-Markern.
 
    Gibt die erkannten Eckpunkte sowie die zugehörigen Marker-IDs zurück.
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _ = detector.detectMarkers(frame)
    return corners, ids

def setup(cap):
    """
    Führt die einmalige Kalibrierung beim Programmstart durch.
 
    Wartet, bis alle vier benötigten ArUco-Marker (67, 69, 187, 420) im
    Kamerabild sichtbar sind, und berechnet daraus die Homographie-Matrix
    zur Umrechnung von Pixel- in Weltkoordinaten.
    """
    while True:
        first_frame = cap.read()
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
    """
    Verarbeitet ein einzelnes Kamerabild und bestimmt die Weltkoordinaten
    aller darin sichtbaren, vollständig im Trapez-Bereich liegenden Objekte.
 
    Ablauf: Graustufen, Weichzeichnen, Otsu-Schwellwert, morphologisches
    Opening, Maskierung auf den Trapez-Bereich, Objekterkennung über
    zusammenhängende Flächen, Filterung nach Größe und Sichtbarkeit,
    Schwerpunktberechnung je Objekt und Umrechnung in Weltkoordinaten
    über die Homographie-Matrix H.
 
    Gibt eine nach x-Position sortierte Liste von (Welt-x, Welt-y,
    Zeitstempel) zurück.
    """
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    grey = cv2.medianBlur(grey, 5)
    binary = cv2.threshold(grey, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    kernel = np.ones((7, 7), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    maske = np.zeros(binary.shape, dtype=np.uint8)
    cv2.fillPoly(maske, [TRAPEZ], 255)
    roi_binary = cv2.bitwise_and(binary, binary, mask=maske)
    num_labels, stats = cv2.connectedComponentsWithStats(roi_binary)

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
            
            M = cv2.moments(roi_binary[y:y+h, x:x+w])
            if M["m00"] == 0:
                continue
            center_x = x + int(M["m10"] / M["m00"])
            center_y = y + int(M["m01"] / M["m00"])
            
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