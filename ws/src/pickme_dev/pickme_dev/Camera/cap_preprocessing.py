import cv2
import numpy as np
import os


OBEN_LINKS   = (100, 100)
OBEN_RECHTS  = (1534, 100)
UNTEN_LINKS  = (128, 726)
UNTEN_RECHTS = (1534, 686)

ROI_X_MIN = OBEN_LINKS[0]
ROI_X_MAX = OBEN_RECHTS[0]
ROI_Y_MIN = OBEN_LINKS[1]
ROI_Y_MAX = UNTEN_LINKS[1]

'''
def detect_aruco_markers(gray_img):                             
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, rejected = detector.detectMarkers(gray_img)
    return corners, ids                                         # 3--2
                                                                # 1--0
corners, id = detect_aruco_markers(binary)
if id == 3:  # obere linker aruco marker
    punkt_3 = corners[2]
    x_3, y_3 = punkt_3
if id == 2:  # obere rechter aruco marker
    punkt_2 = corners[3]
    x_2, y_2 = punkt_2
if id == 1:  # unterer linker aruco marker
    punkt_1 = corners[1]
    x_1, y_1 = punkt_1
if id == 0:  # unterer rechter aruco marker
    punkt_0 = corners[0]
    x_0, y_0 = punkt_0
'''

ordner = 'ws/src/pickme_dev/pickme_dev/Camera/test_images/Productive/Einhorn'
bilder = sorted(os.listdir(ordner))
index = 0

'''Für die Kamera
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
'''

while True:
    frame = cv2.imread(os.path.join(ordner, bilder[index]))
    '''
    ret, frame = cap.read()
    if not ret:
    continue
    '''
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Graustufen
    grey = cv2.medianBlur(grey, 5) #Medianfilter 
    binary = cv2.threshold(grey, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1] #Otsu Schwellwert 
    #vorher Closing evtl. größer
    #Opening: kleines Rauschen entfernen und entfernt Zusammenhangskomponenten
    kernel = np.ones((7, 7), np.uint8) #7x7 Quadrat
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    roi_binary = binary[ROI_Y_MIN:ROI_Y_MAX, ROI_X_MIN:ROI_X_MAX] # ROI ausschneiden
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(roi_binary) #Connected Components nur auf dem ROI

    #richtiges Objekt filtern
    biggest_label = -1
    biggest_area = 0
    for i in range(1, num_labels):
        area  = stats[i, cv2.CC_STAT_AREA]
        w_comp = stats[i, cv2.CC_STAT_WIDTH]
        h_comp = stats[i, cv2.CC_STAT_HEIGHT]
        if area > 2000 and area < 80000 and w_comp < 400 and h_comp > 150 and area > biggest_area:
            biggest_area = area
            biggest_label = i

    if biggest_label != -1:
        x = stats[biggest_label, cv2.CC_STAT_LEFT]
        y = stats[biggest_label, cv2.CC_STAT_TOP]
        w = stats[biggest_label, cv2.CC_STAT_WIDTH]
        h = stats[biggest_label, cv2.CC_STAT_HEIGHT]

        center_x = x + w // 2
        center_y = y + h // 2

        print(f'Bild: {bilder[index]} | Bounding Box: {x} {y} {w} {h} | Zentrum: {center_x} {center_y}')

        #Offset für Zeichnen im Originalbild draufaddieren
        cv2.rectangle(frame, (x + ROI_X_MIN, y + ROI_Y_MIN), (x + w + ROI_X_MIN, y + h + ROI_Y_MIN), (0, 255, 0), 2)
        cv2.circle(frame, (center_x + ROI_X_MIN, center_y + ROI_Y_MIN), 5, (0, 0, 255), -1)

    #ROI Bereich einzeichnen
    cv2.rectangle(frame, (ROI_X_MIN, ROI_Y_MIN), (ROI_X_MAX, ROI_Y_MAX), (255, 0, 0), 2)
    cv2.putText(frame, bilder[index], (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
    cv2.imshow('Bild', frame)
    cv2.imshow('ROI Binary', roi_binary)
    taste = cv2.waitKey(0)

    if taste == 83:
        index = min(index + 1, len(bilder) - 1)
    elif taste == 81:
        index = max(index - 1, 0)
    elif taste == ord('q'):
        break

    '''
    if cv2.waitKey(1) == ord('q'):
    break
    cap.release()
    '''

cv2.destroyAllWindows()