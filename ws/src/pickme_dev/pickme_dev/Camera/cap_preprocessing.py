import cv2 

frame = cv2.imread('ws/src/pickme_dev/pickme_dev/Camera/Images/Test/Cat/bild_49.jpg')

grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


binary = cv2.threshold(grey, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
punkt_3=0,0 
punkt_2=0,0
punkt_1=0,0
punkt_0=0,0
'''
def detect_aruco_markers(gray_img):                             
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) # größe an unsere marker anpassen 
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, rejected = detector.detectMarkers(gray_img)
    return corners, ids                                         # 3--2
                                                                #1--0

corners,id =detect_aruco_markers(binary)
if id== 3:# ecke mit x und y obere linker  aruco marker 
    punkt_3 = corners[2]
    x_3, y_3 = punkt_3 
if id== 2:# ecke mit x und y obere rechter   aruco marker 
    punkt_2 = corners[3]
    x_2, y_2 = punkt_2 
if id== 1:# ecke mit x und y unterer linker aruco marker 
    punkt_1 = corners[1]
    x_1, y_1 = punkt_1 
if id== 0:# ecke mit x und y unterer rechter aruco marker 
    punkt_0 = corners[0]
    x_0, y_0 = punkt_0 
'''
num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary)#!! SEGMENTIERUNG und andere Methoden aus der Vorlesung, evtl. gauss, latation
# die funktion nicht hier ausführen sondern auf der begrenzten bild menge 
biggest_label = -1
biggest_area = 0
for i in range(1, num_labels):
    y_pos = stats[i, cv2.CC_STAT_TOP]
    area = stats[i, cv2.CC_STAT_AREA]
    if y_pos >= 180 and y_pos <= 334 and area > biggest_area:   #hier mit den vier punkten arbeiten wo du x und y hast von den punkten 
        biggest_area = area
        biggest_label = i
    
if biggest_label != -1:
    x = stats[biggest_label, cv2.CC_STAT_LEFT]
    y = stats[biggest_label, cv2.CC_STAT_TOP]
    w = stats[biggest_label, cv2.CC_STAT_WIDTH]
    h = stats[biggest_label, cv2.CC_STAT_HEIGHT]

    center_x = x + w // 2
    center_y = y + h // 2

    print('Bounding Box:', x, y, w, h)
    print('Geometrisches Zentrum:', center_x, center_y)

    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

cv2.imshow('Bild', frame)
#cv2.imshow('Graustufen', grey)
#cv2.imshow('Otsu', binary)
cv2.waitKey(0)
cv2.destroyAllWindows()