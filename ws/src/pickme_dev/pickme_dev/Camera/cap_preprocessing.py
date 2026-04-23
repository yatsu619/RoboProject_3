import cv2 

frame = cv2.imread('ws/src/pickme_dev/pickme_dev/Camera/Images/Test/Cat/bild_49.jpg')

grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

binary = cv2.threshold(grey, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary)#!! SEGMENTIERUNG und andere Methoden aus der Vorlesung, evtl. gauss, latation

biggest_label = -1
biggest_area = 0
for i in range(1, num_labels):
    y_pos = stats[i, cv2.CC_STAT_TOP]
    area = stats[i, cv2.CC_STAT_AREA]
    if y_pos >= 180 and y_pos <= 334 and area > biggest_area:   #die 180 und 334 als konstante variablen oben angeben und mit den consts dann arbeiten
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