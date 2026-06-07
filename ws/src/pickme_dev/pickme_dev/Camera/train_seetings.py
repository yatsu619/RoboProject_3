import cv2
import numpy as np
import os

BASE = "/home/yatheesh/Documents/rohbotik_project/PickMe/RoboProject_3/ws/src/pickme_dev/pickme_dev/Camera/test_images3"

for folder in ["Katze", "Einhorn", "Kreis", "Quadrat"]:
    hu_list = []
    folder_path = os.path.join(BASE, folder)
    for filename in os.listdir(folder_path):
        if not filename.endswith(".jpg"):
            continue
        img = cv2.imread(os.path.join(folder_path, filename))
        roi = img[280:710, :]
        width = roi.shape[1]
        roi[:, :int(width * 0.40)] = 0
        roi[:, int(width * 0.85):] = 0
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if cv2.contourArea(c) > 1000]
        if not valid:
            continue
        contour = max(valid, key=cv2.contourArea)
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        hu_raw = cv2.HuMoments(moments).flatten()
        hu_log = -np.sign(hu_raw) * np.log10(np.abs(hu_raw) + 1e-10)
        hu_list.append(hu_log)
    
    if hu_list:
        avg = np.mean(hu_list, axis=0)
        print(f"\n{folder}:")
        for i, v in enumerate(avg):
            print(f"  hu_{i} = {v:.4f}")