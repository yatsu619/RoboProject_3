import cv2
import numpy as np
import joblib
import os

TRAPEZ = np.array([[280, 285], [1485, 265], [1472, 620], [288, 690]])

class MLDetection:
    def __init__(self):
        model_path = os.path.join(os.path.dirname(__file__), "model3.pkl")
        self._model = joblib.load(model_path)

    def classify(self, img):
        maske = np.zeros(img.shape[:2], dtype=np.uint8)
        cv2.fillPoly(maske, [TRAPEZ], 255)
        img_masked = cv2.bitwise_and(img, img, mask=maske)

        gray = cv2.cvtColor(img_masked, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        kernel = np.ones((7, 7), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid = []
        for c in contours:
            area = cv2.contourArea(c)
            bx, by, bw, bh = cv2.boundingRect(c)
            if area > 2000 and area < 80000 and bw < 400 and bh > 50 and bh < 500:
                if bx >= 288 and bx + bw <= 1470:
                    valid.append(c)
        contours = valid
        contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0], reverse=True)
        labels = []
        for c in contours:
            moments = cv2.moments(c)
            if moments["m00"] == 0:
                continue
            hu_raw = cv2.HuMoments(moments).flatten()
            hu_log = -np.sign(hu_raw) * np.log10(np.abs(hu_raw) + 1e-10)
            label = int(self._model.predict([[hu_log[0], hu_log[3]]])[0])
            labels.append(label)
        return labels