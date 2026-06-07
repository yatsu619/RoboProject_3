import cv2
import numpy as np
import joblib
import os

class MLDetection:

    def __init__(self):
        model_path = os.path.join(os.path.dirname(__file__), "model.pkl")
        self._model = joblib.load(model_path)

    def classify(self, img):
        roi = img[280:710, :]
        width = roi.shape[1]
        roi[:, :int(width * 0.40)] = 0
        roi[:, int(width * 0.85):] = 0

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid = [c for c in contours if cv2.contourArea(c) > 1000]
        if not valid:
            return None

        contour = max(valid, key=cv2.contourArea)
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None

        hu_raw = cv2.HuMoments(moments).flatten()
        hu_log = -np.sign(hu_raw) * np.log10(np.abs(hu_raw) + 1e-10)

        return int(self._model.predict([hu_log.tolist()])[0])