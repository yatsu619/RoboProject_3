import cv2
import numpy as np
import os
import joblib
from sklearn.tree import DecisionTreeClassifier

IMAGE_DIR = "/home/yatheesh/Documents/rohbotik_project/PickMe/RoboProject_3/ws/src/pickme_dev/pickme_dev/Camera/test_images2"
MODEL_PATH = "/home/yatheesh/Documents/rohbotik_project/PickMe/RoboProject_3/ws/src/pickme_dev/pickme_dev/Camera/model.pkl"

CLASSES = {
    "Katze": 2,
    "Einhorn": 1,
    "Kreis": 0,
    "Quadrat": 0
}

def get_features(image_path):
    img = cv2.imread(image_path)
    if img is None:
        return None

    roi = img[280:710, :]
    width = roi.shape[1]
    roi[:, :int(width * 0.40)] = 0
    roi[:, int(width * 0.85):] = 0
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    valid = [c for c in contours if cv2.contourArea(c) > 3000]
    if not valid:
        return None

    contour = max(valid, key=cv2.contourArea)

    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None

    hu_raw = cv2.HuMoments(moments).flatten()
    hu_log = -np.sign(hu_raw) * np.log10(np.abs(hu_raw) + 1e-10)

    #hu_0 = hu_log[0]
    #hu_3 = hu_log[3]

    return hu_log.tolist()

X = []
y = []

for folder_name, label in CLASSES.items():
    folder_path = os.path.join(IMAGE_DIR, folder_name)

    for filename in os.listdir(folder_path):
        if not filename.endswith(".jpg") and not filename.endswith(".png"):
            continue

        image_path = os.path.join(folder_path, filename)
        features = get_features(image_path)

        if features is None:
            print(f"WARNUNG: Kein Objekt gefunden in {folder_name}/{filename}")
            continue

        X.append(features)
        y.append(label)
        print(f"{folder_name}/{filename} → label={label}")

print(f"\nGesamt: {len(X)} Bilder geladen")

X = np.array(X)
y = np.array(y)

model = DecisionTreeClassifier(max_depth=5)
model.fit(X, y)

print(f"Trainingsgenauigkeit: {model.score(X, y) * 100:.1f}%")

from sklearn.metrics import classification_report
y_pred = model.predict(X)
print(classification_report(y, y_pred, target_names=["reject", "unicorn", "cat"]))

joblib.dump(model, MODEL_PATH)
print(f"Modell gespeichert: {MODEL_PATH}")