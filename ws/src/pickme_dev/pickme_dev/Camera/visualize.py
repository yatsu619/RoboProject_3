import cv2
import numpy as np
import os
import joblib
import matplotlib.pyplot as plt
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
from sklearn.tree import DecisionTreeClassifier
from itertools import combinations

IMAGE_DIR = "/home/yatheesh/Documents/rohbotik_project/PickMe/RoboProject_3/ws/src/pickme_dev/pickme_dev/Camera/test_images4"
IMAGE_DIR_TEST = "/home/yatheesh/Documents/rohbotik_project/PickMe/RoboProject_3/ws/src/pickme_dev/pickme_dev/Camera/test_images5"

CLASSES = {
    "Katze":   2,
    "Einhorn": 1,
    "Kreis":   0,
    "Quadrat": 0
}

TRAPEZ = np.array([[280, 285], [1485, 265], [1472, 620], [288, 690]])

def get_all_features(image_path):
    img = cv2.imread(image_path)
    if img is None:
        return None
    maske = np.zeros(img.shape[:2], dtype=np.uint8)
    cv2.fillPoly(maske, [TRAPEZ], 255)
    img = cv2.bitwise_and(img, img, mask=maske)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    valid = [c for c in contours if cv2.contourArea(c) > 1000]
    if not valid:
        return None
    contour = max(valid, key=cv2.contourArea)
    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None
    hu_raw = cv2.HuMoments(moments).flatten()
    hu_log = -np.sign(hu_raw) * np.log10(np.abs(hu_raw) + 1e-10)
    return hu_log.tolist()

def load_data(image_dir):
    X, y = [], []
    for folder_name, label in CLASSES.items():
        folder_path = os.path.join(image_dir, folder_name)
        if not os.path.exists(folder_path):
            continue
        for filename in os.listdir(folder_path):
            if not filename.endswith(".jpg") and not filename.endswith(".png"):
                continue
            features = get_all_features(os.path.join(folder_path, filename))
            if features is None:
                continue
            X.append(features)
            y.append(label)
    return np.array(X), np.array(y)

print("Lade Daten...")
X_train, y_train = load_data(IMAGE_DIR)
X_test, y_test = load_data(IMAGE_DIR_TEST)

einzeln = [(i,) for i in range(7)]
paare = list(combinations(range(7), 2))
alle_kombis = einzeln + paare

ergebnisse = []

for kombi in alle_kombis:
    idx = list(kombi)
    X_tr = X_train[:, idx]
    X_te = X_test[:, idx]
    model = DecisionTreeClassifier(max_depth=5, random_state=42)
    model.fit(X_tr, y_train)
    genauigkeit = model.score(X_te, y_test) * 100
    y_pred = model.predict(X_te)
    cm = confusion_matrix(y_test, y_pred)
    name = "hu_" + "_hu_".join(str(i) for i in kombi)
    ergebnisse.append((name, genauigkeit, cm))

#ergebnisse.sort(key=lambda x: x[1], reverse=True)

print("\nAlle Ergebnisse sortiert:")
for name, gen, _ in ergebnisse:
    print(f"  {name}: {gen:.1f}%")

n = len(ergebnisse)
cols = 4
rows = (n + cols - 1) // cols

fig, axes = plt.subplots(rows, cols, figsize=(cols * 5, rows * 4))

fig, axes = plt.subplots(rows, cols, figsize=(cols * 4, rows * 4))
axes = axes.flatten()

for i, (name, gen, cm) in enumerate(ergebnisse):
    disp = ConfusionMatrixDisplay(cm, display_labels=["reject", "unicorn", "cat"])
    disp.plot(cmap="Blues", ax=axes[i])
    axes[i].set_title(f"{name}\n{gen:.1f}%", fontsize=8)

for j in range(i + 1, len(axes)):
    axes[j].set_visible(False)


plt.tight_layout()
plt.savefig("/home/yatheesh/Documents/rohbotik_project/PickMe/RoboProject_3/ws/src/pickme_dev/pickme_dev/Camera/alle_hu_vergleich.png", dpi=100)
plt.show()

