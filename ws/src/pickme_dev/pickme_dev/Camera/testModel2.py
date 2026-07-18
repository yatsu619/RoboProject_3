import cv2
import numpy as np
import os
import joblib
from sklearn.metrics import classification_report

"""
Testet das trainierte Klassifikationsmodell auf einem separaten
Testdatensatz.
 
Berechnet für jedes Testbild dieselben Merkmale wie beim Training,
lässt das gespeicherte Modell eine Vorhersage treffen und vergleicht
diese mit der tatsächlichen Klasse. Gibt am Ende die Gesamtgenauigkeit
sowie eine klassenweise Auswertung aus.
"""

IMAGE_DIR = "/home/yatheesh/Documents/rohbotik_project/PickMe/RoboProject_3/ws/src/pickme_dev/pickme_dev/Camera/test_images"
MODEL_PATH = "/home/yatheesh/Documents/rohbotik_project/PickMe/RoboProject_3/ws/src/pickme_dev/pickme_dev/Camera/model3.pkl"

CLASSES = {
    "Katze":   2,
    "Einhorn": 1,
    "Kreis":   0,
    "Quadrat": 0
}

TRAPEZ = np.array([[280, 285], [1485, 265], [1472, 620], [288, 690]])

def get_features(image_path):
    """
    Berechnet die Merkmale (Hu-Momente hu_0 und hu_3) für ein einzelnes
    Testbild, auf dieselbe Weise wie beim Training.
 
    Gibt None zurück, wenn kein gültiges Objekt gefunden wurde.
    """
    img = cv2.imread(image_path)
    if img is None:
        return None
    
    maske = np.zeros(img.shape[:2],dtype = np.uint8)
    cv2.fillPoly(maske, [TRAPEZ], 255)
    img = cv2.bitwise_and(img, img, mask = maske)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    contours, _, = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None 
    valid = [c for c in contours if cv2.contourArea(c) > 1000]
    if not valid:
        return None
    contours = max(valid, key = cv2.contourArea)

    moments = cv2.moments(contours)
    if moments["m00"] == 0:
        return None

    hu_raw = cv2.HuMoments(moments).flatten()
    hu_log = -np.sign(hu_raw) * np.log10(np.abs(hu_raw) + 1e-10)

    return [hu_log[0], hu_log[3]]

model = joblib.load(MODEL_PATH)

X = []
y = []

for folder_name, label in CLASSES.items():
    folder_path = os.path.join(IMAGE_DIR, folder_name)
    
    for filename in os.listdir(folder_path):
        if not filename.endswith(".jpg") and not filename.endswith(".png"):
            continue
        
        features = get_features(os.path.join(folder_path, filename))
        
        if features is None:
            print(f"WARNUNG: {folder_name}/{filename}")
            continue
        
        prediction = model.predict([features])[0]
        X.append(features)
        y.append(label)
        
        status = "✓" if prediction == label else "✗"
        print(f"{status} {folder_name}/{filename} → erwartet={label}, vorhergesagt={prediction}")

y = np.array(y)
y_pred = np.array([model.predict([f])[0] for f in X])

print(f"\nGesamt: {len(X)} Bilder getestet")
print(f"Genauigkeit: {(y == y_pred).mean() * 100:.1f}%")
print(classification_report(y, y_pred, target_names=["reject", "unicorn", "cat"]))