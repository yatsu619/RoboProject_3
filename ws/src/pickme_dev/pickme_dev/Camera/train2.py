import cv2
import numpy as np
import os
import joblib
from sklearn.tree import DecisionTreeClassifier

"""
Trainiert den Decision-Tree-Klassifikator anhand eigener Testbilder.
 
Liest für jede Klasse (Katze, Einhorn, Kreis, Quadrat) alle Bilder aus dem
jeweiligen Unterordner ein, berechnet daraus die Merkmale (Hu-Momente
hu_0 und hu_3) und trainiert damit einen Decision Tree Classifier.
Kreis und Quadrat werden dabei beide als Ausschuss (Label 0) behandelt.
Das fertige Modell wird anschließend als Datei gespeichert.
"""

IMAGE_DIR = "/home/yatheesh/Documents/rohbotik_project/PickMe/RoboProject_3/ws/src/pickme_dev/pickme_dev/Camera/test_images4"
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
    Trainingsbild.
 
    Maskiert das Bild auf den Trapez-Bereich, wandelt es in ein
    Binärbild um, sucht die größte äußere Kontur und berechnet daraus
    die Hu-Momente. Gibt None zurück, wenn kein gültiges Objekt
    gefunden wurde.
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

    # log-Transformation, damit die Hu-Momente-Werte besser vergleichbar sind
    hu_raw = cv2.HuMoments(moments).flatten()
    hu_log = -np.sign(hu_raw) * np.log10(np.abs(hu_raw) + 1e-10)
    return [hu_log[0], hu_log[3]]

# Merkmale und Labels aus allen Trainingsbildern sammeln
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
        
        X.append(features)
        y.append(label)
        print(f"{folder_name}/{filename} → hu_0={features[0]:.4f}, hu_3={features[1]:.4f}, label={label}")

print(f"\nGesamt: {len(X)} Bilder geladen")

X = np.array(X)
y = np.array(y)

# Decision Tree trainieren und Trainingsgenauigkeit ausgeben
model = DecisionTreeClassifier(max_depth=5)
model.fit(X, y)

print(f"Trainingsgenauigkeit: {model.score(X, y) * 100:.1f}%")

joblib.dump(model, MODEL_PATH)
print(f"Modell gespeichert: {MODEL_PATH}")