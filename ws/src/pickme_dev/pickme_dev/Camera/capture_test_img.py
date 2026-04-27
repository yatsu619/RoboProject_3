import cv2
import numpy as np
import os


# Ordner zum Speichern (anpassen je nach Objekt: Cat oder MagicUnicorn)
SAVE_FOLDER = 'Images/Productive/Quadrat'
CAMERA_INDEX = 2

os.makedirs(SAVE_FOLDER, exist_ok=True)

# Kameraparameter laden
data = np.load('camera_params.npz')
camera_matrix = data['camera_matrix']
dist_coeffs = data['dist_coeffs']

cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

print(f'Bilder werden gespeichert in: {SAVE_FOLDER}')
print('Drücke "s" um ein Bild zu speichern.')
print('Drücke "q" zum Beenden.')

anzahl = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Bild entzerren
    frame_entzerrt = cv2.undistort(frame, camera_matrix, dist_coeffs)

    cv2.putText(frame_entzerrt, f'Gespeichert: {anzahl}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Aufnahme', frame_entzerrt)

    taste = cv2.waitKey(1)

    if taste == ord('s'):
        dateiname = f'{SAVE_FOLDER}/bild_{anzahl}.jpg'
        cv2.imwrite(dateiname, frame_entzerrt)
        anzahl += 1
        print(f'Bild gespeichert: {dateiname}')

    elif taste == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()