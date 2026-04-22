import cv2
import numpy as np


# Schachbrettgröße: Anzahl innerer Ecken (nicht Felder!)
# Bei einem 9x6 Schachbrett: 8 Ecken breit, 5 Ecken hoch
SCHACHBRETT = (8, 5)

# Echte Größe eines Feldes in Metern (messen nach dem Ausdrucken!)
FELDGROESSE = 0.025  # 25mm = 0.025m


# Vorbereitung: 3D-Punkte des Schachbretts (immer gleich, da flach)
objp = np.zeros((SCHACHBRETT[0] * SCHACHBRETT[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:SCHACHBRETT[0], 0:SCHACHBRETT[1]].T.reshape(-1, 2)
objp *= FELDGROESSE

# Listen für gefundene Punkte
objpoints = []  # 3D Punkte in der echten Welt
imgpoints = []  # 2D Punkte im Bild

cap = cv2.VideoCapture(0)
print('Kamera geöffnet.')
print('Halte das Schachbrettmuster vor die Kamera.')
print('Drücke "s" um ein Bild zu speichern (mind. 15-20 Bilder).')
print('Drücke "q" um die Kalibrierung zu starten.')

anzahl = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Schachbrettecken suchen
    gefunden, ecken = cv2.findChessboardCorners(gray, SCHACHBRETT, None)

    # Wenn Schachbrett gefunden → grün einzeichnen
    anzeige = frame.copy()
    if gefunden:
        cv2.drawChessboardCorners(anzeige, SCHACHBRETT, ecken, gefunden)
        cv2.putText(anzeige, 'Schachbrett erkannt!', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        cv2.putText(anzeige, 'Kein Schachbrett gefunden', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.putText(anzeige, f'Gespeicherte Bilder: {anzahl}', (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

    cv2.imshow('Kalibrierung', anzeige)

    taste = cv2.waitKey(1)

    # "s" drücken → Bild speichern
    if taste == ord('s') and gefunden:
        objpoints.append(objp)
        imgpoints.append(ecken)
        anzahl += 1
        print(f'Bild {anzahl} gespeichert.')

    elif taste == ord('s') and not gefunden:
        print('Schachbrett nicht erkannt - bitte neu positionieren.')

    # "q" drücken → Kalibrierung starten
    elif taste == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Kalibrierung berechnen
if anzahl < 5:
    print(f'Zu wenige Bilder ({anzahl}). Mindestens 5 werden benötigt.')
else:
    print(f'Kalibrierung wird berechnet mit {anzahl} Bildern...')

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    # Parameter speichern
    np.savez('camera_params.npz',
             camera_matrix=camera_matrix,
             dist_coeffs=dist_coeffs)

    print('Kalibrierung abgeschlossen!')
    print('Parameter gespeichert in: camera_params.npz')
    print(f'Fehler (sollte < 1.0 sein): {ret:.4f}')