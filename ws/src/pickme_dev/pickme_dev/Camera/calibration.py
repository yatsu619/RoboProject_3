import cv2
import numpy as np


# ChArUco Board Einstellungen
FELDER_X = 7
FELDER_Y = 5
FELDGROESSE = 0.024   # 2.4cm in Metern
MARKERGROESSE = 0.018 # ca. 75% der Feldgröße

CAMERA_INDEX = 2

# ArUco Dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# ChArUco Board erstellen
board = cv2.aruco.CharucoBoard(
    (FELDER_X, FELDER_Y),
    FELDGROESSE,
    MARKERGROESSE,
    aruco_dict
)

detector = cv2.aruco.CharucoDetector(board)

objpoints = []
imgpoints = []

cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

print('Kamera geöffnet.')
print('Halte das ChArUco Board vor die Kamera.')
print('Drücke "s" um ein Bild zu speichern (mind. 15-20 Bilder).')
print('Drücke "q" um die Kalibrierung zu starten.')

anzahl = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)

    anzeige = frame.copy()

    if charuco_ids is not None and len(charuco_ids) > 4:
        cv2.aruco.drawDetectedCornersCharuco(anzeige, charuco_corners, charuco_ids)
        cv2.putText(anzeige, 'Board erkannt!', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        cv2.putText(anzeige, 'Board nicht gefunden', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.putText(anzeige, f'Gespeicherte Bilder: {anzahl}', (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

    cv2.imshow('Kalibrierung', anzeige)

    taste = cv2.waitKey(1)

    if taste == ord('s'):
        if charuco_ids is not None and len(charuco_ids) > 4:
            objpoints.append(charuco_corners)
            imgpoints.append(charuco_ids)
            anzahl += 1
            print(f'Bild {anzahl} gespeichert.')
        else:
            print('Board nicht erkannt - bitte neu positionieren.')

    elif taste == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if anzahl < 5:
    print(f'Zu wenige Bilder ({anzahl}). Mindestens 5 werden benötigt.')
else:
    print(f'Kalibrierung wird berechnet mit {anzahl} Bildern...')

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        objpoints, imgpoints, board, gray.shape[::-1], None, None
    )

    np.savez('camera_params.npz',
             camera_matrix=camera_matrix,
             dist_coeffs=dist_coeffs)

    print('Kalibrierung abgeschlossen!')
    print('Parameter gespeichert in: camera_params.npz')
    print(f'Fehler (sollte < 1.0 sein): {ret:.4f}')