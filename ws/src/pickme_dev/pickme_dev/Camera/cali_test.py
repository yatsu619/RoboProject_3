import cv2
import numpy as np

# Parameter laden
data = np.load('camera_params2.npz')
camera_matrix = data['camera_matrix']
dist_coeffs = data['dist_coeffs']

print('Camera Matrix:')
print(camera_matrix)
print()
print('Distortion Coefficients:')
print(dist_coeffs)

# Live Bild entzerren zum Testen
cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()
    
    # Bild entzerren mit den Kameraparametern
    undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
    
    cv2.imshow('Original', frame)
    cv2.imshow('Entzerrt', undistorted)
    
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()