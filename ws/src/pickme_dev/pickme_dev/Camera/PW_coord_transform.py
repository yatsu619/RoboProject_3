import cv2
import numpy as np

# Ursprung (0,0) = Mitte zwischen Marker 67 und 69 (rechte Seite)
# X positiv nach links, Y positiv nach oben
MARKER_WORLD_COORDS = {
    67:  (0.000,  0.0615),
    69:  (0.000, -0.0615),
    187: (0.156,  0.0615),
    420: (0.156, -0.0615),
}

def calibrate(corners, ids):
    pixel_pts = []
    world_pts = []
    for i, marker_id in enumerate(ids.flatten()):
        if marker_id in MARKER_WORLD_COORDS:
            pixel_pts.append([corners[i][0][:, 0].mean(), corners[i][0][:, 1].mean()])
            world_pts.append(MARKER_WORLD_COORDS[marker_id])
    H, _ = cv2.findHomography(np.array(pixel_pts, dtype=np.float32), np.array(world_pts, dtype=np.float32))
    return H

def pixel_to_world(px, py, H):
    result = cv2.perspectiveTransform(np.array([[[px, py]]], dtype=np.float32), H)
    return float(result[0][0][0]), float(result[0][0][1])