import cv2
import numpy as np

# Ursprung (0,0) = Mitte zwischen Marker 67 und 69 (rechte Seite)
# X positiv nach links, Y positiv nach oben
"""
Umrechnung von Pixel- in Weltkoordinaten über eine Homographie.
 
Nutzt vier fest im Arbeitsbereich montierte ArUco-Marker mit bekannten
realen Positionen, um einmalig eine Umrechnungsmatrix (Homographie) zu
berechnen. Diese Matrix wird danach für jeden beliebigen Pixelpunkt
wiederverwendet.
"""

MARKER_WORLD_COORDS = {
    67:  (0.000,  0.0625),
    69:  (0.000, -0.0625), 
    187: (0.198,  0.0620),
    420: (0.198, -0.0620),
}

def calibrate(corners, ids):
    """
    Berechnet die Homographie-Matrix aus den erkannten Marker-Eckpunkten.
 
    Ordnet jedem erkannten Marker seine bekannte reale Weltposition zu
    und berechnet daraus mit cv2.findHomography die Umrechnungsmatrix H.
    Gibt zur Kontrolle die berechneten Weltkoordinaten der Marker aus
    und vergleicht sie mit den erwarteten Werten.
    """
    pixel_pts = []
    world_pts = []
    for i, marker_id in enumerate(ids.flatten()):
        if marker_id in MARKER_WORLD_COORDS:
            pixel_pts.append([corners[i][0][:, 0].mean(), corners[i][0][:, 1].mean()])
            world_pts.append(MARKER_WORLD_COORDS[marker_id])
    H, _ = cv2.findHomography(np.array(pixel_pts, dtype=np.float32), np.array(world_pts, dtype=np.float32))
    print(f'Pixel-Punkte: {pixel_pts}')
    print(f'Welt-Punkte: {world_pts}')
    print(f'H Matrix: {H}')
    for i in range(len(pixel_pts)):
        wx, wy = pixel_to_world(pixel_pts[i][0], pixel_pts[i][1], H)
        print(f'Pixel {pixel_pts[i]} → Welt berechnet: ({wx:.4f}, {wy:.4f}) | Welt erwartet: {world_pts[i]}')
    return H

def pixel_to_world(px, py, H):
    """
    Rechnet einen einzelnen Pixelpunkt (px, py) über die Homographie-Matrix H
    in eine Weltkoordinate (x, y) in Metern um.
    """
    result = cv2.perspectiveTransform(np.array([[[px, py]]], dtype=np.float32), H)
    return float(result[0][0][0]), float(result[0][0][1])