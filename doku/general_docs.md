<!-- Geschrieben und dokumentiert von Yatheesh Sugumar, Marcel Kattirs und Johannes Sedlmayr -->

# Allgemeine Dokumentation für das PickMe Robotikprojekt

1. [Projektplan](#1-projektplan)
2. [Software-Architektur](#2-software-architektur)
3. [Designentscheidungen](#3-designentscheidungen)
4. [Technische Herleitungen](#4-technische-herleitungen)
5. [Lessons Learned](#5-lessons-learned)
6. [Auswertung des Gesamtsystems](#6-auswertung-des-gesamtsystems)
7. [Dokumente und Referenzen](#7-dokumente-und-referenzen)

## Zugehörige dokumentation
Die Dokumentation der verwendeten und eigenentwickelten ROS2 knoten wurden separat von den jeweiligen entwicklern / maintainern festgehalten und können hier eingesehen werden:

### Eigenentwicklungen:

Motioncontroller: **[motion_docs.md](motion_docs.md)**

Vision Pipeline: **[vision_docs.md](vision_docs.md)**

Waypoint Prediction: **[prediction_docs.md](prediction_docs.md)**

### Externe ROS2 knoten:

ROS2 serial bridge: **[README.md](/ws/src/ro45_ros2_pickrobot_serial/README.md)**

ROS2 Portalrobot interfaces: **[README.md](/ws/src/ro45_portalrobot_interfaces/README.md)**

# 1 Projektplan

## 1.1 Projektübersicht

### 1.1.1 Ziele
- Entwicklung einer Sortieranlage für Katzen und einhörner basierend auf einem Portalroboter mit Kamerasystem
- Entwicklung der Robotersoftware mit dem ROS2 'Humble' framework mit der Programmiersprache Python

### 1.1.2 Projektteam
- Yatheesh Sugumar (Zuständig für die Visionpipeline)
- Marcel Kattirs (Zuständig für die achsenansteuerung)
- Johannes Sedlmayr (Zuständig für die WaypointPredictionEngine)

## 1.2 Meilensteine und Zeitplan
| Meilenstein | Termin | Status | Beschreibung |
|-------------|--------|--------|--------------|
| Kick-off-Präsentation | 30.03.26 | Abgeschlossen | Softwarekonzept erarbeiten und in Meilensteine aufteilen. Vorstellung der Meilensteine und des Gesamtkonzeptes. |
| Koordinatensysteme festlegen | 06.04.26 | Abgeschlossen | Weltkoordinatensystem (WKS) festgelegt sowie Roboterkoordinatensystem (RCS) und Kamerakoordinatensystem (CCS) aufbau verstanden. Koordinatentransformationen überlegen und vorbereiten. |
| Bounding Box festlegen | 27.04.26 | Abgeschlossen | Die kamera hat nun ein festgelegtes und implementiertes konzept für eine box um das erkannte objekt. |
| Ausgabe der prädizierten Positionen im WKS + Test | 11.05.26 | Abgeschlossen | Die WKS koordinaten des erkannten objektes können ausgegeben werden und auch über ROS2 topics weitergeleitet werden. |
| Regelung auf einen vorgegebenen punkt im WKS | 25.05.26 | Abgeschlossen | Der Roboter soll nun vorgegebene punkte im WKS anfahren können. |
| Objekt sortiert und in box abgelegt | 08.06.26 | Abgeschlossen | Der Roboter kann nun objekte mit dem greifer aufnehmen und in die jeweilige box sortieren. |
| Erfolgsrate von 90% eingehalten | 22.06.26 | Abgeschlossen | Testen und verbesseren bis die anforderungen der 90% korrekt sortierten objekte eingehalten wird |
| Abschlusspräsentation | 29.06.26 | Abgeschlossen | Fertiger Roboter wird vorgestellt, softwareentscheidungen begründet und dargelegt und Praktische demonstration wird durchgeführt. |

# 2 Software Architektur
![Software Architektur](</doku/software_arch/SoftwareArch.png>)
Die blauen ovale stellen die implementierten ROS2 knoten dar. Die pinken kästchen stellen die jeweiligen Topics dar. Die orangenen kästchen hierbei die jeweiligen logiken dar.

# 3 Designentscheidungen
Bei fragen zu detaillierten technischen entscheidungen spezifischer verwendeter ROS2 knoten ist [hier](#zugehörige-dokumentation) einzusehen.

## 3.1 ROS2 Messages
Da die logik in verschiedene knoten aufgeteilt wurden, mussten custom messages verwendet werden. Anstatt ein separates paket zu erstellen,wurden diese dem von Prof. Lorenzen bereitgestellten 'ro45_portalrobot_interfaces' hinzugefügt. Folgende sind die von PickMe neu erstellten:
- CamData.msg
- ExtDebug.msg
- PredictedPos.msg
- PredictedPosdelay.msg
- RobotPosStamped.msg

# 4 Technische herleitungen

## 3.1 Initiales Konzept
![Initiales Konzept](<PickMe Konzept.png>)
Dies ist das Konzept welches zu beginn entwickelt wurde und auch mit relativ wenigen veränderungen umgesetzt wurde. Die Transformation von Kamerakoordinaten in Weltkoordinaten wurde, statt einer normalen Transformation, über 4 ArUco marker und Homographie gelöst.

# 5 Lessons learned

# 6 Auswertung des Gesamtsystems

# 7 Dokumente und referenzen
