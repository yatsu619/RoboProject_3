<!-- Geschrieben und dokumentiert von Marcel Kattirs -->

# Motioncontroller Dokumentation
Dieses Dokument umfasst die dokumentation, insbesondere die technische implementation, des Motioncontrollers. Die codedokumentation wurde mittels docstrings in den jeweiligen Pythonfiles durchgeführt und wird als solches hier nur angeschnitten.

## Inhaltsverzeichnis

1. [Konzeptüberblick](#1-konzeptüberblick)
2. [Technische Herleitungen](#2-technische-herleitungen)
3. [ROS2-2 Knoten und Veröffentlichung](#3-ros2-knoten-und-veröffentlichung)

# 1 Konzeptüberblick
Der Motioncontroller ist essentieller Bestandteil für die ansteuerung des Portalroboters. Er ist über ROS2 topics und messages mit dem Portalroboter und der WaypointPredictionEngine verbunden. Zur erfüllung dieser funktion wurde sich für eine timerbasierte ausführung entschieden welche mit 10 Hz läuft. Um ein effizientes Positionsregeln zu gewährleisten wurde mithilfe des Programms Matlab-Simulink ein PD-Regler entwickelt. Zusätzlich ist eine Interne Statemachine implementiert welche auch für eine einfache weiterentwicklung erweiterbar ist. Zuletzt, um auch ohne zugang zu Hardware zu entwickeln, wurde ein Proprietärer Simulator entwickelt der mittels Matplotlib die positionen der achsen plottet.

# 2 Technische Herleitungen

## 2.1 Timerbasierte ausführung
In ROS2 gibt mehrere herangehensweisen wie man die ausführung seines Programmes steuern kann. Zum einen gibt es die timerbasierte ausführung, die hier gewählt wurde, und zum anderen deb action-server mechanismus.

Es wurde sich für eine Timerbasierte lösung entschieden aufgrund der einfacheren implementierung und dem einfacheren verständnis der zu implementierenden logik.

## 2.2 PD-Regler
Bei der entwicklung des Reglers wurde als randbedingung festgelegt das, zumindest aus sicht der simulation, keine überschwinger auftreten sollen, er also eine kritische dämpfung besitzt. Hierfür wurde Matlab-Simulink verwendet da matlab zum einen hierfür prädestiniert ist und Matlab mit der Simulink erweiterung ein relativ einfach zu bedienendes Grafisches interface bietet mit welchem Reglersysteme schnell entwickelt und simuliert werden können.

Mehrere Reglertypen wurden entwickelt und getestet, bis auf den PD-Regler schlussendlich verworfen. Nachfolgend eine nicht-vollständige aber repräsentative liste:

| Reglertyp | Grund für Simulation | Probleme | Status |
|-----------|----------------------|----------|--------|
| P-Regler  | Bekannter Regler, einfach zu implementieren | Instabilität, Dauerschwingen | Verworfen
| PID-Regler | Komplex zu implementieren, hätte aber aufgrund des I-Anteils eine Steilere flanke und könnte schneller Regeln | Starkes überschwingen (konnte nicht gelöst werden) | Verworfen |
| PD-Regler | D-Glied wirkt stabilisierend, erreichung kritischer dämpfung einfacher | Keine Probleme | Implementiert |

Das equivalente Blockschaltbild des Reglers sieht dann wie folgt aus:
![Simulink Reglermodell](</doku/reglermodell/simulink_regler.PNG>)
Auf der linken seite wird der regelfehler berechnet. Dieser Regelfehler wird dann in einen P-Regler (G = 1) sowie den D-Anteil (G = 2) geführt. Der Ausgang der beiden regler wird dann zusammengeführt. Die rechte seite wendet auf den reglerausgang den threshold an welcher der maximalen beschleunigung des roboters entspricht.

## 2.3 Interne State-Machine
Aufgrunddessen wie die logik des Motioncontrollers aufgebaut und konzipiert ist, lassen sich die einzelnen Programmschnitte in feste gruppierungen (states) aufteilen. Nachfolgend eine beschreibung der einzelnen states:

**Homing:**
Bei jedem neustart des ROS2 Knotens wird ein homing durchgeführt. Hier werden alle achsen des Roboters in die endanschläge gefahren. Um zu verhindern dass der homing-prozess nicht von einem erkannten objekt unterbrochen wird, wird der mutex 'new_object_lock' erst nach dem homing deaktiviert. Da auch das topic des roboters `/RobotPos` bei jedem neustart seine aktuelle position als ursprung festlegt, als solches also als pseudo-random zu betrachten ist, ist der homing schritt notwendig. Zuerst wird hier die TCP (Toolcenterpoint) position berechnet indem der offset der achsen zu den endanschlägen abgezogen wird. Im Hauptprogramm wird dann in jedem durchlauf jede achse einzeln mittels ausgemessener offsets zum TCP und WKS die aktuelle TCP position im WKS bestimmt.

**IDLE:**
Nach dem Homing folgt der state 'IDLE'. In diesem Zustand wird in einer position mittig über dem förderband in einer höhe von etwa 4cm gewartet bis ein objekt erkannt und dessen position prädiziert wird.

**PICK:**
Wird ein objekt erkannt wird in diesen zustand gewechselt. Dieser steuert die Prädizierte x und y Position an. Diese Position wird solange angesteuert bis eine z-position wenige millimeter über dem förderband erreicht wurde. Wird die z-position erreicht, wird der greifer aktiviert und der zustand 'AFTERPICK' folgt. Außerdem wird der 'new_object_lock' mutex gesetzt sodass der Pick and Place Prozess nicht unterbrochen wird.

**AFTERPICK:**
Wird ein objekt aufgenommen, folgt der 'AFTERPICK'. Hier wird auf den Weltkoordinatenursprung zurückgefahren und die z-achse wird wieder auf eine höhe von 4cm gefahren. Dies ist notwendig um eine gewisse marge zu den verschiedenen Objektfangboxen zu erreichen.

**PLACE:**
Direkt auf den 'AFTERPICK' folgt der 'PLACE'. In diesem wird zuerst entschieden um welches objekt es sich handelt. Er teilt sich somit in folgende unterzustände auf:

**CAT (PLACE):**
In diesem Zustand wurde eine Katze erkannt. Hier wird die y-achse in richtung des negativen endanschlages kommandiert. Ebenso wird die X-Achse auf die Position der weiter hinten liegenden, zweiten Fangbox, geregelt. Werden die Positionen erreicht, wird das objekt fallen gelassen. Außerdem wird das 'new_object_lock' mutex deaktiviert.

**UNICORN (PLACE):**
In diesem Zustand wurde ein Einhorn erkannt. Hier wird die y-achse in richtung des negativen endanschlages kommandiert. Ebenso wird die X-Achse auf die Position der weiter vorne liegenden, ersten Fangbox, geregelt. Werden die Positionen erreicht, wird das objekt fallen gelassen. Außerdem wird das 'new_object_lock' mutex deaktiviert.

Damit sehen alle states und wie sie aufeinander folgen so aus:
![Motioncontroller Statemachine](</doku/motioncontroller_states.PNG>)
Im Diagramm ist zwar abgebildet, dass nach dem Place prozess in den 'IDLE' gewechselt wird, allerdings ist ein sofortiges regeln auf ein neu Prädiziertes objekt möglich.

## 2.4 Simulator
Der Simulator verwendet das Konzept der Doppelintegration um den Roboter zu Simulieren. Da dem Roboter nur beschleunigungen kommandiert werden können, müssen diese zur Positionsregelung verwendet werden. Der Physikalische zusammenhang ist wie folgt:

Beschleunigung ---> Geschwindigkeit ---> Position

Diese Daten werden vom Simulator aufgezeichnet und die position wird mittels Matplotlib geplottet. Nachfolgend ein plot aus dem Simulator.

![Simulator](</doku/simulator/fertiger_sim.png>)

# 3 ROS2-Knoten und Veröffentlichung
Der Motioncontroller-Knoten ist für die steuerung des Roboters zuständig und greift als solches auf die Topics `/RobotPos` und `/RobotCmd` zu. Hierbei ist `/RobotPos` die vom Roboter aktuell übermittelte Position in Roboterkoordinaten und `/RobotCmd` die schnittstelle die das senden von Befehlen an den Roboter erlaubt. Auf `/RobotPos` wird dementsprechend ein subscriber gelegt und auf `/RobotCmd` ein publisher. Ebenso muss für die steuerung und regelung auf ein erkanntes objekt auf das Topic `/predicted_position` subscribed werden.