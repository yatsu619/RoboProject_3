# RoboProject_3
# PickMe --> Pick-and-Identify Controlunit for Kinetic Modules & Entities
Die dokumentation des Projekts ist zu [hier](/doku/general_docs.md) zu finden. Jeder erstellte knoten enthält zudem seine eigene dokumentation welche in der Hauptdokumentation verlink ist. Deren dokus wären hier zu finden:

Motioncontroller: **[motion_docs.md](motion_docs.md)**

Vision Pipeline: **[vision_docs.md](vision_docs.md)**

Waypoint Prediction: **[prediction_docs.md](prediction_docs.md)**

# Starten der Knoten

Unter Ubuntu 22.04 einfach in das verzeichnis des repositories gehen bis in den workspace unterordner (`RoboProject_3/ws`). Hier einfach den `colcon build` command ausführen und entsprechend sourcen mit `source ./install/setup.bash`. Die ROS2 Pakete `ro45_portalrobot_interfaces` sowie `ro45_ros2_pickrobot_serial` müssen gegebenenfalles separat mittels colcon gebaut werden. Dies ist am besten in den jeweiligen readme dateien nachzulesen. Diese sind in der dokumentation verlinkt. Danach können die einzelnen Kntoten wie folgt gestartet werden:

- ros2 launch ro45_ros2_pickrobot_serial launch_nodes.py // Startet die ROS2 serial bridge. Roboter muss über USB eingesteckt sein!
- ros2 run pickme_dev motioncontroller_node 
- ros2 run pickme_dev delay_buffer_node
- ros2 run pickme_dev WaypointPredition_node
