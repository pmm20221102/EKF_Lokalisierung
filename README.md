# EKF_Lokalisierung für lidarslam und vslam
## Erklären
Der EKF verwaltet einen Zustandsvektor (der typischerweise Position, Lage und Geschwindigkeit enthält). Jeder Sensor (IMU, , Lidar) berechnet unabhängig sein eigenes Ergebnis (Lage- oder Geschwindigkeitsschätzung), und der EKF gewichtet und fusioniert diese Ergebnisse anschließend basierend auf der **Kovarianzmatrix (Konfidenzniveau)** jedes Sensors.
## Version
ROS2 humble + Ubuntu 22.04 + gazebo classic
## Package
ros2_control 

slam_toolbox

nav2

rtab_map

robot_lokalization
## EKF (initial Odom und IMU)
1.Gazebo-Emulator öffnen
```bash
ros2 launch fishbot_description gazebo_sim.launch.py
```
2.EKF-Knoten öffnen
```bash
ros2 launch fishbot_navigation2 ekf.launch.py
```
3.Öffnen Sie die Slam-launchdatei.
```bash
ros2 launch  fishbot_navigation2 lidarslam.launch.py
```
4.Konfigurieren Sie die Konfiguration im geöffneten Rivz2.

add --> by topic --> map

add --> by display type --> Robotmodel --> topic name --> robot_description

add --> by display type --> tf

5.An diesem Punkt können Sie xterm zur Tastatursteuerung verwenden und werden feststellen, dass die TF-Übertragung von odom zu map äußerst stabil ist.
## EKF(initial Odom  IMU  Kamera)
1.Gazebo-Emulator öffnen
```bash
ros2 launch fishbot_description gazebo_sim_smallhaus.launch.py
```
2.Öffnen Sie die Slam-launchdatei.
```bash
ros2 launch  fishbot_navigation2 vslam_smallhaus.launch.py
```
3.Konfigurieren Sie die Konfiguration im geöffneten Rivz2.

add --> by topic --> map

add --> by topic --> cloudmap --> pointcloud2 

add --> by display type --> Robotmodel --> topic name --> robot_description

add --> by display type --> tf

4.An diesem Punkt können Sie xterm zur Tastatursteuerung verwenden und die von der Tiefenkamera erzeugte 3D-Punktwolkenkarte anzeigen. Die TF-Übertragung von odom zu map ist derzeit jedoch noch nicht sehr stabil.

## Navigation
1.Gazebo-Emulator öffnen
```bash
ros2 launch fishbot_description gazebo_sim.launch.py
```
2.EKF-Knoten öffnen
```bash
ros2 launch fishbot_navigation2 ekf.launch.py
```
3.Öffnen Sie die nav2-launchdatei.
```bash
ros2 launch  fishbot_navigation2 navigation2.launch.py
```
4.An dieser Stelle können Sie Rviz2 für die Navigationssimulation verwenden. Geben Sie zunächst eine 2D-Anfangspositionsschätzung an und legen Sie anschließend den Zielpunkt und die Wegpunktverfolgung fest.
