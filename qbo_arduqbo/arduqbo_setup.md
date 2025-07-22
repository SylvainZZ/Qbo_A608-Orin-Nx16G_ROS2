# qbo_arduqbo

**Version :** 0.1.3

## 📦 Description du package

Contrôleurs ROS 2 pour la gestion matérielle des cartes Q.bo (QBoard1, QBoard2, QBoard3) sur le robot Q.bo V2. Ce package permet de piloter et diagnostiquer les différentes fonctionnalités matérielles embarquées via des interfaces série et I2C.

## 📦 Fonctionnalités

- **QBoard1 (base)** :
  - Contrôleur de base (odométrie, vitesses)
  - IMU (accéléromètre + gyroscope)
  - Écran LCD (affichage d'informations)
- **QBoard2 (head)** :
  - LED Nez RGB (mode 8 couleurs)
  - Afficheur Bouche (matrice 4x5 LED bleues)
- **QBoard3 (battery)** :
  - Surveillance batterie (tension, charge, autonomie)
  - Statut de l'alimentation (PC, charge, secteur)

## ⚙️ Compilation du package

```bash
colcon build --packages-select qbo_arduqbo qbo_msgs --cmake-clean-cache
colcon build --packages-select qbo_msgs   --cmake-clean-cache   --allow-overriding qbo_msgs
source install/setup.bash
```

## 🚀 Lancement

Lancement global avec :
```bash
ros2 launch qbo_arduqbo qbo_full.launch.py
```

Ou lancement manuel :
```bash
ros2 run qbo_arduqbo qbo_arduqbo --ros-args --params-file src/qbo_arduqbo/config/qboards_config.yaml
ros2 run qbo_arduqbo qbo_arduqbo   --ros-args --params-file src/qbo_arduqbo/config/qboards_config.yaml --log-level DEBUG
```

## ⚙️ Paramètres de configuration

Extrait de `qboards_config.yaml` :
```yaml
qbo_arduqbo:
  ros__parameters:
      enable_qboard1: true
      port1: "/dev/ttyQboard1"
      baud1: 115200
      timeout1: 3.0
      enable_qboard2: true
      port2: "/dev/ttyQboard2"
      baud2: 115200
      timeout2: 3.0

      enable_base: true
      enable_battery: true
      enable_imu_base: true
      enable_lcd: true
      enable_nose: true
      enable_mouth: true
      enable_audio: true

/qbo_arduqbo/battery_ctrl:
  ros__parameters:
    rate: .0
    error_battery_level: 11.5
    warn_battery_level: 12.2
    capacity_ah: 10.0
    battery_type: LiFePo4
    nominal_voltage: 13.2

/qbo_arduqbo/base_ctrl:
  ros__parameters:
    rate: 15.0
    topic: "base_ctrl/cmd_vel"
    odom_topic: "base_ctrl/odom"
    tf_odom_broadcast: true

/qbo_arduqbo/imu_ctrl:
  ros__parameters:
    rate: 15.0
    topic: "imu_ctrl/imu_state"

/qbo_arduqbo/lcd_ctrl:
  ros__parameters:
    rate: 1.0
    topic: "lcd_ctrl/cmd_lcd"

/qbo_arduqbo/nose_ctrl:
  ros__parameters:
    rate: 10.0
    topic: "nose_ctrl/cmd_nose"

/qbo_arduqbo/mouth_ctrl:
  ros__parameters:
    rate: 10.0
    topic: "mouth_ctrl/cmd_mouth"

/qbo_arduqbo/audio_ctrl:
  ros__parameters:
    rate: 15.0
    topic: "audio_ctrl/mic_report"

```

## 📡 Contrôleurs disponibles

| Contrôleur           | Type     | Description                            | ROS Topic / Service                  |
|----------------------|----------|----------------------------------------|--------------------------------------|
| `base_controller`    | Node     | Odométrie + commande de base           | `/cmd_vel`, `/odom`                  |
| `imu_controller`     | Node     | Lecture IMU + calibration              | `/imu_state/data`, `/imu_state/calibrate` |
| `lcd_controller`     | Node     | Affichage info système/batterie        | `/cmd_lcd`, `/diagnostics`           |
| `nose_controller`    | Node     | LED RGB Nez                            | `/cmd_nose`                          |
| `mouth_controller`   | Node     | Afficheur LED 4x5                      | `/cmd_mouth`, `/test_mouth` (srv)    |
| `battery_controller` | Node     | Surveillance batterie (via I2C)        | `/diagnostics`                       |

## 🧪 Services utiles

### Imposer une odométrie ou mettre à zéro
```bash
ros2 service call /qbo_arduqbo/base_ctrl/set_odometry qbo_msgs/srv/SetOdometry "{x: 1.0, y: 2.0, theta: 1.57}"
```

### Stoper le déplacement ou débloquer les moteurs
```bash
ros2 service call /qbo_arduqbo/base_ctrl/stop_base std_srvs/srv/Empty "{}"
ros2 service call /qbo_arduqbo/base_ctrl/unlock_motors_stall std_srvs/srv/Empty "{}"
```

### Calibrer l'IMU
```bash
ros2 service call /qbo_arduqbo/imu_ctrl/imu_state/calibrate qbo_msgs/srv/CalibrateIMU "{}"
```

### Tester l'éclairage du nez
```bash
ros2 topic pub -1 /qbo_arduqbo/cmd_nose qbo_msgs/msg/Nose "{color: 4}"
# 0 = Off
# 1 = Red
# 2 = Blue
# 3 = Violet
# 4 = Green
# 5 = Yellow (Green+Red)
# 6 = Magenta (Blue+Red)
# 7 = White (RGB on)

ros2 service call /qbo_arduqbo/nose_ctrl/test_leds qbo_msgs/srv/TestLeds "{}"
```


### Tester les LED de la bouche
```bash
ros2 service call /qbo_arduqbo/mouth_ctrl/test_leds qbo_msgs/srv/TestLeds "{}"
ros2 service call /qbo_arduqbo/audio_ctrl/set_mouth_animation qbo_msgs/srv/SetMouthAnimation "{enable: true}"

```

## 🩺 Diagnostic

Tous les contrôleurs publient dans `/diagnostics` via `diagnostic_updater`.

Exemples :
- Tension de la batterie
- Températures de l’Orin NX
- Présence des capteurs I2C
- Contrôleur IMU calibré ou non
- État de l’afficheur LCD

## 🛠 Développement

### Compilation

```bash
colcon build --packages-select qbo_arduqbo
source install/setup.bash
```

### Dépendances clés

- ROS 2 Humble
- JetPack 6.2 (Ubuntu 22.04)
- Q.bo V2 sur Orin NX
- [`qbo_msgs`](https://github.com/...)

### Structure

```
qbo_arduqbo/
├── src/
│   ├── controllers/
│   ├── drivers/
│   └── ...
├── include/
├── launch/
├── config/
├── CMakeLists.txt
└── package.xml
```

## 📋 TODO

- Implémentation du contrôleur IMU Head (QBoard3)
- Affichage dynamique + personnalisation LCD
- Amélioration test LED bouche (animation ?)
- Migration vers composant unique ?

---

ros2 run qbo_arduqbo qbo_arduqbo   --ros-args --params-file src/qbo_arduqbo/config/qboards_config.yaml
ros2 run qbo_arduqbo qbo_arduqbo   --ros-args --params-file src/qbo_arduqbo/config/qboards_config.yaml --log-level DEBUG
colcon build --packages-select qbo_msgs   --cmake-clean-cache   --allow-overriding qbo_msgs
ros2 service call /base_ctrl/set_odometry qbo_msgs/srv/SetOdometry "{x: 1.0, y: 2.0, theta: 1.57}"
ros2 service call /base_ctrl/stop_base std_srvs/srv/Empty "{}"
ros2 service call /base_ctrl/unlock_motors_stall std_srvs/srv/Empty "{}"
ros2 launch qbo_arduqbo qbo_full.launch.py
ros2 run diagnostic_aggregator aggregator_node --ros-args --params-file /home/qbo-v2/qbo_ws/src/qbo_arduqbo/config/diagnostics_aggregator.yaml
ros2 topic pub -1 /cmd_nose qbo_msgs/msg/Nose "{color: 4}"
ros2 service call /mouth_ctrl/test_leds qbo_msgs/srv/TestMouthLeds
rosservice call /base_ctrl/stop_base std_srvs/srv/Empty
ros2 topic pub  /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.5}}'


**Auteur :** Sylvain Zwolinski
**Licence :** BSD-3-Clause