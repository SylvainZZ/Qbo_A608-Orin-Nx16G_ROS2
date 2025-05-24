
# qbo_arduqbo

**Version :** 0.1.2

## 📦 Description du package

Ce package ROS 2 (`Humble`) est conçu pour contrôler des servos Dynamixel AX12A / AX18A sur la plateforme Q.bo équipée d'une Jetson Orin NX (JetPack 6.2, Ubuntu 22.04) et de la carte A608.

Il permet :

- Le contrôle Pan & Tilt via le topic `/cmd_joints`
- La publication d'états moteurs (`/joint_states`, `/dynamixel_state`)
- La supervision via `/diagnostics`
- Le contrôle du couple via le service `/torque_enable`
- Une configuration dynamique des servos via un fichier YAML

---

## ✅ Installation des dépendances

```bash
sudo apt install ros-humble-dynamixel-workbench
sudo apt install ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-diagnostic-updater
```

---

## 🧪 Utilitaire Python pour scanner le bus Dynamixel

Permet de détecter les servos connectés (adapter le port et la vitesse à 57600 / 1000000 / 115200 selon les cas).

```bash
# Vérifier les ports disponibles :
ls /dev/ttyUSB*
ls /dev/ttyAMA*

# Installer la dépendance :
sudo apt install python3-serial

# Ajouter l'utilisateur au groupe dialout, puis redémarrer la session :
sudo usermod -a -G dialout $USER
groups

# Lancer le scan :
python3 src/qbo_arduqbo/scan_dxl.py
```

---

## ⚙️ Compilation du package

```bash
colcon build --packages-select qbo_arduqbo qbo_msgs --cmake-clean-cache
source install/setup.bash
```

---

## ▶️ Commandes de test

### 🚀 Lancement du contrôleur Dynamixel

```bash
ros2 run qbo_arduqbo qbo_dynamixel   --ros-args --params-file src/qbo_arduqbo/config/dynamixel_config.yaml
```

---

### 🎮 Commande de mouvement (publisher)

```bash
ros2 topic pub -1 /cmd_joints sensor_msgs/JointState   "{name: ['head_pan_joint'], position: [2.0], velocity: [3.5]}"
```

---

### 🧩 Contrôle du couple (services)

```bash
ros2 service call /head_pan_joint/torque_enable qbo_msgs/srv/TorqueEnable   "{torque_enable: false}"

ros2 service call /head_tilt_joint/torque_enable qbo_msgs/srv/TorqueEnable   "{torque_enable: false}"
```

---

### 📡 Surveillance des topics

```bash
ros2 topic echo /dynamixel_state
ros2 topic echo /joint_states
ros2 topic echo /diagnostics
```

---

### 🧩 Chagement des paramétres à la volé (rosparam)

```bash
ros2 param set /qbo_dynamixel dynamixel.motors.head_pan_joint.max_angle_degrees 60.0
ros2 param set /qbo_dynamixel dynamixel.motors.head_pan_joint.min_angle_degrees -60.0
ros2 param set /qbo_dynamixel dynamixel.motors.head_pan_joint.range 300.0
ros2 param set /qbo_dynamixel dynamixel.motors.head_pan_joint.ticks 1024
ros2 param set /qbo_dynamixel dynamixel.motors.head_pan_joint.neutral 512
```

---

## 📁 Structure YAML attendue (extrait)

```yaml
qbo_dynamixel:
  ros__parameters:
    dynamixel.motor_keys: ["motor_1", "motor_2"]

    dynamixel.motors.motor_1.name: head_pan_joint
    dynamixel.motors.motor_1.id: 1
    dynamixel.motors.motor_1.neutral: 512
    ...

    dynamixel.motors.motor_2.name: head_tilt_joint
    dynamixel.motors.motor_2.id: 8
    dynamixel.motors.motor_2.neutral: 480
    ...
```

**Auteur :** Sylvain Zwolinski
**Licence :** BSD-3-Clause

