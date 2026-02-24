# qbo_arduqbo

**Version :** 0.1.4

## 📦 Résumé

`qbo_arduqbo` regroupe les contrôleurs ROS 2 qui pilotent et diagnostiquent les cartes Q.bo via ports série et bus I2C. Le nœud principal instancie les contrôleurs selon la configuration, publie un diagnostic global et relaie l'état des QBoards (ID, version, disponibilité).

## 🧭 Cartes gérées

- **QBoard1 (base)** : base mobile, IMU, capteurs, LCD.
- **QBoard2 (head)** : nez RGB, bouche LED, audio.
- **QBoard3 (battery)** : batterie et alimentation.

## ⚙️ Paramètres globaux (nœud qbo_arduqbo)

```yaml
qbo_arduqbo:
  ros__parameters:
    port1: "/dev/ttyQboard1"
    port2: "/dev/ttyQboard2"
    baud1: 115200
    baud2: 115200
    timeout1: 3.0
    timeout2: 3.0

    enable_qboard1: true
    enable_qboard2: true

    enable_base: true
    enable_battery: true
    enable_imu_base: true
    enable_lcd: true
    enable_nose: true
    enable_mouth: true
    enable_audio: true
    enable_sensors: true
```

## 🩺 Diagnostics globaux

- **Arduqbo Status** : statut de démarrage des QBoards, ID/version, contrôleurs activés.
- Criticité : `ERROR` si QBoard1 et QBoard2 absentes, `WARN` si une seule absente, `OK` sinon.

## 🎛️ Contrôleurs

### BaseController (base_ctrl)

**Rôle** : commande de vitesse, odométrie, TF, estimation puissance.

**Paramètres**
- `rate` (double) : fréquence de boucle.
- `topic` (string) : topic commande vitesse (Twist).
- `odom_topic` (string) : topic odométrie publié.
- `tf_odom_broadcast` (bool) : publication TF `odom` → `base_footprint`.

**Topics**
- Publie : `<odom_topic>` (`nav_msgs/msg/Odometry`).
- Abonne : `<topic>` (`geometry_msgs/msg/Twist`).
- TF : `/tf` et `/tf_static`.

**Services**
- `<node>/unlock_motors_stall` (`std_srvs/srv/Empty`).
- `<node>/stop_base` (`std_srvs/srv/Empty`).
- `<node>/set_odometry` (`qbo_msgs/srv/SetOdometry`).

**Diagnostics** (Base Status)
- Criticité : `ERROR` si moteur en erreur/communication, `WARN` si lecture odométrie échoue, `OK` sinon.
- Champs : `Left Motor OK`, `Right Motor OK`, modèle moteur, tension/couple/vitesse, gearbox, encoder, `Estimated Motor Power`.

**Exemples**
```bash
ros2 topic pub /qbo_arduqbo/base_ctrl/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.1}}'
ros2 topic echo /qbo_arduqbo/base_ctrl/odom
ros2 service call /qbo_arduqbo/base_ctrl/set_odometry qbo_msgs/srv/SetOdometry "{x: 1.0, y: 2.0, theta: 1.57}"
ros2 service call /qbo_arduqbo/base_ctrl/stop_base std_srvs/srv/Empty "{}"
ros2 service call /qbo_arduqbo/base_ctrl/unlock_motors_stall std_srvs/srv/Empty "{}"
```

---

### ImuController (imu_ctrl)

**Rôle** : lecture IMU, calibration IMU.

**Paramètres**
- `rate` (double) : fréquence de publication.
- `topic` (string) : base du topic IMU.

**Topics**
- Publie : `<topic>/data` (`sensor_msgs/msg/Imu`).
- Publie : `<topic>/is_calibrated` (`std_msgs/msg/Bool`).

**Services**
- `<topic>/calibrate` (`qbo_msgs/srv/CalibrateIMU`).

**Diagnostics** (IMU Status)
- Criticité : `ERROR` si IMU incomplète, `WARN` si non calibrée, `OK` sinon.
- Champs : présence gyro/accel, modèles et adresses I2C, `IMU calibrated`, `Last calibration`.

**Exemples**
```bash
ros2 topic echo /qbo_arduqbo/imu_ctrl/imu_state/data
ros2 topic echo /qbo_arduqbo/imu_ctrl/imu_state/is_calibrated
ros2 service call /qbo_arduqbo/imu_ctrl/imu_state/calibrate qbo_msgs/srv/CalibrateIMU "{}"
```

---

### BatteryController (battery_ctrl)

**Rôle** : mesure tension et état batterie, estimation autonomie/puissance.

**Paramètres**
- `error_battery_level` (double) : seuil batterie vide.
- `warn_battery_level` (double) : seuil batterie faible.
- `capacity_ah` (double) : capacité batterie.
- `nominal_voltage` (double) : tension nominale.
- `battery_type` (string) : type de batterie.

**Topics**
- Aucun topic applicatif dédié (diagnostics uniquement).
- Abonné à `/diagnostics` pour récupérer la puissance A608.

**Services**
- Aucun.

**Diagnostics** (Battery Status)
- Criticité : `STALE` si pas de communication QBoard3, `WARN`/`ERROR` selon seuils, `OK` sinon.
- Champs : `Voltage`, `Type`, `Nominal Voltage`, `Capacity`, `Status`, `Charge Mode`, `External Power`, `PC On`, `Boards On`, `Charge Mode Description`, `Estimated Runtime`, `Estimated Power`, `Estimated Extras`.

**Exemples**
```bash
ros2 topic echo /diagnostics
```

---

### LcdController (lcd_ctrl)

**Rôle** : affichage LCD (ligne commandée + infos diagnostics).

**Paramètres**
- `rate` (double) : fréquence d'update.
- `topic` (string) : topic d'affichage LCD.

**Topics**
- Abonne : `<topic>` (`qbo_msgs/msg/LCD`).
- Abonne : `/diagnostics` (pour afficher réseau, batterie, temperature CPU).

**Services**
- Aucun.

**Diagnostics** (LCD Status)
- Criticité : `OK` si LCD présent, `ERROR` sinon.
- Champs : `LCD Present`, `LCD Model`, `I2C Address`.

**Exemples**
```bash
ros2 topic pub -1 /qbo_arduqbo/lcd_ctrl/cmd_lcd qbo_msgs/msg/LCD "{msg: 'Hello Q.bo'}"
```

---

### NoseController (nose_ctrl)

**Rôle** : LED RGB du nez, test.

**Paramètres**
- `rate` (double) : fréquence de boucle.
- `topic` (string) : topic couleur nez.

**Topics**
- Abonne : `<topic>` (`qbo_msgs/msg/Nose`).

**Services**
- `<node>/test_leds` (`qbo_msgs/srv/TestLeds`).

**Diagnostics** (Nose Status)
- Criticité : `WARN` si la derniere commande/test a echoue, `OK` sinon.
- Champs : `color_code`, `color_name`.

**Exemples**
```bash
ros2 topic pub -1 /qbo_arduqbo/nose_ctrl/cmd_nose qbo_msgs/msg/Nose "{color: 4}"
ros2 service call /qbo_arduqbo/nose_ctrl/test_leds qbo_msgs/srv/TestLeds "{}"
```

---

### MouthController (mouth_ctrl)

**Rôle** : matrice LED 4x5, test.

**Paramètres**
- `rate` (double) : fréquence de boucle.
- `topic` (string) : topic motif bouche.

**Topics**
- Abonne : `<topic>` (`qbo_msgs/msg/Mouth`).

**Services**
- `<node>/test_leds` (`qbo_msgs/srv/TestLeds`).

**Diagnostics** (Mouth Status)
- Criticité : `WARN` si le test LED a echoue, `OK` sinon.

**Exemples**
```bash
ros2 topic pub -1 /qbo_arduqbo/mouth_ctrl/cmd_mouth qbo_msgs/msg/Mouth "{mouth_image: [true, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, true]}"
ros2 service call /qbo_arduqbo/mouth_ctrl/test_leds qbo_msgs/srv/TestLeds "{}"
```

---

### SensorController (sens_ctrl)

**Rôle** : capteurs distance SRF10/VL53L1X (I2C) et Sharp IR (ADC).

**Paramètres**
- `rate` (double) : fréquence de boucle (default 15.0).
- `topic` (string) : base des topics publies (default `srf10_state`).
- `sensors.<group>.<name>.type` : `srf10`, `VL53L1X`, `gp2d12`, `gp2d120`, `GP2Y0A21YK`.
- `sensors.<group>.<name>.address` : adresse I2C/ADC.
- `sensors.<group>.<name>.frame_id` : frame du capteur.
- `sensors.<group>.<name>.topic` : topic de publication (optionnel).
- `sensors.<group>.<name>.publish_if_obstacle` : publier uniquement si obstacle.

**Topics**
- Publie : `<base_topic>/<name>` (`sensor_msgs/msg/PointCloud`) pour chaque capteur.

**Services**
- Aucun.

**Diagnostics** (Sensors Status)
- Criticité : `WARN` si aucun capteur configure ou si un capteur ne repond plus, `OK` sinon.
- Champs : `Missing SRF10`, `Missing ADC`, `SRF10 count`, `ADC count`, `Board configured`.

**Exemples**
```bash
ros2 topic echo /qbo_arduqbo/sens_ctrl/srf10_state/front_left
```

## 🚀 Lancement

```bash
ros2 launch qbo_arduqbo qbo_full.launch.py
```

Ou en manuel :

```bash
ros2 run qbo_arduqbo qbo_arduqbo --ros-args --params-file src/qbo_arduqbo/config/qboards_config.yaml
```

## 🛠 Dépendances

- ROS 2 Humble
- JetPack 6.x (Ubuntu 22.04)
- Q.bo V2 sur Orin NX
- `qbo_msgs`

**Auteur :** Sylvain Zwolinski
**Licence :** BSD-3-Clause