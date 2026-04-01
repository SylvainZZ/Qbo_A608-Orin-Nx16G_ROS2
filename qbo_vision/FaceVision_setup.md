
# FaceVision Setup - QBO Robot

Guide de configuration et utilisation du système de vision pour le robot QBO.

---

## 🛠️ Compilation

```bash
colcon build --packages-select qbo_vision --cmake-clean-cache --allow-overriding qbo_vision
source install/setup.bash
```

---

## 📷 Calibration Caméra

### 1. Démarrer la caméra USB

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p pixel_format:=yuyv \
  -p image_width:=640 \
  -p image_height:=480 \
  -p framerate:=30.0
```

### 2. Lancer l'outil de calibration

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.0285 \
  --no-service-check \
  --ros-args \
  -r image:=/image_raw
```

### 3. Extraire le fichier de calibration

```bash
mkdir -p /tmp/calib_temp && \
tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/calib_temp && \
mv /tmp/calib_temp/ost.yaml ~/qbo_ws/src/qbo_vision/config/calibration/new_calibration.yaml && \
rm -r /tmp/calib_temp
```

---

## 🚀 Démarrage Caméra

```bash
ros2 launch qbo_vision camera_with_calibration.launch.py
```

---

## 👤 Node Face Tracker

### Démarrage

```bash
ros2 run qbo_vision face_tracker_node --ros-args \
  --params-file ~/qbo_ws/src/qbo_vision/config/face_tracker.yaml
```

### Visualisation

```bash
# Image de debug avec détections
ros2 run rqt_image_view rqt_image_view /qbo_face_tracking/debug_image

# Topic publié (migration vers FaceObservation uniquement)
ros2 topic echo /qbo_face_tracking/face_observation
```

### Contrôle

#### ✅ Activer le tracking

```bash
ros2 service call /qbo_face_tracker/enable std_srvs/srv/SetBool "{data: true}"
```

**Réponse attendue :**
```
success: True
message: 'Face tracking enabled'
```

> ⚠️ Le CPU va augmenter quand le tracking est activé.

#### ❌ Désactiver le tracking

```bash
ros2 service call /qbo_face_tracker/enable std_srvs/srv/SetBool "{data: false}"
```

> 💡 Le CPU retourne à ~0% quand désactivé.

### Options de démarrage

```bash
# Démarrer avec le tracking déjà activé
ros2 run qbo_vision face_tracker --ros-args -p start_enabled:=true

# Activer les logs détaillés
ros2 run qbo_vision face_tracker --ros-args --log-level debug
```

### Commandes utiles

```bash
# Voir les services disponibles
ros2 service list | grep enable

# Voir le type du service
ros2 service type /qbo_face_tracker/enable
```

---

## 🎯 Node Face Follower

### Démarrage

```bash
ros2 run qbo_vision face_follower_node --ros-args \
  --params-file src/qbo_vision/config/face_follower.yaml
```

### ⚙️ Contrôle via Service (SBE - Social Behavior Engine)

Le follower expose un service pour activer/désactiver les mouvements indépendamment.

#### Activer tête + rotation base

```bash
ros2 service call /qbo_face_following/set_status qbo_msgs/srv/SetFollowerStatus \
  "{enable_head_movement: true, enable_base_rotation: true}"
```

#### Tête seule (base immobile)

```bash
ros2 service call /qbo_face_following/set_status qbo_msgs/srv/SetFollowerStatus \
  "{enable_head_movement: true, enable_base_rotation: false}"
```

#### Tout désactiver

```bash
ros2 service call /qbo_face_following/set_status qbo_msgs/srv/SetFollowerStatus \
  "{enable_head_movement: false, enable_base_rotation: false}"
```

### 📊 Topic de Statut

Le follower publie en continu son statut avec la position 3D du visage :

```bash
ros2 topic echo /qbo_face_following/status
```

#### Champs importants du FollowerStatus

| Champ | Type | Description |
|-------|------|-------------|
| `tracking_state` | uint8 | 0=IDLE, 1=SEARCHING, 2=TRACKING, 3=BLOCKED |
| `face_x`, `face_y`, `face_z` | float32 | Coordonnées 3D du visage dans le repère robot (mètres) |
| `face_distance` | float32 | Distance au visage (mètres) |
| `head_pan`, `head_tilt` | float32 | Position actuelle de la tête (radians) |
| `head_movement_enabled` | bool | Mouvements de tête activés |
| `base_rotation_enabled` | bool | Rotation de la base activée |
| `faces_detected` | uint8 | Nombre de visages détectés |

### 📝 Notes Importantes

- ❌ Le contrôle linéaire (avance/recul) a été retiré du follower
- ✅ La navigation est déléguée au SBE qui utilise `move_base`
- ✅ Le follower gère uniquement : tête (pan/tilt) + rotation base (centrage)
- ✅ La position 3D du visage permet au SBE de planifier la navigation

---

## 🧠 Node Face Recognition

### Surveillance des résultats

```bash
ros2 topic echo /qbo_face_recognition/result
```

### Enregistrer une nouvelle personne

```bash
ros2 service call /face_recognition/start_enroll qbo_msgs/srv/StartEnrollPerson "{name: 'Sylvain'}"
```

### Lister les personnes enregistrées

```bash
ros2 service call /face_recognition/list_persons qbo_msgs/srv/ListPersons
```

---

## 📦 Dépendances

```bash
sudo apt install ros-humble-rqt-graph
sudo apt install ros-humble-image-pipeline
```

---

## 🔧 Commandes Diverses Utiles

### Paramètres ROS

```bash
rosparam list /face_follower
```

### Topics

```bash
# Vérifier la fréquence de publication
ros2 topic hz /camera_left/image_raw
ros2 topic hz /qbo_face_tracking/face_observation

# Vérifier le délai
ros2 topic delay /camera_left/image_raw
```

### Caméra

```bash
# Lister les formats supportés
v4l2-ctl --list-formats-ext -d /dev/video0
```

### TF (Transform)

```bash
# Visualiser l'arbre des transformations
ros2 run tf2_tools view_frames

# Publier une transformation statique
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

### Processus

```bash
# Arrêter le node RealSense
pkill -f realsense2_camera_node
```

### Navigation

```bash
# Envoyer un goal de navigation
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
    header:
      frame_id: 'odom'
    pose:
      position:
        x: 0.5
        y: 0.2
        z: 0.0
      orientation:
        z: 0.0
        w: 1.0
"
```

---

## 📊 Architecture du Système

### Flux de données

```
Camera → face_tracker → FaceObservation → face_follower → FollowerStatus → SBE
                            ↓                    ↓
                     face_recognition      head + base rotation
                            ↓
                    FaceRecognitionResult
```

### Messages et Topics

#### 📡 `/qbo_face_tracking/face_observation`

**Type :** `qbo_msgs/FaceObservation`
**Publié par :** `face_tracker`

**Contenu :**
- `tracking_state` : 0=DISABLED, 1=SEARCH, 2=CANDIDATE, 3=TRACKING
- `center_x`, `center_y` : Position dans l'image
- `x`, `y`, `width`, `height` : Bounding box
- `landmarks[10]` : 5 points YuNet (yeux, nez, coins bouche)
- `distance` : Distance en mètres (par triangulation)
- `detector_score`, `quality` : Scores de confiance
- `face_id`, `track_id` : Identifiants

#### 📡 `/qbo_face_following/status`

**Type :** `qbo_msgs/FollowerStatus`
**Publié par :** `face_follower` (~30Hz)

**Contenu :**
- `tracking_state` : 0=IDLE, 1=SEARCHING, 2=TRACKING, 3=BLOCKED
- `face_x`, `face_y`, `face_z` : Coordonnées 3D dans le repère robot (mètres)
- `face_distance` : Distance en mètres
- `head_pan`, `head_tilt` : Position actuelle de la tête (radians)
- `head_movement_enabled`, `base_rotation_enabled` : Capacités activées
- `blocking_reason` : Raison de blocage si applicable

#### 📡 `/qbo_face_recognition/result`

**Type :** `qbo_msgs/FaceRecognitionResult`
**Publié par :** `face_recognition`

**Contenu :**
- `recognized` : bool
- `name` : string
- `confidence` : float
- `face_id` : uint32

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/qbo_face_tracker/enable` | `std_srvs/SetBool` | Activer/désactiver le tracking (économie CPU) |
| `/qbo_face_following/set_status` | `qbo_msgs/SetFollowerStatus` | Contrôler les capacités du follower (SBE) |
| `/face_recognition/start_enroll` | `qbo_msgs/StartEnrollPerson` | Enregistrer une nouvelle personne |
| `/face_recognition/list_persons` | `qbo_msgs/ListPersons` | Lister les personnes enregistrées |

---

## 🤖 Architecture Orientée SBE

### Simplification du face_follower

Le `face_follower` a été transformé en **"capteur intelligent"** :

- ✅ **Gère :** Contrôle tête (pan/tilt) + Rotation base (centrage visage)
- ❌ **Ne gère plus :** Contrôle linéaire (avance/recul)

### Rôle du SBE (Social Behavior Engine)

Le SBE prend en charge la prise de décision de haut niveau :

1. S'abonne à `/qbo_face_following/status`
2. Reçoit la position 3D du visage (`face_x`, `face_y`, `face_z`, `distance`)
3. Planifie la navigation avec `move_base`
4. Peut activer/désactiver dynamiquement les capacités du follower

### Avantages

| Avantage | Description |
|----------|-------------|
| **Séparation claire** | Perception (follower) vs. décision (SBE) |
| **Flexibilité** | Le SBE peut désactiver la rotation pour éviter les conflits |
| **Évolutivité** | Ajout facile de comportements complexes (approche, fuite, etc.) |
| **Coordination** | Le SBE orchestre follower + navigation + autres comportements |

---

## 📐 Coordonnées 3D du Visage

### Repère Robot (base_link)

| Axe | Direction | Description |
|-----|-----------|-------------|
| **X** | Avant | Positif = devant le robot |
| **Y** | Gauche | Positif = à gauche, Négatif = à droite |
| **Z** | Haut | Positif = au-dessus de la base |

### Formules de Calcul

```
X = distance × cos(tilt) × cos(pan)
Y = distance × cos(tilt) × sin(pan)
Z = -distance × sin(tilt)  [signe inversé pour convention QBO]
```

### Exemple

**Entrées :**
- `face_distance` = 1.37 m
- `head_pan` = -0.13 rad (~7° vers la droite)
- `head_tilt` = -0.52 rad (~30° vers le haut)

**Sorties :**
- `face_x` = **1.18 m** (devant)
- `face_y` = **-0.15 m** (légèrement à droite)
- `face_z` = **0.68 m** (au-dessus de la base)

---

## 📝 Logs Optimisés

Les logs ont été allégés pour une meilleure lisibilité :

- ✅ Affichage uniquement sur changements d'état importants
- ✅ Logs détaillés disponibles avec `--log-level debug`

### États Affichés

| Icône | État | Description |
|-------|------|-------------|
| 🔴 | **DISABLED** | Tracker désactivé |
| 🔍 | **SEARCH** | Recherche de visage |
| ⚠️  | **CANDIDATE** | Visage potentiel détecté |
| ✅ | **TRACKING** | Suivi actif du visage |

---

**Documentation mise à jour le :01 Avril 2026
**Auteur : Zwolinski Sylvain



