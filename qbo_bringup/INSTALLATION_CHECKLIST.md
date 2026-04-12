# 🗂️ CHECKLIST D'INSTALLATION — QBO Bringup Manager

## 📦 Fichiers fournis → Destinations

```
FICHIER SOURCE                          →  DESTINATION
═══════════════════════════════════════════════════════════════════════════════

ManageProfile.srv                       →  ~/qbo_ws/src/qbo_msgs/srv/
qbo_bringup_manager.py                  →  ~/qbo_ws/src/qbo_bringup/qbo_bringup/
launch/profile_minimal.launch.py        →  ~/qbo_ws/src/qbo_bringup/launch/
launch/profile_vision.launch.py         →  ~/qbo_ws/src/qbo_bringup/launch/
launch/profile_navigation.launch.py     →  ~/qbo_ws/src/qbo_bringup/launch/
launch/profile_full.launch.py           →  ~/qbo_ws/src/qbo_bringup/launch/
launch/qbo_system.launch.py             →  ~/qbo_ws/src/qbo_bringup/launch/
test_bringup_manager.py                 →  ~/qbo_ws/src/qbo_bringup/test/
qbo_profile.sh                          →  ~/qbo_ws/ (chmod +x)

═══════════════════════════════════════════════════════════════════════════════
```

## ⚙️ Modifications nécessaires

### 1. qbo_msgs/CMakeLists.txt

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  # ... messages existants ...
  "srv/ManageProfile.srv"    # ← AJOUTER
  DEPENDENCIES std_msgs
)
```

### 2. qbo_bringup/setup.py

```python
entry_points={
    'console_scripts': [
        'qbo_bringup_manager = qbo_bringup.qbo_bringup_manager:main',  # ← AJOUTER
        # ... autres executables ...
    ],
},
```

Vérifier aussi :
```python
data_files=[
    # ...
    (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),  # ← Vérifier présent
],
```

### 3. qbo_social/action_executor.py

**REMPLACER** par la version modifiée fournie (avec client ManageProfile)

## 🔨 Build

```bash
cd ~/qbo_ws

# 1. Build qbo_msgs (pour générer ManageProfile.srv)
colcon build --packages-select qbo_msgs
source install/setup.bash

# 2. Build qbo_bringup (pour installer le manager)
colcon build --packages-select qbo_bringup
source install/setup.bash

# 3. Build qbo_social (pour action_executor modifié)
colcon build --packages-select qbo_social
source install/setup.bash
```

## ✅ Vérification

```bash
# Service message disponible ?
ros2 interface show qbo_msgs/srv/ManageProfile

# Exécutable installé ?
ros2 pkg executables qbo_bringup | grep qbo_bringup_manager

# Launch files copiés ?
ros2 pkg prefix qbo_bringup
# Vérifier dans: install/qbo_bringup/share/qbo_bringup/launch/
```

## 🚀 Test rapide

```bash
# Terminal 1
ros2 run qbo_bringup qbo_bringup_manager

# Terminal 2
./qbo_profile.sh start MINIMAL

# Ou directement :
ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
  "{profile_name: 'MINIMAL', action: 'START'}"
```

## 📊 Ordre de démarrage recommandé

```bash
# Option 1 : Launch complet (recommandé)
ros2 launch qbo_bringup qbo_system.launch.py use_dashboard:=true

# Option 2 : Manuel
# Terminal 1
ros2 run qbo_bringup qbo_bringup_manager

# Terminal 2
ros2 run qbo_social event_adapter

# Terminal 3
ros2 run qbo_social world_model

# Terminal 4
ros2 run qbo_social behavior_engine

# Terminal 5
ros2 run qbo_social action_executor

# Terminal 6
ros2 run qbo_social system_mode_manager

# Terminal 7 (optionnel, dashboard)
ros2 run qbo_social debug_behavior_state
```

## 🧪 Tests

```bash
# Test automatisé complet
ros2 run qbo_bringup qbo_bringup_manager  # Terminal 1
python3 test_bringup_manager.py           # Terminal 2

# Test avec dashboard
ros2 run qbo_social debug_behavior_state  # Observer les changements en live
./qbo_profile.sh start VISION             # Terminal 2
```

## 🐛 Troubleshooting rapide

| Problème                          | Solution                                          |
|-----------------------------------|---------------------------------------------------|
| Service non disponible            | Vérifier `ros2 service list \| grep manage`       |
| Launch file not found             | Vérifier `install/qbo_bringup/share/.../launch/`  |
| Import error ManageProfile        | Rebuild `qbo_msgs`, puis source `install/setup.bash` |
| Nœuds ne démarrent pas            | Vérifier ports série dans launch files            |
| Permission denied qbo_profile.sh  | `chmod +x qbo_profile.sh`                         |

## 📚 Documentation

- **README.md** : Vue d'ensemble
- **INTEGRATION_GUIDE.md** : Guide détaillé avec exemples
- **test_bringup_manager.py** : Suite de tests complète

---

**Quick Start** : Copiez tous les fichiers, modifiez les 3 fichiers (CMakeLists, setup.py, action_executor), rebuild, testez !
