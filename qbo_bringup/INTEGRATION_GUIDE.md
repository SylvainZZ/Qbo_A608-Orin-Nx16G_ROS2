# 🚀 QBO Bringup Manager — Guide d'Intégration

Architecture complète pour la gestion dynamique des profils de nœuds QBO.
Mimique le comportement des Lifecycle Nodes sans nécessiter que chaque nœud implémente l'interface Lifecycle.

## 📦 Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    QBO Social Architecture                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  system_mode_manager  ──┬──► START_PROFILE intent               │
│  (détecte nœuds)        │                                       │
│                         │                                       │
│  behavior_engine ───────┴──► /qbo_social/intent_raw             │
│  (orchestration)            │                                   │
│                             │                                   │
│  action_executor ◄──────────┘                                   │
│  (exécution)                │                                   │
│                             │                                   │
│                             ▼                                   │
│              ┌──────────────────────────┐                       │
│              │ qbo_bringup_manager      │                       │
│              │ (service ROS 2)          │                       │
│              └───────────┬──────────────┘                       │
│                          │                                      │
│         ┌────────────────┼────────────────┐                     │
│         ▼                ▼                ▼                     │
│   profile_minimal  profile_vision  profile_full                 │
│   (launch files)   (launch files)  (launch files)               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## 📁 Structure des fichiers

```
qbo_bringup/
├── qbo_bringup_manager.py          # Node manager (service ROS 2)
├── launch/
│   ├── profile_minimal.launch.py   # Profil base
│   ├── profile_vision.launch.py    # + face tracking
│   ├── profile_navigation.launch.py # + SLAM/Nav
│   └── profile_full.launch.py      # Tous nœuds
└── CMakeLists.txt / setup.py       # Build configuration

qbo_msgs/
└── srv/
    └── ManageProfile.srv            # Service definition

qbo_social/
└── action_executor.py               # Client du service (modifié)
```

---

## 🔧 Installation

### Étape 1 : Ajouter le service message

1. **Copier le fichier `ManageProfile.srv`** dans :
   ```
   ~/qbo_ws/src/qbo_msgs/srv/ManageProfile.srv
   ```

2. **Modifier `qbo_msgs/CMakeLists.txt`** pour ajouter le service :
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     # ... autres messages ...
     "srv/ManageProfile.srv"
     DEPENDENCIES std_msgs
   )
   ```

3. **Rebuild qbo_msgs** :
   ```bash
   cd ~/qbo_ws
   colcon build --packages-select qbo_msgs
   source install/setup.bash
   ```

### Étape 2 : Ajouter le qbo_bringup_manager

1. **Copier `qbo_bringup_manager.py`** dans :
   ```
   ~/qbo_ws/src/qbo_bringup/qbo_bringup/qbo_bringup_manager.py
   ```

2. **Modifier `qbo_bringup/setup.py`** pour ajouter l'exécutable :
   ```python
   entry_points={
       'console_scripts': [
           'qbo_bringup_manager = qbo_bringup.qbo_bringup_manager:main',
           # ... autres executables ...
       ],
   },
   ```

3. **Rebuild qbo_bringup** :
   ```bash
   cd ~/qbo_ws
   colcon build --packages-select qbo_bringup
   source install/setup.bash
   ```

### Étape 3 : Ajouter les launch files

1. **Copier les 4 launch files** dans :
   ```
   ~/qbo_ws/src/qbo_bringup/launch/
     - profile_minimal.launch.py
     - profile_vision.launch.py
     - profile_navigation.launch.py
     - profile_full.launch.py
   ```

2. **Vérifier que setup.py inclut le répertoire launch** :
   ```python
   data_files=[
       ('share/ament_index/resource_index/packages',
           ['resource/' + package_name]),
       ('share/' + package_name, ['package.xml']),
       (os.path.join('share', package_name, 'launch'),
           glob('launch/*.launch.py')),  # ← Important !
   ],
   ```

3. **Rebuild** :
   ```bash
   cd ~/qbo_ws
   colcon build --packages-select qbo_bringup
   source install/setup.bash
   ```

### Étape 4 : Mettre à jour action_executor

1. **Remplacer le contenu de `action_executor.py`** par la version modifiée.

2. **Rebuild qbo_social** :
   ```bash
   cd ~/qbo_ws
   colcon build --packages-select qbo_social
   source install/setup.bash
   ```

---

## 🚦 Utilisation

### Démarrer le manager

```bash
# Terminal 1 : Démarrer le bringup manager
ros2 run qbo_bringup qbo_bringup_manager
```

Output attendu :
```
======================================================================
QBO Bringup Manager started
  Service: /qbo_bringup/manage_profile
  Launch directory: /path/to/qbo_ws/install/qbo_bringup/share/qbo_bringup/launch
  Available profiles: ['MINIMAL', 'VISION', 'NAVIGATION', 'FULL']
======================================================================
```

### Commandes manuelles (test)

#### Démarrer un profil
```bash
ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
  "{profile_name: 'MINIMAL', action: 'START'}"
```

#### Vérifier l'état
```bash
ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
  "{profile_name: 'MINIMAL', action: 'STATUS'}"
```

#### Arrêter un profil
```bash
ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
  "{profile_name: 'MINIMAL', action: 'STOP'}"
```

#### Redémarrer
```bash
ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
  "{profile_name: 'VISION', action: 'RESTART'}"
```

### Intégration avec qbo_social (automatique)

Le système fonctionne automatiquement via la chaîne :

1. **system_mode_manager** détecte des nœuds manquants
2. Publie un event `START_PROFILE` avec payload JSON
3. **behavior_engine** transforme en `BehaviorIntent`
4. **action_executor** reçoit l'intent et appelle le service

```bash
# Terminal 1 : Bringup manager
ros2 run qbo_bringup qbo_bringup_manager

# Terminal 2 : System mode manager
ros2 run qbo_social system_mode_manager

# Terminal 3 : Action executor
ros2 run qbo_social action_executor

# Terminal 4 : Simuler un nœud manquant (test)
ros2 topic pub /qbo_social/events qbo_msgs/msg/SocialEvent \
  "{event_type: 'NODE_MISSING', payload_json: '{\"profile\":\"VISION\",\"missing_nodes\":[\"qbo_vision\"]}'}"
```

---

## 🧪 Tests

### Test 1 : Cycle de vie complet (MINIMAL)

```bash
# 1. Démarrer
ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
  "{profile_name: 'MINIMAL', action: 'START'}"

# Attendre 5 secondes

# 2. Vérifier nœuds actifs
ros2 node list | grep qbo

# 3. Status
ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
  "{profile_name: 'MINIMAL', action: 'STATUS'}"

# 4. Arrêter
ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
  "{profile_name: 'MINIMAL', action: 'STOP'}"
```

### Test 2 : Escalade de profils

```bash
# Démarrer MINIMAL → VISION → FULL
ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
  "{profile_name: 'MINIMAL', action: 'START'}"

sleep 5

ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
  "{profile_name: 'VISION', action: 'START'}"

sleep 5

ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
  "{profile_name: 'FULL', action: 'START'}"
```

### Test 3 : Intégration avec diagnostics

```bash
# 1. Lancer le dashboard pour voir les nœuds actifs en temps réel
ros2 run qbo_social debug_behavior_state

# 2. Dans un autre terminal, démarrer/arrêter des profils
ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
  "{profile_name: 'VISION', action: 'START'}"

# → Observe dans le dashboard les nœuds qui apparaissent
```

---

## 🛠️ Personnalisation

### Ajouter un nouveau profil

1. **Créer un nouveau launch file** :
   ```python
   # qbo_bringup/launch/profile_custom.launch.py
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(package='...', executable='...', name='...')
       ])
   ```

2. **Enregistrer dans qbo_bringup_manager.py** :
   ```python
   PROFILE_LAUNCH_FILES = {
       # ... existants ...
       "CUSTOM": "profile_custom.launch.py",
   }
   ```

3. **Rebuild et tester** :
   ```bash
   colcon build --packages-select qbo_bringup
   source install/setup.bash
   ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
     "{profile_name: 'CUSTOM', action: 'START'}"
   ```

### Adapter les nœuds dans les launch files

Modifier les fichiers `profile_*.launch.py` pour correspondre à votre configuration :
- Ports série (`/dev/ttyUSB0`, etc.)
- Topics ROS
- Paramètres des nœuds

---

## 🔍 Debugging

### Vérifier que le service est accessible

```bash
ros2 service list | grep manage_profile
# → /qbo_bringup/manage_profile

ros2 service type /qbo_bringup/manage_profile
# → qbo_msgs/srv/ManageProfile
```

### Logs du manager

```bash
ros2 run qbo_bringup qbo_bringup_manager --ros-args --log-level debug
```

### Vérifier les nœuds lancés

```bash
ros2 node list
ros2 topic list
ros2 topic echo /diagnostics
```

---

## ✅ Avantages de cette architecture

✅ **Séparation des responsabilités** : chaque node a un rôle clair
✅ **Réutilisable** : service accessible par n'importe quel node
✅ **Robuste** : gestion propre du cycle de vie avec l'API Launch
✅ **Testable** : service testable indépendamment
✅ **Évolutif** : facile d'ajouter de nouveaux profils
✅ **Compatible** : mime Lifecycle sans modifier les nodes existants
✅ **Centralisé** : logs et contrôle en un seul endroit

---

## 🎯 Prochaines étapes

- [ ] Implémenter la gestion fine des nœuds individuels (via `target_nodes`)
- [ ] Ajouter un monitoring de santé des profils (heartbeat)
- [ ] Créer un CLI dédié : `qbo profile start VISION`
- [ ] Intégrer avec un dashboard web (rqt plugin)
- [ ] Ajouter la persistance de l'état (reload après reboot)

---

## 📚 Références

- [ROS 2 Launch API](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS 2 Managed Nodes (Lifecycle)](https://design.ros2.org/articles/node_lifecycle.html)
- [QBO Social Dashboard](debug_behavior_state.py)

---

**Auteur** : QBO Project Team
**Date** : Avril 2026
**Version** : 1.0.0
