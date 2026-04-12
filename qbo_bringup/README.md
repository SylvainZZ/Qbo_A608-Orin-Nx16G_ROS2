# 🤖 QBO Bringup Manager — Implémentation Complète

Ce dossier contient tous les fichiers nécessaires pour implémenter le **QBO Bringup Manager**, une solution robuste de gestion des profils de nœuds qui mimique le comportement des Lifecycle Nodes.

## 📋 Contenu du dossier

```
qbo_bringup_implementation/
├── ManageProfile.srv                  # Service message definition
├── qbo_bringup_manager.py             # Node manager principal
├── launch/
│   ├── profile_minimal.launch.py      # Profil base
│   ├── profile_vision.launch.py       # + vision
│   ├── profile_navigation.launch.py   # + navigation
│   └── profile_full.launch.py         # Complet
├── test_bringup_manager.py            # Suite de tests automatisés
├── INTEGRATION_GUIDE.md               # Guide complet d'installation
└── README.md                          # Ce fichier
```

## 🎯 Objectif

Permettre le démarrage/arrêt dynamique de profils de nœuds via un service ROS 2, orchestré par `system_mode_manager` et `behavior_engine`, et exécuté via `action_executor`.

## 🏗️ Architecture

```
system_mode_manager → behavior_engine → action_executor
                                            ↓
                                    (service call)
                                            ↓
                                  qbo_bringup_manager
                                            ↓
                            ┌───────────────┼───────────────┐
                            ↓               ↓               ↓
                      MINIMAL          VISION            FULL
                   (launch files)   (launch files)  (launch files)
```

## 🚀 Installation rapide

### 1. Copier les fichiers aux bons emplacements

```bash
# Service message
cp ManageProfile.srv ~/qbo_ws/src/qbo_msgs/srv/

# Node manager
cp qbo_bringup_manager.py ~/qbo_ws/src/qbo_bringup/qbo_bringup/

# Launch files
cp launch/*.launch.py ~/qbo_ws/src/qbo_bringup/launch/

# Script de test
cp test_bringup_manager.py ~/qbo_ws/src/qbo_bringup/test/
```

### 2. Modifier les CMakeLists.txt / setup.py

**qbo_msgs/CMakeLists.txt** :
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ManageProfile.srv"
  # ... autres messages ...
)
```

**qbo_bringup/setup.py** :
```python
entry_points={
    'console_scripts': [
        'qbo_bringup_manager = qbo_bringup.qbo_bringup_manager:main',
    ],
},
```

### 3. Rebuild

```bash
cd ~/qbo_ws
colcon build --packages-select qbo_msgs qbo_bringup qbo_social
source install/setup.bash
```

### 4. Tester

```bash
# Terminal 1
ros2 run qbo_bringup qbo_bringup_manager

# Terminal 2
ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
  "{profile_name: 'MINIMAL', action: 'START'}"
```

## 📖 Documentation complète

Consultez [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) pour :
- Instructions d'installation détaillées
- Guide d'utilisation
- Tests et debugging
- Personnalisation

## 🧪 Tests automatisés

```bash
# Démarrer le manager
ros2 run qbo_bringup qbo_bringup_manager

# Dans un autre terminal, lancer les tests
python3 test_bringup_manager.py
```

## 📦 Profils disponibles

| Profil       | Nœuds inclus                                           |
|--------------|--------------------------------------------------------|
| **MINIMAL**  | qbo_arduqbo, qbo_driver, qbo_audio                     |
| **VISION**   | MINIMAL + qbo_vision, qbo_dynamixel, face_follower     |
| **NAVIGATION**| MINIMAL + qbo_navigation, qbo_lidar, qbo_slam         |
| **FULL**     | VISION + NAVIGATION (tous nœuds)                       |

## 🔧 Personnalisation

### Adapter les launch files

Les fichiers `launch/profile_*.launch.py` sont fournis comme templates.
**Vous devez les adapter** selon votre configuration :

- Ports série (`/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.)
- Topics ROS
- Paramètres des nœuds
- Nœuds disponibles dans votre système

### Ajouter un nouveau profil

1. Créer `launch/profile_custom.launch.py`
2. Modifier `PROFILE_LAUNCH_FILES` dans `qbo_bringup_manager.py`
3. Rebuild et tester

## 📝 Modifications nécessaires dans vos fichiers

### action_executor.py (déjà fourni modifié)

Le fichier modifié inclut :
- Import de `ManageProfile`
- Client service `/qbo_bringup/manage_profile`
- Handlers `_handle_start_profile()` et `_handle_stop_node()` implémentés
- Callback `_on_bringup_response()`

### system_mode_manager.py (pas de modification nécessaire)

Le `system_mode_manager` publie déjà les intents `START_PROFILE` avec le bon format JSON.

### behavior_engine.py (pas de modification nécessaire)

Le `behavior_engine` transmet déjà les intents correctement.

## ✅ Checklist d'intégration

- [ ] Service `ManageProfile.srv` ajouté à `qbo_msgs`
- [ ] `qbo_bringup_manager.py` installé et exécutable
- [ ] Launch files copiés et adaptés
- [ ] `action_executor.py` mis à jour
- [ ] `qbo_msgs` rebuild
- [ ] `qbo_bringup` rebuild
- [ ] `qbo_social` rebuild
- [ ] Test manuel : `START MINIMAL` fonctionne
- [ ] Test automatisé : `test_bringup_manager.py` passe
- [ ] Intégration complète : nœuds manquants détectés et démarrés automatiquement

## 🐛 Troubleshooting

### Service non disponible
```bash
ros2 service list | grep manage_profile
# Si absent → vérifier que qbo_bringup_manager est lancé
```

### Launch file non trouvé
```bash
ros2 pkg prefix qbo_bringup
# Vérifier que les launch files sont dans install/qbo_bringup/share/qbo_bringup/launch/
```

### Nœuds ne démarrent pas
- Vérifier les logs : `ros2 run qbo_bringup qbo_bringup_manager --ros-args --log-level debug`
- Vérifier les ports série dans les launch files
- Tester manuellement : `ros2 launch qbo_bringup profile_minimal.launch.py`

## 📚 Références

- **Guide d'intégration** : [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md)
- **ROS 2 Launch API** : https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/
- **ROS 2 Services** : https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/

## 🤝 Contribution

Cette implémentation est conçue pour être :
- **Robuste** : gestion propre des erreurs
- **Évolutive** : facile d'ajouter de nouveaux profils
- **Testable** : suite de tests automatisés incluse
- **Documentée** : guide complet fourni

N'hésitez pas à adapter selon vos besoins !

---

**Auteur** : QBO Project Team
**Date** : Avril 2026
**Version** : 1.0.0
**Licence** : À définir
