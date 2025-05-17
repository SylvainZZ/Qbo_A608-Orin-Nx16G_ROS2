# QBO Talk & Listen - ROS 2 Usage Guide

Ce document récapitule l'utilisation des nœuds qbo_talk et qbo_listen dans ROS 2,
les paramètres configurables, les topics à publier, et les modes de lancement disponibles.

---

## 🔄 Lancement avec ros2 launch

Fichier à utiliser : qbo_audio.launch.py

Exemple de lancement avec valeurs personnalisées :

```bash
ros2 launch qbo_driver qbo_audio.launch.py \
  audio_in:=jabra \
  audio_out:=usb \
  lang:=fr \
  volume:=80 \
  mute_micro:=true
```

Arguments disponibles :

| Nom de l'argument | Description                               | Valeur par défaut     |
|:------            |:---------                                 |:------                |
| audio_in          | Nom du micro (ex. "jabra")                | jabra                 |
| audio_in          | Nom du haut-parleur (ex. "usb")           | usb                   |
| lang              | Langue système initiale (ex. fr, en, es)  | fr                    |
| volume            | Volume de lecture (0-100%)                | 30                    |
| mute_micro        | Coupe le micro pendant la lecture         | true                  |

---

## 🚀 Lancement avec ros2 run

qbo_talk

```bash
ros2 run qbo_driver qbo_talk --ros-args \
  -p audio_out_device_name:=usb \
  -p audio_in_device_name:=jabra \
  -p default_lang:=fr \
  -p mute_micro_during_playback:=true
```

qbo_listen

```bash
ros2 run qbo_driver qbo_listen --ros-args \
  -p audio_in_device_name:=jabra \
  -p default_lang:=fr
```
---

## 📲 Topics ROS utilisés

/system_lang

Type : std_msgs/String

Description : Change dynamiquement la langue des nœuds (TTS, ASR...)

Exemple :

```bash
ros2 topic pub /system_lang std_msgs/String '{data: "en"}'
```

/out_volume

Type : std_msgs/String

Description : Ajuste dynamiquement le volume audio (0-150%)

Exemple :
```bash
ros2 topic pub /out_volume std_msgs/String '{data: "80"}'
```

---

## 🛠️ Paramètres ROS2 internes (qbo_talk)

| Nom                       | Type          | Description     |
|:------                    |:---------     |:------          |
| audio_out_device_name     | string        | Nom du device son (lecture) via sounddevice |
| audio_in_device_name      | string        | Nom du micro via sounddevice et pactl |
| mute_micro_during_playback| bool          | Active la mise en sourdine temporaire du micro pendant la lecture |
| default_lang              | string        | Langue initiale (changée ensuite via topic) |

---

## 🤖 Depuis un autre nœud Python

### 📢 Pour changer la langue :
```bash
from std_msgs.msg import String
publisher = node.create_publisher(String, "/system_lang", 10)
publisher.publish(String(data="fr"))
```
### 🎧 Pour modifier le volume :
```bash
publisher = node.create_publisher(String, "/out_volume", 10)
publisher.publish(String(data="70"))
```

---

## 🔧 Astuce pour initialiser la langue via un script
```bash
ros2 topic pub --once /system_lang std_msgs/String '{data: "fr"}'
```

