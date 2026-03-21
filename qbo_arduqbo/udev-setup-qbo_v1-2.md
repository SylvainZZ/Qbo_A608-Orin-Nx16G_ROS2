# Configuration des règles UDEV pour Qbo
Version: 1.2
Date: 2026-03-21
---

## 🔄 Introduction

Dans un projet robotique comme **Qbo**, il est crucial que les périphériques USB (Arduino, Lidar, caméras, interfaces série…) aient des noms **stables** dans `/dev/`.

Par défaut, Linux assigne des noms dynamiques :

```
/dev/ttyUSB0
/dev/ttyUSB1
/dev/video0
/dev/video1
```

⚠️ Ces noms peuvent changer à chaque redémarrage ou rebranchement USB.

👉 Solution : utiliser des **règles UDEV** pour créer des **liens symboliques stables**.

---

## 🎯 Objectifs

* Assigner un nom stable à chaque périphérique
* Garantir un fonctionnement reproductible
* Éviter les bugs aléatoires au démarrage
* Améliorer la robustesse ROS2

---

# 🧠 Principe des règles UDEV

UDEV permet de matcher un périphérique via :

* idVendor / idProduct
* numéro de série (serial)
* chemin physique (KERNELS)
* type de périphérique (SUBSYSTEM)

Puis d’appliquer :

* un nom stable (`SYMLINK`)
* des droits (`MODE`, `GROUP`)
* des actions (`RUN+=`)

---

# ⚡ Optimisation : `latency_timer` (IMPORTANT)

## 🧠 Principe

Les interfaces USB → série (FTDI, CP210x…) utilisent un **buffer interne**.

Par défaut :

* les données sont envoyées toutes les **16 ms**

👉 Cela introduit une latence inutile.

---

## 🚀 Solution

On force :

```
latency_timer = 1 ms
```

➡️ Résultat :

* latence divisée par ~10
* meilleure réactivité capteurs
* navigation ROS2 plus fluide

---

## ⚠️ Impact

| Avantage             | Inconvénient                  |
| -------------------- | ----------------------------- |
| Latence réduite      | CPU légèrement plus sollicité |
| Meilleure réactivité | Plus d’interruptions          |

👉 Sur Orin NX → aucun problème

---

## ✅ Compatibilité

| Driver   | Support |
| -------- | ------- |
| ftdi_sio | ✅       |
| cp210x   | ⚠️      |
| ch341    | ❌       |

---

## 🔍 Vérifier le support

```bash
ls /sys/bus/usb-serial/devices/
```

Puis :

```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

Ou en automatique :

```bash
for d in /sys/bus/usb-serial/devices/*; do
  echo -n "$d: "
  cat $d/latency_timer 2>/dev/null || echo "NOT SUPPORTED"
done
```

---

# ✍️ Étapes de configuration

## 1. Identifier les périphériques

```bash
lsusb
```

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E 'ATTRS{idVendor}|ATTRS{idProduct}|ATTRS{serial}'
```

---

## 2. Créer le fichier de règles

```bash
sudo adduser $USER dialout
sudo nano /etc/udev/rules.d/99-usb-serial.rules
```

---

## 3. Règles UDEV complètes

```udev
# ================================
# PORTS SÉRIE (avec low latency)
# ================================

# ttyQboard2 (USB0) par chemin physique
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", KERNELS=="1-2.2:1.0", \
  SYMLINK+="ttyQboard2", MODE="0666", GROUP="dialout", \
  RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"

# ttyDmx (USB1) par chemin physique
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", KERNELS=="1-2.2:1.1", \
  SYMLINK+="ttyDmx", MODE="0666", GROUP="dialout", \
  RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"

# ttyQboard1 identifié par idProduct
SUBSYSTEM=="tty", ATTRS{idProduct}=="6001", \
  SYMLINK+="ttyQboard1", MODE="0666", GROUP="dialout", \
  RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"

# ttyLidar
SUBSYSTEM=="tty", ATTRS{idProduct}=="ea60", \
  SYMLINK+="ttyLidar", MODE="0666", GROUP="dialout", \
  RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"


# ================================
# CAMÉRA USB (WEBCAM)
# ================================

# Sélection du flux principal (index 0)
SUBSYSTEM=="video4linux", \
  ATTRS{idVendor}=="32e4", ATTRS{idProduct}=="0577", \
  ATTRS{serial}=="01.00.00", \
  ATTR{index}=="0", \
  SYMLINK+="cam_usb"
```

---

# 🎥 Gestion des caméras (IMPORTANT)

## 🔍 Pourquoi plusieurs `/dev/videoX` ?

Une caméra UVC expose plusieurs flux :

* index 0 → flux principal ✅
* index 1 → flux secondaire (souvent inutile)

---

## 🔎 Vérifier les flux

```bash
v4l2-ctl --list-devices
```

```bash
ffplay /dev/video6
ffplay /dev/video7
```

---

## 🎯 Bonne pratique

Toujours filtrer avec :

```
ATTR{index}=="0"
```

---

## ⚠️ Cas RealSense

* expose plusieurs `/dev/videoX`
* mapping complexe

👉 NE PAS faire de règle UDEV

✔️ utiliser le driver ROS2 `realsense2_camera`

---

## 📌 Alternative simple

Linux fournit déjà des liens stables :

```bash
ls -l /dev/v4l/by-id/
ls -l /dev/v4l/by-path/
```

---

# 🔄 Appliquer les règles

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

# ✅ Vérifications

## Ports série

```bash
ls -l /dev/ttyQ*
ls -l /dev/ttyDmx
```

## Caméra

```bash
ls -l /dev/cam_usb
```

---

## Vérifier latency

```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

👉 doit afficher :

```
1
```

---

# 🔧 Debug avancé

## Tester une règle

```bash
udevadm test /sys/class/tty/ttyUSB0
```

---

## Infos complètes device

```bash
udevadm info -q all -n /dev/ttyUSB0
```

---

## Voir les symlinks

```bash
ls -l /dev/tty*
ls -l /dev/cam*
```

---

## Identifier chemin physique

```bash
ls -l /dev/serial/by-path/
```

---

## Identifier driver

```bash
lsusb -t
```

---

## Logs UDEV

```bash
journalctl -xe | grep tty
```

---

# 🚀 Intégration ROS2

Dans tes nodes :

```yaml
port: /dev/ttyLidar
video_device: /dev/cam_usb
```

👉 garantit un système stable au reboot

---

# 🧠 Bonnes pratiques (robotique)

✔️ Toujours utiliser des symlinks
✔️ Préférer `serial` si disponible
✔️ Utiliser `index==0` pour caméra
✔️ Activer `latency_timer` sur capteurs critiques
✔️ Éviter les noms dynamiques

---

# 📄 Conclusion

Avec cette configuration :

* périphériques stables
* latence optimisée
* comportement déterministe
* système prêt pour production robotique

👉 C’est une base solide pour un robot fiable et maintenable.
