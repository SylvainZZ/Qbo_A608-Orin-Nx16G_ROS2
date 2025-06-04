# Configuration des règles UDEV pour Qbo

## 🔄 Introduction
Dans un projet robotique comme **Qbo**, il est crucial que les périphériques connectés en USB (Arduino, Lidar, interfaces série) aient des noms stables dans `/dev/`. Par défaut, Linux assigne des noms dynamiques tels que `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc., qui peuvent changer à chaque redémarrage.

Pour éviter ce comportement, on utilise des **règles UDEV** permettant de créer des liens symboliques stables basés sur l'identité ou le chemin physique du périphérique.

---

## ✅ Objectifs
- Assigner un nom stable à chaque périphérique USB utilisé par Qbo.
- Garantir un fonctionnement prévisible au démarrage.

---

## ✍️ Étapes de configuration

### 1. Créer le fichier de règles

```bash
sudo nano /etc/udev/rules.d/99-usb-serial.rules
```

### 2. Coller les règles suivantes (adaptées à ton nouveau PC)

```udev
# ttyQboard2 (USB0) par chemin physique
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", KERNELS=="1-2.2:1.0", SYMLINK+="ttyQboard2"

# ttyDmx (USB1) par chemin physique
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", KERNELS=="1-2.2:1.1", SYMLINK+="ttyDmx"

# ttyQboard1 (USB2) identifié par idProduct
SUBSYSTEM=="tty", ATTRS{idProduct}=="6001", SYMLINK+="ttyQboard1"

# ttyLidar (optionnel) identifié par idProduct
SUBSYSTEM=="tty", ATTRS{idProduct}=="ea60", SYMLINK+="ttyLidar"
```

---

## 🔄 Appliquer les changements

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Vérifier les liens symboliques :
```bash
ls -l /dev/ttyQboard*
ls -l /dev/ttyDmx
```

---

## 🔧 Astuces de débogage

- Test d'une règle :
  ```bash
  udevadm test /sys/class/tty/ttyUSB0
  ```

- Logs UDEV :
  ```bash
  journalctl -xe | grep tty
  ```

- Identifier le chemin physique :
  ```bash
  ls -l /dev/serial/by-path/
  ```

- Identifier les KERNELS :
  ```bash
  udevadm info -a -n /dev/ttyUSB0 | grep KERNELS
  ```

---

## 📄 Conclusion
Avec ces règles en place, les composants du robot Qbo auront des noms stables, permettant un démarrage fiable et reproductible à chaque redémarrage de la machine.
