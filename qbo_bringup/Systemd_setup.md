# ✅ Qbo ROS 2 Bringup — systemd & timer setup

This guide explains how to automatically launch the QBO system using `systemd` with a post-boot delay via a `.timer`.

**Two strategies are available:**
1. **Social-only start** (recommended) — Let system_mode_manager auto-start MINIMAL profile
2. **Full start** — Launch both social and MINIMAL profile at boot

---

## 🎯 Strategy 1: Social-only (RECOMMENDED)

Launch only the social architecture + bringup_manager, then let the system auto-detect and start MINIMAL.

### ✅ Goals

- 🔁 Automatically start social architecture 20 seconds after boot
- 🧠 Let system_mode_manager detect missing nodes and start MINIMAL profile
- 👤 Run as non-root user (`qbo-v2`)
- 📋 Log output with `journalctl`
- 🔄 Auto-restart on failure
- 🕓 Use a systemd `.timer` for robust post-boot triggering

---

## 🧾 1. Create the service file (Strategy 1)

```bash
sudo nano /etc/systemd/system/qbo_bringup.service
```

### Contents:

```ini
[Unit]
Description=Qbo ROS 2 Social Architecture + Bringup Manager
After=network-online.target jtop.service
Requires=network-online.target jtop.service

StartLimitIntervalSec=0

[Service]
Type=simple
User=qbo-v2
WorkingDirectory=/home/qbo-v2/qbo_ws/
Environment="HOME=/home/qbo-v2"
Environment="DISPLAY=:0.0"

# 🧠 Lancement ROS2 — Social + Bringup Manager only
# system_mode_manager will auto-start MINIMAL profile when needed
ExecStart=/bin/bash -c "echo 'STARTING QBO SOCIAL ARCHITECTURE' >> /tmp/qbo_bringup_debug.log && date >> /tmp/qbo_bringup_debug.log && source /opt/ros/humble/setup.bash && source /home/qbo-v2/qbo_ws/install/setup.bash && ros2 launch qbo_bringup qbo_system.launch.py"

Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=none
```

---

## 🎯 Strategy 2: Full start (Alternative)

Launch both social architecture AND MINIMAL profile at boot.

## 🧾 1. Create the service file (Strategy 2)

```bash
sudo nano /etc/systemd/system/qbo_bringup.service
```

### Contents:

```ini
[Unit]
Description=Qbo ROS 2 Full System (Social + MINIMAL Profile)
After=network-online.target jtop.service
Requires=network-online.target jtop.service

StartLimitIntervalSec=0

[Service]
Type=simple
User=qbo-v2
WorkingDirectory=/home/qbo-v2/qbo_ws/
Environment="HOME=/home/qbo-v2"
Environment="DISPLAY=:0.0"

# 🧠 Lancement ROS2 — Social + Auto-start MINIMAL profile
ExecStart=/bin/bash -c "echo 'STARTING QBO FULL SYSTEM' >> /tmp/qbo_bringup_debug.log && date >> /tmp/qbo_bringup_debug.log && source /opt/ros/humble/setup.bash && source /home/qbo-v2/qbo_ws/install/setup.bash && ros2 launch qbo_bringup qbo_system.launch.py auto_start_profile:=MINIMAL"

Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=none
```

---

## 📊 Comparison: Which strategy to choose?

| Aspect | Strategy 1 (Social-only) | Strategy 2 (Full) |
|--------|-------------------------|-------------------|
| **Boot complexity** | Simple | Simple |
| **Auto-recovery** | ✅ Yes (system_mode_manager) | ⚠️ Manual restart needed |
| **Flexibility** | ✅ Profiles started on-demand | Fixed MINIMAL at boot |
| **Services count** | 1 systemd service | 1 systemd service |
| **Architecture** | ✅ Uses full social intelligence | Bypasses auto-detection |
| **Recommended for** | Production use | Development/testing |

**Recommendation: Use Strategy 1** — It leverages your intelligent architecture where `system_mode_manager` detects missing nodes and starts profiles automatically.

---

## ⏲️ 2. Create the timer file (same for both strategies)

```bash
sudo nano /etc/systemd/system/qbo_bringup.timer
```

### Contents:

```ini
[Unit]
Description=Start Qbo Bringup after boot delay

[Timer]
OnBootSec=20
Unit=qbo_bringup.service

[Install]
WantedBy=timers.target
```

---

## ⚙️ 3. Enable and start the timer

```bash
sudo systemctl daemon-reload
sudo systemctl enable qbo_bringup.timer
sudo systemctl start qbo_bringup.timer
```

---

## 🧠 How auto-start works (Strategy 1)

When using Strategy 1, the boot sequence is:

1. **Systemd timer** triggers after 20s
2. **qbo_system.launch.py** starts:
   - qbo_bringup_manager (ready to launch profiles, publishes diagnostics every 2s)
   - event_adapter, world_model, behavior_engine, action_executor
   - system_mode_manager (monitors /diagnostics, publishes status every 15s)
3. **system_mode_manager** checks for active profiles (watchdog every 5s)
4. **Detects missing profile** → Publishes `BehaviorIntent(START_PROFILE)` on `/qbo_social/intent`
5. **action_executor** → Receives intent and calls `/qbo_bringup/manage_profile` service
6. **qbo_bringup_manager** → Launches `profile_minimal.launch.py` via subprocess
7. **qbo_bringup_manager** → Publishes profile status on `/diagnostics`
8. **system_mode_manager** → Synchronizes active profile from diagnostics
9. **System is operational** with MINIMAL profile running

### Timing details:
- **qbo_bringup_manager**: Publishes diagnostics every 2s
- **system_mode_manager**: Watchdog checks profile every 5s, publishes status every 15s
- **Maximum detection delay**: ~5s (watchdog frequency)
- **Profile startup time**: ~2-5s depending on hardware

### Watch it happen:

```bash
# Terminal 1: Monitor system events
ros2 topic echo /qbo_social/events

# Terminal 2: Monitor intents
ros2 topic echo /qbo_social/intent

# Terminal 3: Monitor bringup diagnostics
ros2 topic echo /diagnostics | grep "qbo_bringup"

# Terminal 4: Monitor systemd logs
journalctl -fu qbo_bringup.service
```

**Result**: Your robot boots intelligently, starting only what's needed!

---

## 🔍 4. Monitoring

### Timer status:

```bash
systemctl list-timers | grep qbo
systemctl status qbo_bringup.timer
```

### Service status and logs:

```bash
systemctl status qbo_bringup.service
journalctl -u qbo_bringup.service
journalctl -fu qbo_bringup.service
```

### Debug log output:

```bash
cat /tmp/qbo_bringup_debug.log
```

---

## 🧪 5. Post-reboot check

After reboot, wait ~20s then run:

```bash
journalctl -u qbo_bringup.service
```

Or:

```bash
cat /tmp/qbo_bringup_debug.log
```

---

# 🔧 Systemd Commands

## ⏸️ Disable auto-start (keep service but don't start at boot)

```bash
# Disable timer (no auto-start at boot)
sudo systemctl disable qbo_bringup.timer

# Stop timer if currently running
sudo systemctl stop qbo_bringup.timer

# Verify it's disabled
systemctl is-enabled qbo_bringup.timer
# Should return: disabled
```

## ✅ Re-enable auto-start

```bash
# Enable timer (auto-start at boot)
sudo systemctl enable qbo_bringup.timer

# Start timer immediately
sudo systemctl start qbo_bringup.timer
```

## Service

```bash
sudo systemctl start qbo_bringup.service
sudo systemctl stop qbo_bringup.service
sudo systemctl restart qbo_bringup.service
sudo systemctl status qbo_bringup.service
```

## Timer

```bash
sudo systemctl start qbo_bringup.timer
sudo systemctl enable qbo_bringup.timer
sudo systemctl stop qbo_bringup.timer
sudo systemctl disable qbo_bringup.timer
sudo systemctl status qbo_bringup.timer
```

---

## 🛠 When modifying the service or timer files

```bash
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl restart qbo_bringup.timer
```
