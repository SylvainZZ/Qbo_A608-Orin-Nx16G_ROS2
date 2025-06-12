"""hardwareOrinA608.py
Diagnostic publisher for the Seeed A608 carrier board + Jetson Orin NX 16 GB.
Publie les métriques récupérées en une seule passe avec **jtop**.
"""

import sys
import socket
import psutil
import rclpy
from rclpy.node import Node
from diagnostic_updater import Updater, DiagnosticStatusWrapper
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from jtop import jtop
from collections import deque
from time import time

JTOP_MAP = {
    "PowerInstCPUGPU": "Power VDD_CPU_GPU_CV",  # mW
    "PowerInstSOC": "Power VDD_SOC",            # mW
    "PowerInstVDD_IN": "Power TOT",   # mW total
    "TempCPU": "Temp cpu",            # °C
    "TempGPU": "Temp gpu",            # °C
    "RAM": "RAM",                     # % from jtop
    "Fan": "Fan pwmfan0"              # %
}

class OrinA608Diagnostics(Node):
    def __init__(self):
        super().__init__("orin_a608_diag")

        # Initialise jtop en mode context manager permanent
        self.jetson = jtop(interval=1.0)
        self.jetson.start()

        # Affiche les clés/valeurs de jtop disponibles
        # print("\n[jtop] Données disponibles dans jetson.stats :")
        # for key, value in self.jetson.stats.items():
        #     print(f"  {key}: {value}")
        # print("--- Fin des clés jtop ---\n")

        # Historique pour lissage
        self.history_window = 60  # secondes
        self.cpu_history = deque()
        self.ram_history = deque()
        self.gpu_history = deque()

        # Puissances : seuils paramétrables
        self.declare_parameter("power_warning_threshold", 40.0)  # W
        self.declare_parameter("power_error_threshold", 60.0)  # W
        self.warning_power_limit = self.get_parameter("power_warning_threshold").get_parameter_value().double_value
        self.error_power_limit = self.get_parameter("power_error_threshold").get_parameter_value().double_value
        # Température : seuils paramétrables
        self.declare_parameter("temp_warning_threshold", 60.0)  # °C
        self.declare_parameter("temp_error_threshold", 70.0)    # °C
        self.temp_warning_limit = self.get_parameter("temp_warning_threshold").get_parameter_value().double_value
        self.temp_error_limit = self.get_parameter("temp_error_threshold").get_parameter_value().double_value
        # Utilisation (Fan,RAM,GPU,CPU) : Seuils paramétrables
        self.declare_parameter("fan_min_rpm_percent", 20.0)  # % de PWM
        self.fan_min_threshold = self.get_parameter("fan_min_rpm_percent").get_parameter_value().double_value

        self.declare_parameter("ram_warning_threshold", 80.0)
        self.declare_parameter("ram_error_threshold", 95.0)
        self.ram_warn = self.get_parameter("ram_warning_threshold").get_parameter_value().double_value
        self.ram_err = self.get_parameter("ram_error_threshold").get_parameter_value().double_value

        self.declare_parameter("cpu_warning_threshold", 80.0)
        self.declare_parameter("cpu_error_threshold", 95.0)
        self.cpu_warn = self.get_parameter("cpu_warning_threshold").get_parameter_value().double_value
        self.cpu_err = self.get_parameter("cpu_error_threshold").get_parameter_value().double_value

        self.declare_parameter("gpu_warning_threshold", 80.0)
        self.declare_parameter("gpu_error_threshold", 95.0)
        self.gpu_warn = self.get_parameter("gpu_warning_threshold").get_parameter_value().double_value
        self.gpu_err = self.get_parameter("gpu_error_threshold").get_parameter_value().double_value

        # Updater ROS 2
        self.updater = Updater(self)
        self.updater.setHardwareID("orin-nx-16g")

         # Enregistrement des tâches
        self.updater.add("A608 Power", self.diag_power)
        self.updater.add("A608 Temp", self.diag_temp)
        self.updater.add("A608 Fan/RAM/CPU", self.diag_misc)
        self.updater.add("A608 Network", self.diag_net)

        # Timer: une mise à jour par seconde
        self.create_timer(2.0, self.tick)

    # ---------- Callbacks diagnostic_updater ----------
    def diag_power(self, stat: DiagnosticStatusWrapper):
        s = self.jetson.stats
        cpuGpu = self._mw_to_w(s.get(JTOP_MAP["PowerInstCPUGPU"]))
        soc = self._mw_to_w(s.get(JTOP_MAP["PowerInstSOC"]))
        total = self._mw_to_w(s.get(JTOP_MAP["PowerInstVDD_IN"]))
        stat.add("CPU W", f"{cpuGpu:.1f}" if cpuGpu is not None else "n/a")
        stat.add("SOC W", f"{soc:.1f}" if soc is not None else "n/a")
        stat.add("VDD_IN W", f"{total:.1f}" if total is not None else "n/a")

        # État du diagnostic basé uniquement sur le total
        if total is None:
            stat.summary(DiagnosticStatus.ERROR, "Power data unavailable")
        elif total >= self.error_power_limit:
            stat.summary(DiagnosticStatus.ERROR, f"Power overload: {total:.1f} W")
        elif total >= self.warning_power_limit:
            stat.summary(DiagnosticStatus.WARN, f"High power: {total:.1f} W")
        else:
            stat.summary(DiagnosticStatus.OK, "OK")

        return stat

    def diag_temp(self, stat):
        s = self.jetson.stats
        cpu = s.get(JTOP_MAP["TempCPU"])
        gpu = s.get(JTOP_MAP["TempGPU"])
        stat.add("CPU °C", f"{cpu:.1f}" if cpu is not None else "n/a")
        stat.add("GPU °C", f"{gpu:.1f}" if gpu is not None else "n/a")

        # Déterminer le niveau de diagnostic

        # Cas où une valeur est manquante
        if cpu is None or gpu is None:
            stat.summary(DiagnosticStatus.ERROR, "Temperature data unavailable")
            return stat

        # Analyse détaillée
        errors = []
        warnings = []

        if cpu >= self.temp_error_limit:
            errors.append(f"CPU {cpu:.1f}°C")
        elif cpu >= self.temp_warning_limit:
            warnings.append(f"CPU {cpu:.1f}°C")

        if gpu >= self.temp_error_limit:
            errors.append(f"GPU {gpu:.1f}°C")
        elif gpu >= self.temp_warning_limit:
            warnings.append(f"GPU {gpu:.1f}°C")

        if errors:
            stat.summary(DiagnosticStatus.ERROR, "Overheat: " + ", ".join(errors))
        elif warnings:
            stat.summary(DiagnosticStatus.WARN, "High temp: " + ", ".join(warnings))
        else:
            stat.summary(DiagnosticStatus.OK, "OK")

        return stat

    def diag_misc(self, stat):
        s = self.jetson.stats
        level = DiagnosticStatus.OK
        messages = []

        # FAN
        fan = s.get(JTOP_MAP["Fan"])
        if fan is None:
            level = DiagnosticStatus.ERROR
            messages.append("Fan speed unavailable")
            stat.add("FAN %", "n/a")
        elif fan <= 0.0:
            level = DiagnosticStatus.ERROR
            messages.append("Fan not running!")
            stat.add("FAN %", "0")
        elif fan < self.fan_min_threshold:
            if level < DiagnosticStatus.WARN:
                level = DiagnosticStatus.WARN
            messages.append(f"Fan speed low ({fan:.0f}%)")
            stat.add("FAN %", str(int(fan)))
        else:
            stat.add("FAN %", str(int(fan)))

        # RAM
        ram = psutil.virtual_memory().percent
        self._update_history(self.ram_history, ram)
        avg_ram = sum(v for _, v in self.ram_history) / len(self.ram_history)
        stat.add("RAM %", f"{ram:.1f}")
        if avg_ram >= self.ram_err:
            level = DiagnosticStatus.ERROR
            messages.append(f"RAM avg {avg_ram:.1f}% (error)")
        elif avg_ram >= self.ram_warn:
            level = max(level, DiagnosticStatus.WARN)
            messages.append(f"RAM avg {avg_ram:.1f}% (warn)")

        # CPU
        cpu = psutil.cpu_percent(interval=0.1)
        self._update_history(self.cpu_history, cpu)
        avg_cpu = sum(v for _, v in self.cpu_history) / len(self.cpu_history)
        stat.add("CPU %", f"{cpu:.1f}")
        if avg_cpu >= self.cpu_err:
            level = DiagnosticStatus.ERROR
            messages.append(f"CPU avg {avg_cpu:.1f}% (error)")
        elif avg_cpu >= self.cpu_warn:
            level = max(level, DiagnosticStatus.WARN)
            messages.append(f"CPU avg {avg_cpu:.1f}% (warn)")

        # GPU
        gpu = s.get("GPU")
        if isinstance(gpu, (int, float)):
            self._update_history(self.gpu_history, gpu)
            avg_gpu = sum(v for _, v in self.gpu_history) / len(self.gpu_history)
            stat.add("GPU %", f"{gpu:.1f}")
            if avg_gpu >= self.gpu_err:
                level = DiagnosticStatus.ERROR
                messages.append(f"GPU avg {avg_gpu:.1f}% (error)")
            elif avg_gpu >= self.gpu_warn:
                level = max(level, DiagnosticStatus.WARN)
                messages.append(f"GPU avg {avg_gpu:.1f}% (warn)")
        else:
            stat.add("GPU %", "n/a")

        stat.summary(level, "; ".join(messages) if messages else "OK")
        return stat

    def diag_net(self, stat):
        # iface = "enP8p1s0"
        # nstats = psutil.net_if_stats().get(iface)
        # io = psutil.net_io_counters(pernic=True).get(iface)
        # if not nstats:
        #     stat.summary(DiagnosticStatus.WARN, f"{iface} not found")
        #     return stat
        # stat.add("Up", str(nstats.isup))
        # stat.add("Speed(Mb)", str(nstats.speed))
        # stat.add("RX bytes", str(io.bytes_recv if io else 0))
        # stat.add("TX bytes", str(io.bytes_sent if io else 0))
        # Hostname
        hostname = socket.gethostname()
        stat.add("Hostname", hostname)

        # IP Address (via socket)
        ip = self._get_ip_address()
        stat.add("IP Address", ip if ip else "n/a")

        # Diagnostic logic
        if not ip:
            stat.summary(DiagnosticStatus.WARN, "No network or no internet access")
        else:
            stat.summary(DiagnosticStatus.OK, "OK")
        return stat

    # ---------- helpers ----------
    def _mw_to_w(self, mw):
        return None if mw is None else float(mw)/1000.0

    def _get_ip_address(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return None

    def _update_history(self, history, value):
        now = time()
        history.append((now, value))
        # Retire les anciennes entrées
        while history and now - history[0][0] > self.history_window:
            history.popleft()

    def tick(self):
        # Force l'updater à publier un DiagnosticArray
        self.updater.force_update()

    def destroy_node(self):
        self.jetson.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = OrinA608Diagnostics()
    node.get_logger().info("Diagnostics node Orin and A608 start")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested (Ctrl+C)")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
