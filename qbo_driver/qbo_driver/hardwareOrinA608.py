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

        # Updater ROS 2
        self.updater = Updater(self)
        self.updater.setHardwareID("orin-nx-16g")

         # Enregistrement des tâches
        self.updater.add("A608 Power", self.diag_power)
        self.updater.add("A608 Temp", self.diag_temp)
        self.updater.add("A608 Fan/RAM/CPU", self.diag_misc)
        self.updater.add("A608 Network", self.diag_net)

        # Timer: une mise à jour par seconde
        self.create_timer(1.0, self.tick)

    # ---------- Callbacks diagnostic_updater ----------
    def diag_power(self, stat: DiagnosticStatusWrapper):
        s = self.jetson.stats
        cpuGpu = self._mw_to_w(s.get(JTOP_MAP["PowerInstCPUGPU"]))
        soc = self._mw_to_w(s.get(JTOP_MAP["PowerInstSOC"]))
        total = self._mw_to_w(s.get(JTOP_MAP["PowerInstVDD_IN"]))
        stat.add("CPU W", f"{cpuGpu:.1f}" if cpuGpu is not None else "n/a")
        stat.add("SOC W", f"{soc:.1f}" if soc is not None else "n/a")
        stat.add("VDD_IN W", f"{total:.1f}" if total is not None else "n/a")
        stat.summary(DiagnosticStatus.OK, "OK")
        return stat

    def diag_temp(self, stat):
        s = self.jetson.stats
        cpu = s.get(JTOP_MAP["TempCPU"])
        gpu = s.get(JTOP_MAP["TempGPU"])
        stat.add("CPU °C", f"{cpu:.1f}" if cpu is not None else "n/a")
        stat.add("GPU °C", f"{gpu:.1f}" if gpu is not None else "n/a")
        stat.summary(DiagnosticStatus.OK, "OK")
        return stat

    def diag_misc(self, stat):
        s = self.jetson.stats
        fan = s.get(JTOP_MAP["Fan"])
        stat.add("FAN %", str(int(fan)) if fan is not None else "n/a")

        ram = psutil.virtual_memory().percent
        stat.add("RAM %", f"{ram:.1f}")

        cpu = psutil.cpu_percent(interval=0.1)
        stat.add("CPU %", f"{cpu:.1f}")

        gpu = s.get("GPU")
        stat.add("GPU %", f"{gpu:.1f}" if isinstance(gpu, (int, float)) else "n/a")

        stat.summary(DiagnosticStatus.OK, "OK")
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
        # Hostname + IP
        hostname = socket.gethostname()
        stat.add("Hostname", hostname)
        ip = self._get_ip_address()
        stat.add("IP Address", ip or "n/a")

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
