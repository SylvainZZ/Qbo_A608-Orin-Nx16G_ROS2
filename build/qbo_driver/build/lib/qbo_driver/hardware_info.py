import psutil
import os
from jtop import jtop


# Fonctions basées sur jtop
def get_jtop_value(key):
    try:
        with jtop() as jetson:
            if jetson.ok():
                stats = jetson.stats
                if key not in stats:
                    print(f"[jtop] Key '{key}' not found. Available keys: {list(stats.keys())}")
                    return None
                return stats[key]
    except Exception as e:
        print(f"[jtop] Exception while accessing '{key}': {e}")
        return None

def get_ram_usage():
    return get_jtop_value('RAM')

def get_cpu_temp():
    return get_jtop_value('Temp cpu')

def get_gpu_usage():
    return get_jtop_value('GPU')

def get_fan_speed():
    return get_jtop_value('Fan pwmfan0')

def get_power_total():
    mw = get_jtop_value('Power TOT')
    return mw / 1000.0 if mw else None

def get_disk_usage():
    try:
        usage = psutil.disk_usage('/')
        return usage.percent  # % d'occupation du disque principal
    except Exception:
        return None

# Dictionnaire de correspondance entre le nom logique et la fonction
SENSOR_FUNCTIONS = {
    "RAM": get_ram_usage,
    "TEMPERATURE CPU": get_cpu_temp,
    "GPU USAGE": get_gpu_usage,
    "FAN SPEED": get_fan_speed,
    "POWER TOTAL": get_power_total,
    "DISK USAGE": get_disk_usage,
}
