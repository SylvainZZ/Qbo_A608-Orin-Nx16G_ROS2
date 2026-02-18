# ==========================
# SEUILS SYSTEME
# ==========================

BATTERY_LOW_VOLTAGE = 11.5
BATTERY_CRITICAL_VOLTAGE = 10.8

CPU_TEMP_WARNING = 75.0
CPU_TEMP_CRITICAL = 85.0

GPU_TEMP_WARNING = 70.0

# ==========================
# SIMILARITÉ FAISS
# ==========================

FAISS_CONFIDENCE_THRESHOLD = 0.55

# ==========================
# COULEURS NEZ
# ==========================

COLOR_MAP = {
    0: "éteint",
    1: "rouge",
    2: "bleu",
    3: "violet",
    4: "vert",
    5: "jaune",
    6: "magenta",
    7: "blanc"
}

COLOR_KEYWORDS = {
    "rouge": 1,
    "bleu": 2,
    "violet": 3,
    "vert": 4,
    "jaune": 5,
    "magenta": 6,
    "blanc": 7
}

# ==========================
# PHRASES FALLBACK
# ==========================

FALLBACK_PHRASES = [
    "Je ne suis pas sûr de comprendre.",
    "Peux-tu reformuler ?",
    "Je n'ai pas encore appris cela.",
    "Je ne sais pas répondre à cette question."
]

# ==========================
# EVENEMENTS AUTOMATIQUES
# ==========================

EVENT_BATTERY_LOW = "Attention, ma batterie est faible."
EVENT_BATTERY_CHARGING = "Je suis maintenant en charge."
EVENT_CPU_HOT = "Ma température CPU est élevée."
EVENT_IMU_NOT_CALIBRATED = "Mon IMU n'est pas calibrée."
