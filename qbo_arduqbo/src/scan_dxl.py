#!/usr/bin/env python3

from dynamixel_sdk import *  # pip install dynamixel-sdk

# === Configuration ===
PORT = "/dev/ttyUSB1"         # Modifie ici si besoin
BAUDRATE = 1000000              # AX-12/18A par défaut : 57600
PROTOCOL_VERSION = 1.0        # AX-18A utilise protocole 1.0
ID_RANGE = range(1, 11)       # ID à scanner (1 à 10)

# === Initialisation ===
portHandler = PortHandler(PORT)
packetHandler = PacketHandler(PROTOCOL_VERSION)

print(f"🔌 Port : {PORT} | Baudrate : {BAUDRATE} | Protocole : {PROTOCOL_VERSION}")

if not portHandler.openPort():
    print("❌ Impossible d'ouvrir le port série.")
    exit(1)
else:
    print("✅ Port ouvert.")

if not portHandler.setBaudRate(BAUDRATE):
    print(f"❌ Impossible de fixer le baudrate à {BAUDRATE}.")
    exit(1)
else:
    print("✅ Baudrate configuré.")

print("🔍 Scan en cours...\n")

for dxl_id in ID_RANGE:
    dxl_model_number, comm_result, error = packetHandler.ping(portHandler, dxl_id)
    if comm_result == COMM_SUCCESS:
        print(f"🟢 Servo trouvé → ID {dxl_id} | Modèle {dxl_model_number}")
    else:
        print(f"🔸 ID {dxl_id} : aucun retour")

portHandler.closePort()
print("\n🔁 Scan terminé.")
