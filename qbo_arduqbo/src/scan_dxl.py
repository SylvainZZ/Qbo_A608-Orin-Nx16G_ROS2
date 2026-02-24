#!/usr/bin/env python3

from dynamixel_sdk import *  # pip install dynamixel-sdk

# === Configuration ===
PORT = "/dev/ttyUSB1"         # Modifie ici si besoin
BAUDRATE = 1000000            # AX-12/18A par défaut : 57600
PROTOCOL_VERSION = 1.0        # AX-18A utilise protocole 1.0
ID_RANGE = range(1, 11)       # ID à scanner (1 à 10)

# === Initialisation ===
portHandler = PortHandler(PORT)
packetHandler = PacketHandler(PROTOCOL_VERSION)

print(f"🔌 Port: {PORT} | Baud rate: {BAUDRATE} | Protocol: {PROTOCOL_VERSION}")

if not portHandler.openPort():
    print("❌ Unable to open the serial port.")
    exit(1)
else:
    print("✅ Port opened.")

if not portHandler.setBaudRate(BAUDRATE):
    print(f"❌ Unable to set baud rate to {BAUDRATE}.")
    exit(1)
else:
    print("✅ Baud rate configured.")

print("🔍 Scanning in progress...\n")

for dxl_id in ID_RANGE:
    dxl_model_number, comm_result, error = packetHandler.ping(portHandler, dxl_id)
    if comm_result == COMM_SUCCESS:
        print(f"🟢 Servo found → ID {dxl_id} | Model {dxl_model_number}")
    else:
        print(f"🔸 ID {dxl_id}: no response")

portHandler.closePort()
print("\n🔁 Scan completed.")
