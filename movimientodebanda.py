import time
import struct
from serial.tools import list_ports
from pydobot import Dobot
from pydobot.message import Message

def detectar_puerto_dobot():
    """
    Busca automáticamente el puerto USB del Dobot
    (el brazo usa el chip Silicon Labs CP210x USB to UART).
    """
    for puerto in list_ports.comports():
        descripcion = (puerto.description or '').upper()
        hwid = puerto.hwid or ''
        if 'CP210' in descripcion or '10C4:EA60' in hwid:
            return puerto.device
    return None

def mover_banda(dispositivo, velocidad):
    """
    Controla el motor paso a paso (Stepper 1).
    Equivale a SetEMotor() del SDK oficial (DobotDllType).
    Velocidad positiva gira hacia un lado, negativa hacia el otro. 0 detiene.
    """
    estado = 1 if velocidad != 0 else 0
    msg = Message()
    msg.id = 135 # ID de comando para el EMotor del Dobot
    msg.ctrl = 0x01 # Ejecución inmediata con respuesta (equivale a isQueued=0)
    puerto_stepper = 0 # 0 = Stepper 1

    # El Dobot espera: index (uint8), isEnabled (uint8), velocidad (int32)
    msg.params = bytearray(struct.pack('<BBi', puerto_stepper, estado, int(velocidad)))
    dispositivo._send_command(msg)

# ==========================================
# SECUENCIA PRINCIPAL
# ==========================================
puerto = detectar_puerto_dobot()
if puerto is None:
    print("No se encontró el Dobot (chip CP210x). Conéctalo por USB y verifica el driver.")
    exit()

print(f"1. Conectando al Dobot en {puerto}...")
device = Dobot(port=puerto, verbose=False)

print("2. Encendiendo la banda...")
# 5000 pulsos/segundo. Si gira al revés, usa -5000
mover_banda(device, 10000)

# Reemplazamos el time.sleep(7) por un conteo visible en pantalla 
# para asegurarnos de que el programa sí está pausando
for i in range(1, 8):
    print(f"   -> Banda girando... {i} segundos")
    time.sleep(1)

print("3. Deteniendo la banda...")
mover_banda(device, 0)

device.close()
print("4. Prueba finalizada y puerto cerrado.")