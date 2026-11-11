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


def mover_banda(dispositivo, velocidad, puerto_stepper=0):
    """
    Controla la banda transportadora conectada al stepper del Dobot.
    Equivale a SetEMotor() del SDK oficial (DobotDllType), que ya no está
    disponible en el proyecto.
    """
    estado = 1 if velocidad != 0 else 0
    msg = Message()
    msg.id = 135 # ID del comando EMotor
    msg.ctrl = 0x01 # Ejecución inmediata con respuesta (equivale a isQueued=0)
    msg.params = bytearray(struct.pack('<BBi', puerto_stepper, estado, int(velocidad)))
    dispositivo._send_command(msg)


puerto = detectar_puerto_dobot()
if puerto is None:
    print("No se encontró el Dobot (chip CP210x). Conéctalo por USB y verifica el driver.")
    exit()

print(f"Conexión exitosa en {puerto}. Iniciando movimiento simultáneo...")
device = Dobot(port=puerto, verbose=False)

# 1. Configurar velocidad de la banda transportadora
puerto_motor_banda = 0 # 0 = Stepper 1
velocidad_banda = 15000

# 2. Iniciar el movimiento de la banda (arranca de inmediato)
mover_banda(device, velocidad_banda, puerto_motor_banda)

# 3. Iniciar el movimiento del brazo (arranca de inmediato)
# El brazo se moverá hacia la coordenada X=250, Y=0, Z=50 mientras la banda está girando
# move_to sin wait es el equivalente de SetPTPCmd(PTPMOVLXYZMode, ..., isQueued=0)
device.move_to(250, 0, 50, 0)

# Dejar que ambos se muevan simultáneamente durante 4 segundos
print("Banda y brazo moviéndose al tiempo...")
time.sleep(4)

# 4. Detener la banda transportadora
print("Deteniendo sistema...")
mover_banda(device, 0, puerto_motor_banda)

# Volver el brazo a la posición Home
device.move_to(200, 0, 150, 0)
time.sleep(2) # Esperar a que llegue a Home

# Desconectar de forma segura
device.close()
print("Prueba finalizada y Dobot desconectado.")
