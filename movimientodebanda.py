import time
import struct
from pydobot import Dobot
from pydobot.message import Message

def mover_banda(dispositivo, velocidad):
    """
    Controla el motor paso a paso (Stepper 1).
    Velocidad positiva gira hacia un lado, negativa hacia el otro. 0 detiene.
    """
    estado = 2 if velocidad != 0 else 0
    msg = Message()
    msg.id = 135 # ID de comando para el EMotor del Dobot
    puerto_stepper = 0 # 0 = Stepper 1
    
    # IMPORTANTE: Cambiamos '<BBf' a '<BBi' porque el Dobot espera un Entero (int32), no un decimal
    msg.params = bytearray(struct.pack('<BBi', puerto_stepper, estado, int(velocidad)))
    dispositivo._send_command(msg)

# ==========================================
# SECUENCIA PRINCIPAL
# ==========================================
print("1. Conectando al Dobot en COM3...")
device = Dobot(port="COM3", verbose=False)

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