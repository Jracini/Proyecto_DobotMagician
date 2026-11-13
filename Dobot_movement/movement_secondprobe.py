import time
import traceback
from serial.tools import list_ports
from pydobot import Dobot


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


# ==================================================
# CONFIGURACIÓN
# ==================================================
# Ejes: +X adelante, -Y DERECHA, +Y izquierda, +Z arriba.
# Los puntos están a 210 mm de la base (centro del área de trabajo).
# Si el brazo gira hacia el lado contrario, cambia el signo de Y.

Z_SEGURA = 30.0     # Altura de desplazamiento (más baja = recorrido más corto)
Z_GRAB = -20.0      # Altura de agarre (calibrar con el objeto real)
Z_DROP = -15.0      # Altura para soltar el objeto
VELOCIDAD = 200     # Velocidad del brazo (si salta alarma o se ve brusco, bájala)
ACELERACION = 200   # Aceleración del brazo
PAUSA_GARRA = 0.5   # Segundos de espera al abrir/cerrar la garra
R_ANGLE = 0.0
USAR_GARRA = True   # False para probar solo el movimiento

# Posición de seguridad (frente al robot, ángulo 0°)
SEGURA = (210.0, 0.0)
# Punto de agarre (ángulo -45°, hacia la derecha)
AGARRE = (148.5, -148.5)


# ==================================================
# FUNCIONES
# ==================================================
def ir_a(device, punto, z):
    """Mueve el brazo a (x, y) a la altura z y espera a que termine."""
    device.move_to(x=punto[0], y=punto[1], z=z, r=R_ANGLE, wait=True)


def bajar(device, punto, z_inicial, z_final):
    """Baja directamente a z_final en un solo movimiento."""
    ir_a(device, punto, z_final)


# ==================================================
# PROGRAMA
# ==================================================
puerto = detectar_puerto_dobot()
if puerto is None:
    print("No se encontró el Dobot (chip CP210x). Conéctalo por USB y verifica el driver.")
    exit()

device = None
try:
    device = Dobot(port=puerto, verbose=False)
    print(f"Conectado exitosamente al Dobot en {puerto}")
    time.sleep(2)

    device.speed(VELOCIDAD, ACELERACION)

    # 1. Posición de seguridad
    print("1. Posición de seguridad...")
    ir_a(device, SEGURA, Z_SEGURA)

    if USAR_GARRA:
        device.grip(False)  # garra abierta
        time.sleep(PAUSA_GARRA)

    # 2. Girar a la derecha hasta quedar sobre el objeto
    print("2. Girando a la derecha...")
    ir_a(device, AGARRE, Z_SEGURA)

    # 3. Bajar para agarrar
    print("3. Bajando al objeto...")
    bajar(device, AGARRE, Z_SEGURA, Z_GRAB)

    if USAR_GARRA:
        print("   Cerrando la garra...")
        device.grip(True)
        time.sleep(PAUSA_GARRA)

    # 4. Subir con el objeto
    print("4. Subiendo...")
    ir_a(device, AGARRE, Z_SEGURA)

    # 5. Volver a la posición de seguridad
    print("5. Volviendo a la posición de seguridad...")
    ir_a(device, SEGURA, Z_SEGURA)

    # 6. Bajar para soltar
    print("6. Bajando para soltar...")
    bajar(device, SEGURA, Z_SEGURA, Z_DROP)

    if USAR_GARRA:
        print("   Abriendo la garra...")
        device.grip(False)
        time.sleep(PAUSA_GARRA)

    # 7. Subir y quedar en posición de seguridad
    print("7. Subiendo a posición de seguridad...")
    ir_a(device, SEGURA, Z_SEGURA)

    print("Proceso finalizado con éxito.")

except Exception:
    traceback.print_exc()

finally:
    if device is not None:
        if USAR_GARRA:
            try:
                device.grip(False)  # dejar la garra abierta
            except Exception:
                pass
        try:
            device.close()
        except Exception:
            pass