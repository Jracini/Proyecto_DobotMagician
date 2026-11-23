import gc
import math
import time
import struct
import traceback
import serial
from serial.tools import list_ports
from pydobot import Dobot
from pydobot.message import Message

try:
    from pydobot.enums.ptpMode import PTPMode
except ImportError:
    PTPMode = None  # versión de pydobot sin este módulo: se usará movimiento lineal


# ==================================================
# CONEXIÓN ROBUSTA (evita IndexError / NoneType al conectar)
# ==================================================
def _leer_mensaje_robusto(self):
    """
    Reemplaza a Dobot._read_message de pydobot, que espera solo 0.1 s y falla si la
    respuesta llega tarde (None) o incompleta (IndexError). Esta versión espera
    hasta MAX_ESPERA_RESPUESTA segundos a que llegue el mensaje COMPLETO.
    """
    buffer = bytearray()
    limite = time.time() + MAX_ESPERA_RESPUESTA
    while time.time() < limite:
        time.sleep(0.02)
        buffer.extend(self.ser.read_all())
        inicio = buffer.find(b'\xaa\xaa')
        if inicio >= 0 and len(buffer) >= inicio + 3:
            total = buffer[inicio + 2] + 4  # cabecera(2) + largo(1) + datos + checksum(1)
            if len(buffer) >= inicio + total:
                return Message(bytes(buffer[inicio:inicio + total]))
    return None


MAX_ESPERA_RESPUESTA = 1.0   # Segundos máximos esperando la respuesta del robot
Dobot._read_message = _leer_mensaje_robusto


def conectar_dobot(puerto, intentos=5, espera=3.0):
    """Conecta con el Dobot reintentando (tras reiniciar tarda en estar listo)."""
    for intento in range(1, intentos + 1):
        try:
            return Dobot(port=puerto, verbose=False)
        except (AttributeError, IndexError, struct.error, serial.SerialException) as e:
            print(f"Intento {intento}/{intentos}: el Dobot no respondió ({type(e).__name__}).")
            gc.collect()  # libera el puerto serie del intento fallido
            time.sleep(espera)
    print("No se pudo conectar. Revisa el cable USB, la alimentación y que ningún otro programa use el puerto.")
    exit()


# ==================================================
# FUNCIONES DE HARDWARE
# ==================================================
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
    Equivale a SetEMotor() del SDK oficial (DobotDllType).
    Velocidad positiva gira hacia un lado, negativa hacia el otro. 0 detiene.
    """
    estado = 1 if velocidad != 0 else 0
    msg = Message()
    msg.id = 135 # ID del comando EMotor
    msg.ctrl = 0x01 # Ejecución inmediata con respuesta (equivale a isQueued=0)
    msg.params = bytearray(struct.pack('<BBi', puerto_stepper, estado, int(velocidad)))
    dispositivo._send_command(msg)


def apagar_bomba(dispositivo):
    """
    Detiene la bomba de aire/vacío del Dobot y la deja APAGADA al terminar
    (el robot conserva el último estado aunque se cierre el puerto serie).
    Envía el estado "Disable" del efector por los DOS comandos del protocolo:
    SetEndEffectorGripper (id 63) y SetEndEffectorSuctionCup (id 62),
    ambos con isCtrlEnabled=0 y ejecución inmediata (no encolada).
    """
    for id_comando in (63, 62):
        msg = Message()
        msg.id = id_comando
        msg.ctrl = 0x01 # Ejecución inmediata (no encolada)
        msg.params = bytearray([0x00, 0x00]) # isCtrlEnabled=0, estado=0
        dispositivo._send_command(msg)


def activar_sensor(dispositivo, activar=True):
    """
    Activa el sensor infrarrojo de la banda (comando 138, InfraredSensor).
    Parámetros: isEnable, puerto GP (y versión, si SENSOR_VERSION no es None).
    """
    params = [1 if activar else 0, SENSOR_PUERTO]
    if SENSOR_VERSION is not None:
        params.append(SENSOR_VERSION)
    msg = Message()
    msg.id = 138
    msg.ctrl = 0x01 # Escritura inmediata (no encolada)
    msg.params = bytearray(params)
    dispositivo._send_command(msg)


def leer_sensor(dispositivo):
    """
    Lee el sensor infrarrojo (comando 138, lectura inmediata).
    Devuelve 0 o 1, o None si el robot no responde.
    """
    msg = Message()
    msg.id = 138
    msg.ctrl = 0x00 # Lectura inmediata (no encolada)
    msg.params = bytearray([SENSOR_PUERTO])
    try:
        respuesta = dispositivo._send_command(msg)
        if respuesta is None or respuesta.id != 138 or len(respuesta.params) == 0:
            return None  # sin respuesta, o respuesta de otro comando
        return respuesta.params[0]
    except Exception:
        return None


# ==================================================
# CONFIGURACIÓN
# ==================================================
# Ejes: +X adelante, +Y IZQUIERDA, -Y derecha, +Z arriba.
# Los puntos están a 210 mm de la base (centro del área de trabajo).
# Si el brazo gira hacia el lado contrario, cambia el signo de Y.

# Banda transportadora
PUERTO_MOTOR_BANDA = 0   # 0 = Stepper 1
VELOCIDAD_BANDA = 15000  # Si la banda gira al revés, usa un valor negativo

# Sensor infrarrojo de la banda
SENSOR_PUERTO = 1        # Puerto GP donde está conectado: 0=GP1, 1=GP2, 2=GP4, 3=GP5
SENSOR_VERSION = None    # Si el sensor no responde, prueba con 1 (versión del sensor)
SENSOR_ACTIVO = 1        # Valor que entrega el sensor al detectar un objeto (si es al revés, usa 0)
INTERVALO_SENSOR = 0.05  # Segundos entre lecturas del sensor
RETARDO_PARADA = 0.0     # Segundos entre detectar y parar la banda (para centrar el objeto)
MAX_OBJETOS = None       # Cantidad de objetos a procesar (None = hasta Ctrl+C)

# Alturas
Z_SEGURA = 30.0          # Altura de desplazamiento (sin tocar nada)
Z_GRAB_BASE = -20.0      # Altura de agarre que se usaba sin garra
OFFSET_GARRA = 50.0      # La garra alarga la herramienta: ~5 cm más abajo al agarrar
Z_GRAB = Z_GRAB_BASE - OFFSET_GARRA   # = -70
Z_DROP = 20.0            # Al soltar sobre la banda queda casi a la altura segura

# Movimiento
R_ANGLE = 0.0
VELOCIDAD = 200          # Velocidad del brazo (si salta alarma o es brusco, bájala)
ACELERACION = 200        # Aceleración del brazo
PAUSA_CERRAR = 1.0       # Espera para que la bomba genere el agarre
PAUSA_ABRIR = 1.0        # Espera para que el aire invertido suelte el objeto

# Posición de seguridad / soltar (frente al robot, ángulo 0°)
SEGURA = (210.0, 0.0)
# Punto de agarre, a la izquierda. Más ángulo = más a la izquierda.
RADIO_AGARRE = 210.0     # Distancia a la base (mm)
ANGULO_AGARRE = 80.0     # Grados a la izquierda desde el frente (el límite de la base es ~90)
AGARRE = (RADIO_AGARRE * math.cos(math.radians(ANGULO_AGARRE)),
          RADIO_AGARRE * math.sin(math.radians(ANGULO_AGARRE)))


# ==================================================
# FUNCIONES DE MOVIMIENTO
# ==================================================
def ir_a(device, punto, z):
    """Mueve el brazo a (x, y) a la altura z y espera a que termine."""
    device.move_to(x=punto[0], y=punto[1], z=z, r=R_ANGLE, wait=True)


def girar_a(device, punto, z):
    """
    Gira la base hacia (x, y) a la altura z moviendo por ARTICULACIONES (MOVJ).
    Como el radio y la altura no cambian, solo gira la base: el brazo recorre un
    arco y NO pasa cerca de la base, que era lo que lo llevaba a la alarma.
    """
    if PTPMode is None:
        ir_a(device, punto, z)
        return
    device._set_ptp_cmd(punto[0], punto[1], z, R_ANGLE, mode=PTPMode.MOVJ_XYZ, wait=True)


def intentar(accion, descripcion):
    """Ejecuta una acción de banda/bomba; si falla avisa pero no detiene el movimiento."""
    try:
        accion()
    except Exception as e:
        print(f"   Aviso: no se pudo {descripcion} ({e})")


def cerrar_garra(device):
    """Cierra la garra: la bomba aspira (presión directa) y espera el agarre."""
    device.grip(True)
    time.sleep(PAUSA_CERRAR)


def abrir_garra(device):
    """Abre la garra: la bomba INVIERTE el aire (presión inversa) para soltar."""
    device.grip(False)
    time.sleep(PAUSA_ABRIR)


def esperar_objeto(device):
    """Con la banda girando, espera a que el sensor detecte un objeto."""
    fallos = 0
    while True:
        valor = leer_sensor(device)
        if valor is None:
            fallos += 1
            if fallos >= 20:
                raise RuntimeError("El sensor dejó de responder. Revisa el cable y SENSOR_PUERTO.")
        else:
            fallos = 0
            if valor == SENSOR_ACTIVO:
                return
        time.sleep(INTERVALO_SENSOR)


def esperar_sensor_libre(device, tiempo_max=10.0):
    """Espera a que el sensor deje de detectar (evita contar dos veces el mismo objeto)."""
    inicio = time.time()
    while time.time() - inicio < tiempo_max:
        if leer_sensor(device) != SENSOR_ACTIVO:
            return
        time.sleep(INTERVALO_SENSOR)


def ciclo_agarre(device):
    """Movimiento de agarre y soltar, partiendo y terminando en la posición de seguridad."""
    # Girar a la izquierda hasta quedar sobre el objeto
    print("   Girando a la izquierda...")
    girar_a(device, AGARRE, Z_SEGURA)

    # Bajar para agarrar y cerrar la garra
    print("   Bajando al objeto...")
    ir_a(device, AGARRE, Z_GRAB)
    print("   Cerrando la garra (agarre)...")
    cerrar_garra(device)

    # Subir con el objeto
    print("   Subiendo...")
    ir_a(device, AGARRE, Z_SEGURA)

    # Volver a la posición de seguridad
    print("   Volviendo a la posición de seguridad...")
    girar_a(device, SEGURA, Z_SEGURA)

    # Bajar sobre la banda y soltar (la bomba invierte el aire)
    print("   Bajando para soltar...")
    ir_a(device, SEGURA, Z_DROP)
    print("   Abriendo la garra (soltando)...")
    abrir_garra(device)

    # Subir a la posición de seguridad
    ir_a(device, SEGURA, Z_SEGURA)


# ==================================================
# PROGRAMA
# ==================================================
puerto = detectar_puerto_dobot()
if puerto is None:
    print("No se encontró el Dobot (chip CP210x). Conéctalo por USB y verifica el driver.")
    exit()

device = None
try:
    device = conectar_dobot(puerto)
    print(f"Conectado exitosamente al Dobot en {puerto}")
    time.sleep(2)

    device.speed(VELOCIDAD, ACELERACION)
    print(f"Punto de agarre: X={AGARRE[0]:.1f}  Y={AGARRE[1]:.1f}  Z={Z_GRAB:.1f}")

    # Activar el sensor y comprobar que responde
    activar_sensor(device)
    time.sleep(0.5)
    if leer_sensor(device) is None:
        print("El sensor no responde. Revisa el cable, SENSOR_PUERTO y SENSOR_VERSION.")
        exit()
    print("Sensor activado.")

    # Garra abierta (grip(False) invierte el aire) y posición de seguridad
    print("Abriendo la garra...")
    abrir_garra(device)
    print("Posición de seguridad...")
    ir_a(device, SEGURA, Z_SEGURA)

    # Ciclo: banda en marcha -> sensor detecta -> banda para -> agarre y soltar -> banda continúa
    objetos = 0
    while MAX_OBJETOS is None or objetos < MAX_OBJETOS:
        print("Banda en marcha, esperando objeto...")
        mover_banda(device, VELOCIDAD_BANDA, PUERTO_MOTOR_BANDA)
        esperar_objeto(device)

        time.sleep(RETARDO_PARADA)
        mover_banda(device, 0, PUERTO_MOTOR_BANDA)
        objetos += 1
        print(f"Objeto {objetos} detectado: banda detenida.")

        ciclo_agarre(device)

        esperar_sensor_libre(device)

    # Detener la banda y apagar la bomba al terminar
    print("Deteniendo la banda transportadora...")
    mover_banda(device, 0, PUERTO_MOTOR_BANDA)
    print("Apagando la bomba de aire...")
    apagar_bomba(device)
    print(f"Proceso finalizado con éxito ({objetos} objetos).")

except KeyboardInterrupt:
    print("\nDetenido por el usuario.")

except Exception:
    traceback.print_exc()

finally:
    if device is not None:
        try:
            device.grip(False) # soltar la garra si quedó cerrada
        except Exception:
            pass
        try:
            apagar_bomba(device) # apagar la bomba SIEMPRE, sea la última orden
        except Exception:
            pass
        try:
            mover_banda(device, 0, PUERTO_MOTOR_BANDA)
        except Exception:
            pass
        try:
            device.close()
        except Exception:
            pass