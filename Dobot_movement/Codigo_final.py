import gc
import math
import time
import struct
import traceback
import serial
import cv2                                   # Para la cámara y la ventana de video
from ultralytics import YOLO                 # Para el modelo de detección
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


# ==================================================
# CONFIGURACIÓN
# ==================================================
# Ejes: +X adelante, +Y IZQUIERDA, -Y derecha, +Z arriba.
# Los puntos están a 210 mm de la base (centro del área de trabajo).
# Si el brazo gira hacia el lado contrario, cambia el signo de Y.

# Banda transportadora
PUERTO_MOTOR_BANDA = 0   # 0 = Stepper 1
VELOCIDAD_BANDA = 15000  # Si la banda gira al revés, usa un valor negativo
RETARDO_PARADA = 0.0     # Segundos entre que el objeto cruza la línea y se para la banda
                         # (AJUSTA ESTE VALOR para que el objeto quede justo bajo el Dobot)
MAX_OBJETOS = None       # Cantidad de objetos a procesar (None = hasta Ctrl+C o tecla 'q')

# Cámara y modelo YOLO
RUTA_MODELO = r'C:\Users\Juan\Desktop\Proyectos de Python\Proyecto_DobotMagician\best_version121126.pt'
INDICE_CAMARA = 0        # Cámara web externa
CONFIANZA_MIN = 0.30     # Confianza mínima de YOLO para aceptar una detección
TAMANO_IMG = 640         # Tamaño de imagen para la inferencia
VENTANA = "Deteccion de Componentes en Linea de Ensamblaje"

# Línea virtual de disparo
LINEA_X_FRAC = 0.60      # Posición inicial de la línea: 0.0 = borde izquierdo, 1.0 = borde derecho.
                         # Se mueve EN VIVO con las teclas 'a' (izquierda) y 'd' (derecha).
PASO_LINEA = 0.01        # Cuánto se mueve la línea con cada pulsación
DIRECCION_MOVIMIENTO = 0 # 0 = dispara al cruzar en cualquier sentido (recomendado)
                         # +1 = solo izquierda->derecha | -1 = solo derecha->izquierda
MAX_SALTO_PX = 150       # Máx. píxeles que un objeto puede avanzar entre fotogramas y seguir siendo "el mismo"
FRAMES_OLVIDO = 10       # Fotogramas sin ver objetos antes de olvidar las posiciones anteriores
PAUSA_ESTABILIZAR = 0.5  # Segundos que se leen fotogramas tras cada ciclo para vaciar los viejos

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


# Abre la cámara con reintentos y "calentamiento" (los primeros fotogramas suelen fallar).
def abrir_camara(intentos=3):
    """Abre la cámara y espera a que entregue un fotograma válido. Devuelve None si no lo logra."""
    for intento in range(1, intentos + 1):
        cap = cv2.VideoCapture(INDICE_CAMARA, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if cap.isOpened():
            for _ in range(30):          # Hasta ~3 s esperando el primer fotograma bueno
                ret, _frame = cap.read()
                if ret:
                    return cap
                time.sleep(0.1)
        cap.release()                    # Libera la cámara antes de reintentar
        print(f"Intento {intento}/{intentos}: la cámara no entregó imagen.")
        time.sleep(1.0)
    return None


# Posiciones del fotograma anterior, para detectar el cruce real de la línea
estado_seguimiento = {"previos": [], "sin_objetos": 0}


def reiniciar_seguimiento():
    estado_seguimiento["previos"] = []
    estado_seguimiento["sin_objetos"] = 0


def detectar_cruce(model, frame):
    """
    Corre YOLO, dibuja detecciones y la línea virtual, y devuelve True SOLO en el
    fotograma en que el centro de un objeto pasa de un lado de la línea al otro
    (comparando con su posición en el fotograma anterior).
    """
    results = model(frame, imgsz=TAMANO_IMG, conf=CONFIANZA_MIN, verbose=False)
    imagen = results[0].plot()
    alto, ancho = imagen.shape[:2]
    linea_x = int(ancho * LINEA_X_FRAC)

    actuales = [(int(c[0]), int(c[1])) for c in results[0].boxes.xywh.cpu().numpy()]
    previos = estado_seguimiento["previos"]

    cruzo = False
    for (cx, cy) in actuales:
        if not previos:
            break
        # Posición anterior más cercana = el mismo objeto un fotograma antes
        px, py = min(previos, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
        if math.hypot(px - cx, py - cy) > MAX_SALTO_PX:
            continue
        if px < linea_x <= cx:
            sentido = 1      # cruzó de izquierda a derecha
        elif px > linea_x >= cx:
            sentido = -1     # cruzó de derecha a izquierda
        else:
            continue
        if DIRECCION_MOVIMIENTO in (0, sentido):
            cruzo = True

    # Actualizar memoria de posiciones
    if actuales:
        estado_seguimiento["previos"] = actuales
        estado_seguimiento["sin_objetos"] = 0
    else:
        estado_seguimiento["sin_objetos"] += 1
        if estado_seguimiento["sin_objetos"] >= FRAMES_OLVIDO:
            estado_seguimiento["previos"] = []

    # Dibujo: puntos en los centros y línea (verde en el instante del cruce)
    for (cx, cy) in actuales:
        cv2.circle(imagen, (cx, cy), 6, (0, 255, 255), -1)
    color = (0, 255, 0) if cruzo else (0, 0, 255)
    cv2.line(imagen, (linea_x, 0), (linea_x, alto), color, 3)
    cv2.putText(imagen, f"LINEA {LINEA_X_FRAC:.2f}  (a/d mover, q salir)", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.imshow(VENTANA, imagen)
    return cruzo


def manejar_teclas():
    """'a'/'d' mueven la línea en vivo. Devuelve False si se presionó 'q'."""
    global LINEA_X_FRAC
    tecla = cv2.waitKey(1) & 0xFF
    if tecla == ord('q'):
        return False
    if tecla == ord('a'):
        LINEA_X_FRAC = max(0.05, round(LINEA_X_FRAC - PASO_LINEA, 3))
    elif tecla == ord('d'):
        LINEA_X_FRAC = min(0.95, round(LINEA_X_FRAC + PASO_LINEA, 3))
    return True


def estabilizar_tras_ciclo(cap, model):
    """
    Con la banda ya en marcha, lee fotogramas un momento para vaciar los viejos
    acumulados mientras el brazo trabajaba, e ignora cualquier cruce en ese lapso.
    Devuelve False si el usuario presiona 'q'.
    """
    inicio = time.time()
    while time.time() - inicio < PAUSA_ESTABILIZAR:
        ret, frame = cap.read()
        if not ret:
            continue
        detectar_cruce(model, frame)
        if not manejar_teclas():
            return False
    reiniciar_seguimiento()
    return True


def ciclo_agarre(device):
    """
    El brazo espera sobre la banda (AGARRE a Z_SEGURA). Al detectar el objeto:
    baja, lo agarra, sube, GIRA a SEGURA para soltarlo y vuelve a girar a AGARRE a esperar.
    """
    # Ya está sobre el objeto: solo baja para agarrar y cierra la garra
    print("   Bajando al objeto...")
    ir_a(device, AGARRE, Z_GRAB)
    print("   Cerrando la garra (agarre)...")
    cerrar_garra(device)

    # Subir con el objeto
    print("   Subiendo...")
    ir_a(device, AGARRE, Z_SEGURA)

    # Girar a la posición de soltar
    print("   Girando para soltar...")
    girar_a(device, SEGURA, Z_SEGURA)

    # Bajar y soltar (la bomba invierte el aire)
    print("   Bajando para soltar...")
    ir_a(device, SEGURA, Z_DROP)
    print("   Abriendo la garra (soltando)...")
    abrir_garra(device)

    # Subir y volver a girar a la posición de espera sobre la banda
    ir_a(device, SEGURA, Z_SEGURA)
    print("   Volviendo a la posición de espera sobre la banda...")
    girar_a(device, AGARRE, Z_SEGURA)


# ==================================================
# PROGRAMA
# ==================================================
puerto = detectar_puerto_dobot()
if puerto is None:
    print("No se encontró el Dobot (chip CP210x). Conéctalo por USB y verifica el driver.")
    exit()

device = None
cap = None   # Se declara aquí para poder liberarla en el finally
try:
    device = conectar_dobot(puerto)
    print(f"Conectado exitosamente al Dobot en {puerto}")
    time.sleep(2)

    device.speed(VELOCIDAD, ACELERACION)
    print(f"Punto de agarre: X={AGARRE[0]:.1f}  Y={AGARRE[1]:.1f}  Z={Z_GRAB:.1f}")

    # Cargar el modelo YOLO (la cámara se abre más abajo, justo antes del ciclo)
    model = YOLO(RUTA_MODELO)

    # Garra abierta (grip(False) invierte el aire) y posición de seguridad
    print("Abriendo la garra...")
    abrir_garra(device)
    print("Posición de seguridad...")
    ir_a(device, SEGURA, Z_SEGURA)
    print("Posición de espera sobre la banda...")
    girar_a(device, AGARRE, Z_SEGURA)   # El brazo se queda aquí esperando el objeto

    # Encender la cámara AHORA (después de mover el Dobot), con reintentos
    cap = abrir_camara()
    if cap is None:
        print("Error: No se pudo leer la cámara. Cierra otros programas que la usen o prueba otro INDICE_CAMARA.")
        exit()
    print("Cámara encendida. Presiona la tecla 'q' sobre la ventana de video para terminar.")

    # Ciclo: banda en marcha -> objeto cruza la línea -> banda para -> agarre y depósito -> banda continúa
    objetos = 0
    fallos_camara = 0  # Fotogramas fallidos seguidos
    print("Cámara OK. Arrancando la banda...")
    mover_banda(device, VELOCIDAD_BANDA, PUERTO_MOTOR_BANDA)
    print("Banda en marcha, esperando que un objeto cruce la línea...")

    while MAX_OBJETOS is None or objetos < MAX_OBJETOS:
        # Leer un fotograma de la cámara
        ret, frame = cap.read()
        if not ret:
            fallos_camara += 1
            if fallos_camara >= 30:   # 30 fallos seguidos: la cámara se cayó
                print("La cámara dejó de responder. Reconectando...")
                cap.release()
                time.sleep(1.0)
                cap = abrir_camara()   # Intenta reabrirla (hasta 3 veces)
                if cap is None:
                    print("Error al leer el fotograma: no se pudo reconectar la cámara.")
                    break
                fallos_camara = 0
                reiniciar_seguimiento()
                continue
            time.sleep(0.05)
            continue
        fallos_camara = 0

        # Detectar con YOLO y comprobar si un objeto acaba de CRUZAR la línea
        cruzo = detectar_cruce(model, frame)

        # Teclas: 'a'/'d' mueven la línea, 'q' termina
        if not manejar_teclas():
            break

        # Si un objeto cruzó la línea, se detiene la banda y el Dobot hace su ciclo
        if cruzo:
            time.sleep(RETARDO_PARADA)                   # Deja que el objeto avance hasta el punto de agarre
            mover_banda(device, 0, PUERTO_MOTOR_BANDA)   # Detener la banda
            objetos += 1
            print(f"Objeto {objetos} cruzó la línea: banda detenida.")

            ciclo_agarre(device)   # El Dobot agarra el objeto y lo coloca en otro lugar

            print("Banda en marcha, esperando que un objeto cruce la línea...")
            mover_banda(device, VELOCIDAD_BANDA, PUERTO_MOTOR_BANDA)
            if not estabilizar_tras_ciclo(cap, model):
                break

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
    # Liberar la cámara y cerrar la ventana de video
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()

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