import sys
import time
import struct
import msvcrt
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
    SetEndEffectorGripper (id 63) y SetEndEffectorSuctionCup (id 62, que
    según el protocolo oficial es el que controla la bomba de aire),
    ambos con isCtrlEnabled=0 y ejecución inmediata (no encolada).
    """
    for id_comando in (63, 62):
        msg = Message()
        msg.id = id_comando
        msg.ctrl = 0x01 # Ejecución inmediata (no encolada)
        msg.params = bytearray([0x00, 0x00]) # isCtrlEnabled=0, estado=0
        dispositivo._send_command(msg)


PUERTO_MOTOR_BANDA = 0 # 0 = Stepper 1
VELOCIDAD_BANDA = 15000 # Si la banda gira al revés, usa un valor negativo

puerto = detectar_puerto_dobot()
if puerto is None:
    print("No se encontró el Dobot (chip CP210x). Conéctalo por USB y verifica el driver.")
    exit()

# Selección de modo:
# - [ENTER] presionado 2 veces -> programa habitual (pick & place en la banda)
# - cualquier otra tecla -> entra al modo calibración
# - argumento 'calibrar' -> directo a calibración sin preguntar
ejecutar_calibracion = len(sys.argv) > 1 and sys.argv[1].lower() == 'calibrar'

if not ejecutar_calibracion:
    print("=" * 50)
    print("  DOBOT MAGICIAN - PICK & PLACE")
    print("=" * 50)
    print("[ENTER] x2  -> ejecutar el programa habitual")
    print("Otra tecla  -> entrar al modo calibración\n")

    modo = None
    enter_presionados = 0
    while modo is None:
        tecla = msvcrt.getch()
        if tecla in (b'\x00', b'\xe0'):
            msvcrt.getch() # descartar el segundo byte de teclas especiales (flechas)
        elif tecla in (b'\r', b'\n'):
            enter_presionados += 1
            print(f"ENTER presionado ({enter_presionados}/2)")
            if enter_presionados == 2:
                modo = 'habitual'
        else:
            modo = 'calibrar'

    ejecutar_calibracion = modo == 'calibrar'

if ejecutar_calibracion:
    print(f"Conectado al Dobot en {puerto} (modo calibración).")
    print("Mueve el brazo a mano con el botón de desbloqueo del antebrazo.")
    print("[ENTER] guarda el punto actual   [Q] termina y muestra el resumen\n")
    device = Dobot(port=puerto, verbose=False)
    puntos = []
    try:
        while True:
            x, y, z, r, j1, j2, j3, j4 = device.pose()
            print(f"\rX: {x:7.1f}   Y: {y:7.1f}   Z: {z:7.1f}   R: {r:6.1f}   ", end='', flush=True)
            if msvcrt.kbhit():
                tecla = msvcrt.getch()
                if tecla in (b'\x00', b'\xe0'):
                    msvcrt.getch() # descartar el segundo byte de teclas especiales (flechas)
                elif tecla in (b'\r', b'\n'):
                    puntos.append((x, y, z, r))
                    print(f"\nPunto {len(puntos)} guardado -> X: {x:.1f}  Y: {y:.1f}  Z: {z:.1f}  R: {r:.1f}")
                elif tecla.lower() == b'q':
                    break
            time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            apagar_bomba(device) # cortar la bomba si quedó encendida de antes
        except Exception:
            pass
        device.close()

    print("\n========== RESUMEN DE CALIBRACIÓN ==========")
    for i, (x, y, z, r) in enumerate(puntos, 1):
        print(f"Punto {i}:  X = {x:.1f}   Y = {y:.1f}   Z = {z:.1f}   R = {r:.1f}")
        print(f"   device.move_to(x={x:.1f}, y={y:.1f}, z={z:.1f}, r={r:.1f})")
    if puntos:
        print("\nCopia estos valores en TARGET_X/TARGET_Y/Z_GRAB (agarre en la banda)")
        print("y Z_DROP (zona de descarga) del programa.")
    exit()

try:
    print(f"Conectado exitosamente al Dobot en {puerto}")
    device = Dobot(port=puerto, verbose=False)

    # Coordenadas del robot: +X adelante, +Y a la IZQUIERDA, -Y a la DERECHA, +Z arriba
    TARGET_X = 60.0
    TARGET_Y = 160.0 # La banda está a la derecha del robot (-Y)
    Z_HOVER = 30.0
    # Calibrar Z_GRAB con el modo 'calibrar': mueve la garra a mano hasta
    # que apenas toque el componente sobre la banda y anota esa altura Z.
    Z_GRAB = -20.0 # Altura de agarre (sobre la banda)
    Z_DROP = -15.0 # Altura de la zona de descarga (fuera de la banda)
    R_ANGLE = 0.0

    # La banda queda girando continuamente durante todo el ciclo de movimiento
    print("Encendiendo la banda transportadora...")
    mover_banda(device, VELOCIDAD_BANDA, PUERTO_MOTOR_BANDA)

    # La bomba del Dobot trabaja en ambos sentidos:
    # grip(True) cierra la garra y grip(False) invierte el aire para abrirla
    print("Abriendo la garra...")
    device.grip(False)
    time.sleep(1.0)

    # 1. Posición de aproximación
    print("1. Moviendo a posición de aproximación...")
    device.move_to(x=TARGET_X, y=TARGET_Y, z=Z_HOVER, r=R_ANGLE, wait=True)

    # 2. Bajar al objeto en pasos de 5 mm para no golpear la banda
    print("2. Descendiendo...")
    z_actual = Z_HOVER
    while z_actual > Z_GRAB:
        z_actual = max(Z_GRAB, z_actual - 5.0)
        device.move_to(x=TARGET_X, y=TARGET_Y, z=z_actual, r=R_ANGLE, wait=True)

    # 3. Cerrar garra
    print("3. Cerrando la garra (agarre)...")
    device.grip(True)
    time.sleep(1.0)

    # 4. Elevar objeto
    print("4. Elevando...")
    device.move_to(x=TARGET_X, y=TARGET_Y, z=Z_HOVER, r=R_ANGLE, wait=True)

    # 5. Mover a zona de descarga
    print("5. Moviendo a zona de descarga...")
    device.move_to(x=200.0, y=0.0, z=Z_HOVER, r=R_ANGLE, wait=True)
    device.move_to(x=200.0, y=0.0, z=Z_DROP, r=R_ANGLE, wait=True)

    # 6. Abrir garra (la bomba invierte el aire para soltar el componente)
    print("6. Abriendo la garra (soltando)...")
    device.grip(False)
    time.sleep(1.0)

    # Retorno a posición de seguridad
    device.move_to(x=200.0, y=0.0, z=Z_HOVER, r=R_ANGLE, wait=True)

    # Detener la banda al terminar el ciclo
    print("Deteniendo la banda transportadora...")
    mover_banda(device, 0, PUERTO_MOTOR_BANDA)

    # 7. Apagar la bomba de aire para que no quede encendida al terminar
    print("Apagando la bomba de aire...")
    apagar_bomba(device)

    print("Proceso finalizado con éxito.")

except Exception as e:
    print(f"Error durante la ejecución: {e}")

finally:
    if 'device' in locals():
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
