import DobotDllType as dType
import time

# Cargar la librería del Dobot
api = dType.load()

# Conectar al Dobot (Reemplaza "COM3" por tu puerto)
puerto_dobot = "COM3"
estado_conexion = dType.ConnectDobot(api, puerto_dobot, 115200)[0]

if estado_conexion == dType.DobotConnect.DobotConnect_NoError:
    print("Conexión exitosa. Iniciando movimiento simultáneo...")
    
    # Limpiar cualquier comando previo en cola
    dType.SetQueuedCmdClear(api)
    
    # 1. Configurar velocidad de la banda transportadora
    # Parámetros SetEMotor: api, puerto (0 o 1), habilitar (1), velocidad (pulsos/s), isQueued (0 = ejecución inmediata)
    puerto_motor_banda = 0
    velocidad_banda = 15000 
    
    # 2. Iniciar el movimiento de la banda (arranca de inmediato)
    dType.SetEMotor(api, puerto_motor_banda, 1, velocidad_banda, 0)
    
    # 3. Iniciar el movimiento del brazo (arranca de inmediato)
    # El brazo se moverá hacia la coordenada X=250, Y=0, Z=50 mientras la banda está girando
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 250, 0, 50, 0, isQueued=0)
    
    # Dejar que ambos se muevan simultáneamente durante 4 segundos
    print("Banda y brazo moviéndose al tiempo...")
    time.sleep(4)
    
    # 4. Detener la banda transportadora
    print("Deteniendo sistema...")
    dType.SetEMotor(api, puerto_motor_banda, 0, 0, 0)
    
    # Opcional: Volver el brazo a la posición Home
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, 200, 0, 150, 0, isQueued=0)
    time.sleep(2) # Esperar a que llegue a Home
    
    # Desconectar de forma segura
    dType.DisconnectDobot(api)
    print("Prueba finalizada y Dobot desconectado.")

else:
    print("Error de conexión. Verifica el puerto COM.")