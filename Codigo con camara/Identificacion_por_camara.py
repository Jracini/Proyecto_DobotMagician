import cv2
from ultralytics import YOLO

# Carga tu nuevo modelo (fíjate que la ruta ahora dice train-9)
model = YOLO(r'C:\Users\Juan\Desktop\Movimiento de banda\runs\detect\train-10\weights\best.pt')
# Inicia la conexión con tu cámara web externa
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

# Solicitar resolución 1080p (Full HD)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("Error: No se pudo conectar a la cámara web.")
    exit()

print("Cámara encendida. Presiona la tecla 'q' sobre la ventana de video para apagarla")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error al leer el fotograma.")
        break

    # Realizar la inferencia ajustando la confianza mínima a 25% para forzar la detección
    results = model(frame, conf=0.25)
    
    # Visualizar los resultados en tiempo real con las etiquetas
    annotated_frame = results[0].plot()

    cv2.imshow("Deteccion de Componentes en Linea de Ensamblaje", annotated_frame)

    # Salir al presionar la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()