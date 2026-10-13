import cv2
from ultralytics import YOLO

# Carga el modelo básico de YOLO (descargará un archivo pequeño la primera vez)
model = YOLO('yolov8n.pt')

# Inicia la conexión con tu cámara web. 
# Si tienes una laptop, '0' suele ser la cámara integrada. Si usas una externa, intenta con '1'.
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("Error: No se pudo conectar a la cámara web.")
    exit()

print("Cámara encendida. Presiona la tecla 'q' sobre la ventana de video para apagarla.")

while True:
    # Captura cuadro por cuadro
    ret, frame = cap.read()
    if not ret:
        print("Se perdió la señal de video.")   
        break

    # Pasa la imagen por la inteligencia artificial
    resultados = model(frame)

    # Dibuja las cajas delimitadoras sobre los objetos que encuentre
    imagen_anotada = resultados[0].plot()

    # Muestra el resultado en una ventana flotante
    cv2.imshow('Prueba Visión Artificial - YOLO', imagen_anotada)

    # Espera a que presiones 'q' para romper el ciclo
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Apaga la cámara y cierra la ventana
cap.release()
cv2.destroyAllWindows()