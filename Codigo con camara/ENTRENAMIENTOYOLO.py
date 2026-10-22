from ultralytics import YOLO

# Carga un modelo base de YOLO (puedes usar yolov8n.pt o yolo11s.pt)
model = YOLO("yolov8n.pt")

# Inicia el entrenamiento apuntando al archivo data.yaml del dataset descargado
results = model.train(
    data="Electronic-Component-Picking-2/data.yaml", 
    epochs=15,       # Puedes ajustar las épocas si deseas que entrene por más tiempo
    imgsz=1080
)