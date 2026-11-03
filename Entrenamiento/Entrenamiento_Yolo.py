from ultralytics import YOLO

# Parte del mejor modelo ya entrenado (no reinicia desde cero)
model = YOLO(
    r"C:\Users\Juan\Desktop\Proyectos de Python\Proyecto_DobotMagician"
    r"\runs\detect\train-10\weights\best.pt"
)

# Fine-tuning con el dataset aumentado a 1280
results = model.train(
    data=(
        r"C:\Users\Juan\Desktop\Proyectos de Python\Proyecto_DobotMagician"
        r"\Dataugmentation\dataset_aumentado_3\data_aug.yaml"
    ),
    epochs=15,
    imgsz=1280,
    batch=2,
    workers=0,
    lr0=0.001,
    project=(
        r"C:\Users\Juan\Desktop\Proyectos de Python\Proyecto_DobotMagician"
        r"\runs\detect"
    ),
    name="train-aug",
    exist_ok=False,
)
