from roboflow import Roboflow

# Descarga el dataset desde tu proyecto de Roboflow
rf = Roboflow(api_key="g1MIdmzEZY3pDZm1gdtS")
project = rf.workspace("customcomponentesdetectoryolo11").project("electronic-component-picking")
version = project.version(2)
dataset = version.download("yolov8")