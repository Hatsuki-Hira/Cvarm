from ultralytics import YOLO

model = YOLO("yolo11n.pt.pt")
result = model(source=r"D:\Home\Python\Cvarm\mouse.png",
               save=True)