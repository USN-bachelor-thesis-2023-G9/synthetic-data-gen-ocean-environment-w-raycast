from multiprocessing.dummy import freeze_support
from ultralytics import YOLO


def validate_on(data_path, model_path, classes, resolution=640):
    if resolution != 640:
        print("Validating resolution " + str(resolution))

    model = YOLO(model_path)
    metrics = model.val(data=data_path, batch=9, imgsz=resolution, split="test")

    if resolution != 640:
        csv_file = open(f"metrics_{resolution}.csv", "w")
    else:
        csv_file = open(f"metrics.csv", "w")

    csv_lines = "Class,F1,Precision,Recall,mAP50,mAP50-95\n"

    csv_lines += "all,"
    csv_lines += str(sum(metrics.box.f1)/metrics.box.nc) + ","
    csv_lines += str(metrics.box.mean_results()).strip("()[]") + "\n"

    for i in range(len(classes)):
        csv_lines += classes[i] + ","
        csv_lines += str(metrics.box.f1[i]) + ","
        csv_lines += str(metrics.box.class_result(i)).strip("()") + "\n"

    csv_file.write(csv_lines)

if __name__ == "__main__":
    resolutions = []
    resolution = 1920
    model_path = "path/to/model.pt"
    data_path = "path/to/data.yaml"
    classes = ["person", "boat"]

    if len(resolutions) != 0:
        for res in resolutions:
            data_path = data_path.split(".")[-2] + f"_{res}.yaml"
            validate_on(data_path, model_path, classes, res)
    else:
        validate_on(data_path, model_path, classes, resolution)

