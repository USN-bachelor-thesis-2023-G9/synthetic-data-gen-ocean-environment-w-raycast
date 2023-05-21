import json

classes = ["swimmer", "boat", "jetski", "life_saving_appliances", "buoy", "ignored"]

def convert(size, box, width, height):
    w = box[2] / width
    h = box[3] / height
    x = box[0] / width + (1/2) * w
    y = box[1] / height + (1/2) * h
    return (x, y, w, h)

def convert_annotation(json_file, destination_dir):
    with open(json_file,"r") as f:
        data = json.load(f)

    for image in data["images"]:
        image_id = image["id"]      
        file_name = image["file_name"]
        width = image["width"]
        height = image["height"]
        annotations = filter(lambda label: label["image_id"] == image_id, data["annotations"])
        outfile = open(destination_dir + "%s.txt" % (file_name[:-4]), "a+")

        for annotation in annotations:
            category_id = annotation["category_id"]
            category_info = list(filter(lambda cat: cat["id"] == category_id, data["categories"]))
            name = category_info[0]["name"]
            class_id = classes.index(name)

            box = annotation["bbox"]
            bb = convert((width, height), box, width, height)
            
            outfile.write(str(class_id) + " " + " ".join([str(a) for a in bb]) + "\n")

        outfile.close()

convert_annotation("D:/conversions/coco/instances_val.json", "D:/conversions/yolo_coco_classes/")