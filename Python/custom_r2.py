import random
import airsim
import cv2
import os
import numpy as np
import math

# camera to use
camera_name = "0"

basepath = "c:/temp/"
filecounter = 0

# list of the image types that are wanted
requests = [
    airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)
    ]

detectionImageType = airsim.ImageType.Scene;

def newClient():
    # establish connection to airsim misc
    global client
    client = airsim.VehicleClient()
    client.confirmConnection()

    client.simSetDetectionFilterRadius(camera_name, detectionImageType, 80 * 100) # in [cm]
    client.simAddDetectionFilterMeshName(camera_name, detectionImageType, "Cone")
    client.simAddDetectionFilterMeshName(camera_name, detectionImageType, "Cube")
    client.simAddDetectionFilterMeshName(camera_name, detectionImageType, "Cylinder") 
    client.simAddDetectionFilterMeshName(camera_name, detectionImageType, "Sphere")

def writeImageFromResponse(response, imagepath):
    if response.pixels_as_float:
        imagepath = imagepath + ".pfm"
        airsim.write_pfm(os.path.normpath(imagepath), airsim.get_pfm_array(response))
    elif response.compress:
        imagepath = imagepath + ".png"
        airsim.write_file(os.path.normpath(imagepath), response.image_data_uint8)
    else:
        imagepath = imagepath + ".png"
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(response.height, response.width, 3)
        cv2.imwrite(os.path.normpath(imagepath), img_rgb)

def detectionsToYOLO(detections, image_width, image_height):
    yolo = ""
    if detections:
        for idx, detection in enumerate(detections):
            if(idx != 0):
                yolo = yolo + "\n"
            norm_min_x = detection.box2D.min.x_val/image_width
            norm_min_y = detection.box2D.min.y_val/image_height
            width = (detection.box2D.max.x_val - detection.box2D.min.x_val)/image_width
            height = (detection.box2D.max.y_val - detection.box2D.min.y_val)/image_height
            center_x = norm_min_x + width/2
            center_y = norm_min_y + height/2

            class_id = -1
            name = detection.name
            if name.startswith("Cube"):
                class_id = 0
            elif name.startswith("Cylinder"):
                class_id = 1
            elif name.startswith("Cone"):
                class_id = 2
            elif name.startswith("Sphere"):
                class_id = 3
            else:
                print("Can't infer class for name " + name);
            yolo = yolo + f"{class_id} {center_x} {center_y} {width} {height}"
    return yolo

def takePicture(basepath):
    global filecounter
    responses = client.simGetImages(requests)

    #Get object detections on main camera
    detections = client.simGetDetections(camera_name, detectionImageType)

    #Make text for annotation file
    image_width = responses[0].width
    image_height = responses[0].height
    annotation_file = detectionsToYOLO(detections, image_width, image_height)

    for idx, response in enumerate(responses):
        imagepath = basepath
        labelspath = basepath + "labels/"
        if(idx == 0):
            imagepath = imagepath + "rgb/"
        else:
            imagepath = imagepath + "seg/"
        filename = "airsim_pic_" + str(filecounter)
        imagepath = imagepath + filename
        labelspath = labelspath + filename

        # write annotation file
        with open(labelspath + ".txt", "w") as f:
            f.write(annotation_file)

        writeImageFromResponse(response, imagepath)

    filecounter = filecounter + 1

def doOrbit(altitude, radius, steps, center):
    z = altitude
    xy_length = math.sqrt(radius**2 - altitude**2)
    pitch = math.asin(altitude/radius)
    for i in range(steps):
        rel_angle = (i*360/steps)*math.pi/180
        x = math.cos(math.pi + rel_angle)
        y = math.sin(math.pi + rel_angle)
        outVector = airsim.Vector3r(x*xy_length, y*xy_length, z)
        client.simSetVehiclePose(airsim.Pose(outVector, airsim.to_quaternion(pitch,random.random()*2*math.pi,rel_angle)), True)
        takePicture(basepath)

def doBox(altitude, size, stepstotal, center):
    steps = int(stepstotal/4)
    dist_to_center = math.sqrt(altitude**2 + size**2)
    pitch = math.asin(altitude/dist_to_center)

    # x: -size, y: size to -size, yaw: 0
    pos_from = center + airsim.Vector3r(-size, size, altitude)
    pos_to = center + airsim.Vector3r(-size, -size, altitude)
    doLine(pos_from, pos_to, pitch, 0, steps)
    
    # x: -size to size, y: -size, yaw: pi/2
    pos_from = center + airsim.Vector3r(-size, -size, altitude)
    pos_to = center + airsim.Vector3r(size, -size, altitude)
    doLine(pos_from, pos_to, pitch, math.pi/2, steps)
    
    # x: size, y: -size to size, yaw: pi
    pos_from = center + airsim.Vector3r(size, -size, altitude)
    pos_to = center + airsim.Vector3r(size, size, altitude)
    doLine(pos_from, pos_to, pitch, math.pi, steps)
    
    # x: size to -size, y: size, yaw: 3pi/2
    pos_from = center + airsim.Vector3r(size, size, altitude)
    pos_to = center + airsim.Vector3r(-size, size, altitude)
    doLine(pos_from, pos_to, pitch, 3*math.pi/2, steps)

def doLine(pos_from, pos_to, pitch, yaw, steps):
    pos_iter = (pos_to - pos_from)/(steps)
    for i in range(steps):
        pos = pos_from + pos_iter*i
        rot = airsim.to_quaternion(pitch,random.random()*2*math.pi,yaw)
        client.simSetVehiclePose(airsim.Pose(pos, rot), True)
        takePicture(basepath)
    


def procedureBox(steps, distance_min, distance_max, distance_step, altitude_min, altitude_step):
    distance_range = range(distance_min, distance_max + 1, distance_step)

    total = 0
    for s in distance_range:
        for z in range(altitude_min, s + 1, altitude_step):
            total += 1

    count = 1
    for s in distance_range:
        for z in range(altitude_min, s + 1, altitude_step):
            print("Box:" + str(count) + " of " + str(total))
            doBox(-z, s, steps, airsim.Vector3r(0,0,0))
            count += 1

def procedureOrbit(steps, distance_min, distance_max, distance_step, altitude_min, altitude_step):
    distance_range = range(distance_min, distance_max + 1, distance_step)

    total = 0
    for s in distance_range:
        for z in range(altitude_min, s + 1, altitude_step):
            total += 1

    count = 1
    for r in distance_range:
        for z in range(altitude_min, r + 1, altitude_step):
            print("Orbit:" + str(count) + " of " + str(total))
            doOrbit(-z,r,steps,0)
            count += 1

newClient()
procedureBox(120, 15, 25, 1, 0, 30, 1)
procedureOrbit(120, 15, 25, 1, 0, 30, 1)

quit(0)