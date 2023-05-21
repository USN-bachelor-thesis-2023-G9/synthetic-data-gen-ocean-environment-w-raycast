import airsim
import cv2
import os
import numpy as np
import time
import math

# camera to use
camera_name = "0"

# list of the image types that are wanted
requests = [
    airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False),
    airsim.ImageRequest(camera_name, airsim.ImageType.Segmentation, False, False)]

detectionImageType = airsim.ImageType.Scene;

# orbit variables
speed = 2 # full speed of drone (m/s)
radius = 25 # radius of the orbit
center = airsim.Vector3r(0,0,0)
z = -5.5 # altitude of orbit

# establish connection to airsim drone
client = airsim.MultirotorClient()
client.confirmConnection()

# establish connection to airsim misc
vclient = airsim.VehicleClient()
vclient.confirmConnection()

# takeoff
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

# Information about coordinates:
#  To transform unreal engine coordinates to airsim coordinates
#  you need to divide all coordinates by 100 and change the sign of z
#  example unreal coordinate [-2500, 0, 200] becomes [-25, 0, -2]
#
#  Move drone to unreal coord [-2500, 0, 200] at speed 2 m/s
client.moveToPositionAsync(-25, 0, -2, 2).join()

client.simSetDetectionFilterRadius(camera_name, airsim.ImageType.Scene, 80 * 100) # in [cm]
client.simAddDetectionFilterMeshName(camera_name, airsim.ImageType.Scene, "Cylinder") 

time.sleep(10)

print("ramping up to speed...")
start_angle = None
        
# ramp up time
ramptime = radius / 10
start_time = time.time()

counter = 0
while True:
    """
        Title: orbit.py
        Authors: 
            Lovett, C
            Shah, S
            Madaan, R
            Chaves, T
        Date: 2020
        Code version: commit 6ffb7a5
        Availability: https://github.com/microsoft/AirSim/blob/6ffb7a528e5e185d14444afdb44f8ad7ba4917e4/PythonClient/multirotor/orbit.py
    """
    #START OF COPIED SECTION
    # ramp up to full speed in smooth increments so we don"t start too aggressively.
    now = time.time()
    currentspeed = speed
    diff = now - start_time
    if diff < ramptime:
        currentspeed = speed * diff / ramptime
    elif ramptime > 0:
        print("reached full speed...")
        ramptime = 0
                
    lookahead_angle = currentspeed / radius            

    # compute current angle
    pos = client.getMultirotorState().kinematics_estimated.position
    dx = pos.x_val - center.x_val
    dy = pos.y_val - center.y_val
    actual_radius = math.sqrt((dx*dx) + (dy*dy))
    angle_to_center = math.atan2(dy, dx)

    camera_heading = (angle_to_center - math.pi) * 180 / math.pi 

    # compute lookahead
    lookahead_x = center.x_val + radius * math.cos(angle_to_center + lookahead_angle)
    lookahead_y = center.y_val + radius * math.sin(angle_to_center + lookahead_angle)

    vx = lookahead_x - pos.x_val
    vy = lookahead_y - pos.y_val
    #END OF COPIED SECTION

    # set segmentation ids
    vclient.simSetSegmentationObjectID("Cone.*", 1, True)
    vclient.simSetSegmentationObjectID("Cube.*", 2, True)
    vclient.simSetSegmentationObjectID("Cylinder.*", 3, True)
    vclient.simSetSegmentationObjectID("Sphere.*", 4, True)

    responses = client.simGetImages(requests)

    #Get object detections on main camera
    detections = client.simGetDetections(camera_name, detectionImageType)

    #Make text for annotation file
    annotation_file = ""
    image_width = responses[0].width
    image_height = responses[0].height
    if detections:
        for idx, detection in enumerate(detections):
            if(idx != 0):
                annotation_file = annotation_file + "\n"
            norm_min_x = detection.box2D.min.x_val/image_width
            norm_min_y = detection.box2D.min.y_val/image_height
            width = (detection.box2D.max.x_val - detection.box2D.min.x_val)/image_width
            height = (detection.box2D.max.y_val - detection.box2D.min.y_val)/image_height
            center_x = norm_min_x + width/2
            center_y = norm_min_y + height/2

            annotation_file = annotation_file + f"0 {center_x} {center_y} {width} {height}"


    for idx, response in enumerate(responses):
        basepath = "c:/temp/"
        imagepath = basepath
        if(idx == 0):
            imagepath = imagepath + "rgb/"
        else:
            imagepath = imagepath + "seg/"
        filename = "airsim_pic_" + str(counter)
        imagepath = imagepath + filename
        labelspath = basepath + "labels/" + filename

         # write annotation file
        with open(labelspath + ".txt", "w") as f:
            f.write(annotation_file)

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

        print(imagepath, " written to disk")
            
    client.moveByVelocityZAsync(vx, vy, z, 1, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, camera_heading))
    counter += 1

