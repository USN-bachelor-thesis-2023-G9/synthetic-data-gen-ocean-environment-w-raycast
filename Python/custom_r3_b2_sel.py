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

def newClient():
    # establish connection to airsim misc
    global client
    client = airsim.VehicleClient()
    client.confirmConnection()

    while not client.simIsPause():
        pass
    client.simPause(False);

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

def takePicture(basepath):
    global filecounter
    responses = client.simGetImages(requests)

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
        #with open(labelspath + ".txt", "w") as f:
        #    f.write(annotation_file)

        writeImageFromResponse(response, imagepath)

        #print(imagepath, " written to disk")

    filecounter = filecounter + 1
    while not client.simIsPause():
        pass
    client.simPause(False);

def doOrbit(altitude,radius, pitch, steps, center):
    global imgidx
    z = altitude
    dist_to_center = math.sqrt(altitude**2 + radius**2)
    pitch = math.asin(altitude/dist_to_center) + pitch
    for i in range(steps):
        rel_angle = (i*360/steps)*math.pi/180
        x = math.cos(math.pi + rel_angle)
        y = math.sin(math.pi + rel_angle)
        outVector = airsim.Vector3r(x*radius, y*radius, z)
        if selmode == False or selmode == True and len(selected) > 0 and imgidx == selected[0]:
            client.simSetVehiclePose(airsim.Pose(outVector, airsim.to_quaternion(pitch,0,rel_angle)), True)
            takePicture(basepath)
            if selmode == True:
                selected.pop(0)
                if len(selected) <= 0:
                    quit(0)
        imgidx += 1

def doBox(altitude, size, pitch, stepstotal, center):
    steps = int(stepstotal/4)
    dist_to_center = math.sqrt(altitude**2 + size**2)
    pitch = math.asin(altitude/dist_to_center) + pitch

    # x: -size, y: size to -size, yaw: 0
    rot = airsim.to_quaternion(pitch,0,0)
    pos_from = center + airsim.Vector3r(-size, size, altitude)
    pos_to = center + airsim.Vector3r(-size, -size, altitude)
    doLine(pos_from, pos_to, rot, steps)
    
    # x: -size to size, y: -size, yaw: pi/2
    rot = airsim.to_quaternion(pitch,0,math.pi/2)
    pos_from = center + airsim.Vector3r(-size, -size, altitude)
    pos_to = center + airsim.Vector3r(size, -size, altitude)
    doLine(pos_from, pos_to, rot, steps)
    
    # x: size, y: -size to size, yaw: pi
    rot = airsim.to_quaternion(pitch,0,math.pi)
    pos_from = center + airsim.Vector3r(size, -size, altitude)
    pos_to = center + airsim.Vector3r(size, size, altitude)
    doLine(pos_from, pos_to, rot, steps)
    
    # x: size to -size, y: size, yaw: 3pi/2
    rot = airsim.to_quaternion(pitch,0,3*math.pi/2)
    pos_from = center + airsim.Vector3r(size, size, altitude)
    pos_to = center + airsim.Vector3r(-size, size, altitude)
    doLine(pos_from, pos_to, rot, steps)

def doLine(pos_from, pos_to, rot, steps):
    global imgidx
    pos_iter = (pos_to - pos_from)/(steps - 1)
    for i in range(steps):
        pos = pos_from + pos_iter*i
        if selmode == False or selmode == True and len(selected) > 0 and imgidx == selected[0]:
            client.simSetVehiclePose(airsim.Pose(pos, rot), True)
            takePicture(basepath)
            if selmode == True:
                selected.pop(0)
                if len(selected) <= 0:
                    quit(0)
        imgidx += 1
    


def procedureBox(steps, distance_min, distance_max, distance_step, altitude_min, altitude_max, altitude_step, pitch_min, pitch_max, pitch_steps):
    distance_range = range(distance_min, distance_max + 1, distance_step)

    total = 0
    for a in range(pitch_steps):
        for s in distance_range:
            for z in range(altitude_min, altitude_max + 1, altitude_step):
                total += 1

    count = 1
    for a in range(pitch_steps):
        pitch = pitch_min + a*(pitch_max - pitch_min)/(pitch_steps - 1)
        for s in distance_range:
            for z in range(altitude_min, altitude_max + 1, altitude_step):
                print("Box:" + str(count) + " of " + str(total))
                doBox(-z, s, pitch, steps, airsim.Vector3r(0,0,0))
                count += 1

def procedureOrbit(steps, distance_min, distance_max, distance_step, altitude_min, altitude_max, altitude_step, pitch_min, pitch_max, pitch_steps):
    distance_range = range(distance_min, distance_max + 1, distance_step)

    total = 0
    for a in range(pitch_steps):
        for s in distance_range:
            for z in range(altitude_min, altitude_max + 1, altitude_step):
                total += 1

    count = 1
    for a in range(pitch_steps):
        pitch = pitch_min + a*(pitch_max - pitch_min)/(pitch_steps - 1)
        for r in distance_range:
            for z in range(altitude_min, altitude_max + 1, altitude_step):
                print("Orbit:" + str(count) + " of " + str(total))
                doOrbit(-z,r,pitch,steps,0)
                count += 1

pitch_adj = math.pi/8

#If True telect to generate specific picture indexes in the sequence
selmode = False
selected = [ ]
if selmode == True:
    if len(selected) == 0:
        quit(0)
    selected.sort()
    print(selected)

imgidx = 0

# Levelssawe
input("Change to main level")
newClient()
procedureBox(40, 20, 30, 10, 20, 30, 10, -pitch_adj, pitch_adj, 4)
procedureOrbit(40, 20, 30, 10, 20, 30, 10, -pitch_adj, pitch_adj, 4)

pitch_adj = math.pi/12
# level 0-3
for i in range(4):
    input("Change to level " + str(i))
    newClient()
    procedureBox(20, 2, 2, 1, 2, 2, 1, -pitch_adj, pitch_adj, 4)
    procedureOrbit(20, 2, 2, 1, 2, 2, 1, -pitch_adj, pitch_adj, 4)

# level 4-7
for i in range(4):
    input("Change to level " + str(i + 4))
    newClient()
    procedureBox(20, 6, 6, 1, 2, 2, 1, -pitch_adj, pitch_adj, 4)
    procedureOrbit(20, 6, 6, 1, 2, 2, 1, -pitch_adj, pitch_adj, 4)


# level 8-9
for i in range(2):
    input("Change to level " + str(i + 8))
    newClient()
    procedureBox(40, 8, 12, 4, 2, 6, 4, -pitch_adj, pitch_adj, 4)
    procedureOrbit(40, 8, 12, 4, 2, 6, 4, -pitch_adj, pitch_adj, 4)

quit(0)


