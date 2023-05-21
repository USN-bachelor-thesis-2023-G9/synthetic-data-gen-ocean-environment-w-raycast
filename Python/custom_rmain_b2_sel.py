import random
import airsim
import cv2
import os
import numpy as np
import time
import math

"""
    Title: create_ir_segmentation_map.py
    Authors: 
        Gyde, N
        Shah, S
    Date: 2019
    Code version: commit 83e3c87
    Availability: https://github.com/microsoft/AirSim/blob/83e3c87ded1b59c1a0f68bcddc14ba1b5ff724c5/PythonClient/computer_vision/capture_ir_segmentation.py
"""
#START OF COPIED SECTION
def radiance(absoluteTemperature, emissivity, dx=0.01, response=None):
    """
    title::
        radiance

    description::
        Calculates radiance and integrated radiance over a bandpass of 8 to 14
        microns, given temperature and emissivity, using Planck"s Law.

    inputs::
        absoluteTemperature
            temperture of object in [K]

            either a single temperature or a numpy
            array of temperatures, of shape (temperatures.shape[0], 1)
        emissivity
            average emissivity (number between 0 and 1 representing the
            efficiency with which it emits radiation; if 1, it is an ideal 
            blackbody) of object over the bandpass

            either a single emissivity or a numpy array of emissivities, of 
            shape (emissivities.shape[0], 1)
        dx
            discrete spacing between the wavelengths for evaluation of
            radiance and integration [default is 0.1]
        response
            optional response of the camera over the bandpass of 8 to 14 
            microns [default is None, for no response provided]
    
    returns::
        radiance
            discrete spectrum of radiance over bandpass
        integratedRadiance
            integration of radiance spectrum over bandpass (to simulate
            the readout from a sensor)

    author::
        Elizabeth Bondi
    """
    wavelength = np.arange(8,14,dx)
    c1 = 1.19104e8 # (2 * 6.62607*10^-34 [Js] * 
                   # (2.99792458 * 10^14 [micron/s])^2 * 10^12 to convert 
                   # denominator from microns^3 to microns * m^2)
    c2 = 1.43879e4 # (hc/k) [micron * K]
    if response is not None:
        radiance = response * emissivity * (c1 / ((wavelength**5) * \
                   (np.exp(c2 / (wavelength * absoluteTemperature )) - 1)))
    else:
        radiance = emissivity * (c1 / ((wavelength**5) * (np.exp(c2 / \
                   (wavelength * absoluteTemperature )) - 1)))
    if absoluteTemperature.ndim > 1:
        return radiance, np.trapz(radiance, dx=dx, axis=1)
    else:
        return radiance, np.trapz(radiance, dx=dx)

def get_new_temp_emiss_from_radiance(tempEmissivity, response):
    """
    title::
        get_new_temp_emiss_from_radiance

    description::
        Transform tempEmissivity from [objectName, temperature, emissivity]
        to [objectName, "radiance"] using radiance calculation above.

    input::
        tempEmissivity
            numpy array containing the temperature and emissivity of each
            object (e.g., each row has: [objectName, temperature, emissivity])
        response
            camera response (same input as radiance, set to None if lacking
            this information)

    returns::
        tempEmissivityNew
            tempEmissivity, now with [objectName, "radiance"]; note that 
            integrated radiance (L) is divided by the maximum and multiplied 
            by 255 in order to simulate an 8 bit digital count observed by the 
            thermal sensor, since radiance and digital count are linearly 
            related, so it"s [objectName, simulated thermal digital count]

    author::
        Elizabeth Bondi
    """
    numObjects = tempEmissivity.shape[0]

    L = radiance(tempEmissivity[:,1].reshape((-1,1)).astype(np.float64), 
                 tempEmissivity[:,2].reshape((-1,1)).astype(np.float64), 
                 response=response)[1].flatten() 
    L = ((L / L.max()) * 255).astype(np.uint8)

    tempEmissivityNew = np.hstack((
        tempEmissivity[:,0].reshape((numObjects,1)), 
        L.reshape((numObjects,1))))

    return tempEmissivityNew

def set_segmentation_ids(segIdDict, tempEmissivityNew, client):
    """
    title::
        set_segmentation_ids

    description::
        Set stencil IDs in environment so that stencil IDs correspond to
        simulated thermal digital counts (e.g., if elephant has a simulated
        digital count of 219, set stencil ID to 219).

    input::
        segIdDict
            dictionary mapping environment object names to the object names in
            the first column of tempEmissivityNew 
        tempEmissivityNew
            numpy array containing object names and corresponding simulated
            thermal digital count
        client
            connection to AirSim (e.g., client = MultirotorClient() for UAV)

    author::
        Elizabeth Bondi
    """

    #First set everything to 0.
    success = client.simSetSegmentationObjectID("[\w]*", 0, True);
    if not success:
        print("There was a problem setting all segmentation object IDs to 0. ")
        quit(1)

    #Next set all objects of interest provided to corresponding object IDs
    #segIdDict values MUST match tempEmissivityNew labels.
    for key in segIdDict:
        objectID = int(tempEmissivityNew[np.where(tempEmissivityNew == \
                                                     segIdDict[key])[0],1][0])

        success = client.simSetSegmentationObjectID("[\w]*"+key+"[\w]*", 
                                                    objectID, True);
        if not success:
            print("There was a problem setting {0} segmentation object ID to {1!s}, or no {0} was found.".format(key, objectID))
            
    time.sleep(0.1)
#END OF COPIED SECTION

def update_segmentation_map():
    #Calculate radiance.
    tempEmissivityNew = get_new_temp_emiss_from_radiance(tempEmissivity, 
                                                         response)
    #Set IDs in AirSim environment.
    set_segmentation_ids(segIdDict, tempEmissivityNew, client)

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

    while not client.simIsPause():
        pass
    client.simPause(False);

    responses = client.simGetImages(requests)

    for idx, response in enumerate(responses):
        imagepath = basepath
        labelspath = basepath + "labels/"
        if(idx == 0):
            imagepath = imagepath + "rgb/"
        else:
            imagepath = imagepath + "infra/"
        filename = "airsim_pic_" + str(filecounter)
        imagepath = imagepath + filename
        labelspath = labelspath + filename

        writeImageFromResponse(response, imagepath)
    

    #fetch image metadata
    pose = client.simGetCameraInfo(camera_name).pose
    position = pose.position - center
    quat = pose.orientation
    metadata = f"{filecounter},{position.x_val},{position.y_val},{position.z_val},{quat.x_val},{quat.y_val},{quat.z_val},{quat.w_val}\n"
    metafile.write(metadata)
    filecounter = filecounter + 1
    while not client.simIsPause():
        pass
    if debug:
        input("Debug: Press enter to continue...")
    client.simPause(False);

def random_pose_in_range(center, minPos, maxPos, pitch_min=0, pitch_max=0, yaw_factor=0):
    x = random.uniform(minPos.x_val, maxPos.x_val)
    y = random.uniform(minPos.y_val, maxPos.y_val)
    z = random.uniform(minPos.z_val, maxPos.z_val)
    newPos = airsim.Vector3r(x, y, z)

    vCenterToPos = newPos - center
    angle_to_center = math.atan2(vCenterToPos.y_val, vCenterToPos.x_val)
    yaw = (angle_to_center - math.pi)
    pitch = math.asin(vCenterToPos.z_val/vCenterToPos.get_length())
    xyDiff = airsim.Vector3r(vCenterToPos.x_val, vCenterToPos.y_val, 0)
    rot = airsim.to_quaternion(pitch,0,yaw)

    return airsim.Pose(newPos, rot)

startTime = time.time()

segIdDict = {
            "BP_Ocean":"water",
            "boat_Speedboat":"boat",
            "human_Brian_blueprint2":"human",
            "human_Lewis_blueprint_2":"human",
            "human_Louise_blueprint2":"human"
            }
    
#Choose temperature values for winter or summer.
#"""
tempEmissivity = np.array([["human",308,0.985], 
                              ["water",281,0.96],
                              ["boat",282,0.8]])
#"""

#Read camera response.
response = None
camResponseFile = "camera_response.npy"
try:
    np.load(camResponseFile)
except:
    print("{} not found. Using default response.".format(camResponseFile))

# camera to use
camera_name = "0"

basepath = "c:/temp/"
filecounter = 0

# list of the image types that are wanted
requests = [
    airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False),
    airsim.ImageRequest(camera_name, airsim.ImageType.Infrared, False, False)
    ]

detectionImageType = airsim.ImageType.Scene;

pitch_adj = math.pi/8

debug = False

imgidx = 0

center = airsim.Vector3r(640.3, 119.4, 0)

minPos = airsim.Vector3r(565.3, -13.93, -50)
maxPos = airsim.Vector3r(715.3, 252.73, -70)

input("Press enter to start...")
with open(basepath + "meta/camera/metadata_pose.csv", "w") as metafile:
    metafile.write("image_id,x,y,z,quaternion_x,quaternion_y,quaternion_z,quaternion_w\n")

    newClient()
    for i in range(1):
        pose = random_pose_in_range(center, minPos, maxPos)
        client.simSetVehiclePose(pose, True)
        update_segmentation_map()
        takePicture(basepath)

endTime = time.time()

duration = endTime - startTime

print(f"Generation took {duration}")
quit(0)


