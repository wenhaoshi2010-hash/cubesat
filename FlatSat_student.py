"""
The Python code you will write for this module should read
acceleration data from the IMU. When a reading comes in that surpasses
an acceleration threshold (indicating a shake), your Pi should pause,
trigger the camera to take a picture, then save the image with a
descriptive filename. You may use GitHub to upload your images automatically,
but for this activity it is not required.

The provided functions are only for reference, you do not need to use them. 
You will need to complete the take_photo() function and configure the VARIABLES section
"""

#AUTHOR: 
#DATE:

# import libraries
import time
import math
import board
import cv2
import numpy as np
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL
from git import Repo
from picamera2 import Picamera2

# VARIABLES
THRESHOLD = 20
REPO_PATH = "/home/pi/cubesat"
FOLDER_PATH = "Images"

# imu and camera initialization
i2c = board.I2C()
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)
picam2 = Picamera2()


def git_push():
    try:
        repo = Repo(REPO_PATH)
        origin = repo.remote('origin')
        origin.pull()
        repo.git.add(REPO_PATH + FOLDER_PATH)
        repo.index.commit('New Photo')
        origin.push()
    except:
        print("Couldn't upload to git")


def img_gen(name):
    t = time.strftime("_%H%M%S")
    imgname = (f'{name}{t}.jpg')
    return imgname


def generate_depth_and_flatness(image_list):

    print("Generating depth map using all images...")

    disparity_maps = []
    stereo = cv2.StereoBM_create(numDisparities=64, blockSize=15)

    # loop through image pairs
    for i in range(len(image_list) - 1):

        img1 = cv2.imread(image_list[i], 0)
        img2 = cv2.imread(image_list[i + 1], 0)

        disparity = stereo.compute(img1, img2)

        disparity_maps.append(disparity.astype(np.float32))

    # average all disparity maps
    avg_disparity = np.mean(disparity_maps, axis=0)

    cv2.imwrite("depth_map.png", avg_disparity)

    print("Generating flatness map...")

    gradient = np.gradient(avg_disparity)
    flatness = np.sqrt(gradient[0]**2 + gradient[1]**2)

    cv2.imwrite("flatness_map.png", flatness)

def take_photo():

    print("Monitoring IMU...")

    picam2.configure(picam2.create_still_configuration())
    picam2.start()

    while True:

        accelx, accely, accelz = accel_gyro.acceleration

        total_accel = math.sqrt(accelx**2 + accely**2 + accelz**2)

        if total_accel > THRESHOLD:

            print(f"Shake detected! Accel: {total_accel:.2f}")

            # 1 second delay before photos
            time.sleep(1)

            images = []

            # capture 5 images
            for i in range(5):

                name = f"terrain_{i}"
                filename = img_gen(name)

                picam2.capture_file(filename)
                print(f"Saved photo: {filename}")

                images.append(filename)

                time.sleep(1)

            # generate terrain maps using first two images
            generate_depth_and_flatness(images)

            time.sleep(2)

if __name__ == '__main__':

    main()





