import os
import csv

import cv2
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator

import torch
from PIL import Image
import sys
import shutil
import numpy as np
import datetime
import pytz
from pytz import timezone
from extract_images_of_event import extract_all_images
import config
import time

np.set_printoptions(threshold=sys.maxsize)
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH")


# save detected accident to CSV file
def save_accident_to_csv(image_name, camera_name, rosbag_name, csv_name):
    with open(csv_name, 'a', newline='') as file:
        writer = csv.writer(file)

        # convert UTC timestamp to datetime object
        parts = image_name.split("_")
        timestamp = float(parts[0])
        milliseconds = parts[1]
        dt_utc = datetime.datetime.utcfromtimestamp(timestamp)

        # calculate time difference between time zones depending on daylight saving time
        timeZone = pytz.timezone("Europe/Berlin")
        if is_dst(dt_utc, timeZone):
            time_diff = 2
        else:
            time_diff = 1
        time_delta = datetime.timedelta(hours=time_diff)
        time_delta_obj = datetime.timezone(time_delta)

        # convert UTC timestamp to local time
        dt_local = dt_utc.astimezone(time_delta_obj)

        row = [dt_local.year, dt_local.month, dt_local.day, dt_local.hour, dt_local.minute, dt_local.second,
               milliseconds[:3], camera_name, rosbag_name, image_name]
        writer.writerow(row)


# check whether given date is in daylight saving time
def is_dst(dt, timeZone):
    aware_dt = timeZone.localize(dt)
    return aware_dt.dst() != datetime.timedelta(0, 0)


def get_subfolders(folder_path):
    subfolders = [f.path for f in os.scandir(folder_path) if f.is_dir()]
    return subfolders


def remove_folder(folder_path):
    try:
        shutil.rmtree(folder_path)
    except OSError as e:
        print(f"Error: {e}")


# learning-based accident detection
def accident_detection_ml(input_path, output_path, extract_images):
    # extract images of all cameras of input rosbag
    images_path = extract_all_images(input_path, output_path)  # path with /images

    # initialize variables
    subfolders = get_subfolders(images_path)
    number_cameras = len(subfolders)
    rosbag_name = os.path.split(output_path)[-1]
    timestamps = []  # stores list for each camera containing the timestamps of the recorded frames
    accidents_timestamps = []  # stores list for each camera containing timestamps of detected accidents
    accidents = []  # stores array for each camera containing 1 for detected accident at specific timestamp, 0 otherwise

    # figure out maximum amount of recorded frames of all cameras available in rosbag
    number_frames_all_cameras = []
    for i, camera_path in enumerate(subfolders):
        number_of_frames = len(os.listdir(camera_path))
        number_frames_all_cameras.append(number_of_frames)
    max_number_frames = max(number_frames_all_cameras)

    for i in range(number_cameras):
        accidents.append(np.zeros(max_number_frames, dtype=int))

    # create CSV file for storing detected accidents if not already existing
    csv_name = os.path.join(os.path.dirname(output_path.rstrip(os.path.sep)), "accidents_ml.csv")
    if not os.path.exists(csv_name):
        with open(csv_name, 'a', newline='') as file:
            writer = csv.writer(file)
            row_names = ['year', 'month', 'day', 'hour', 'minute', 'second', 'millisecond', 'camera', 'rosbag_name',
                         'image_name', 'accident']
            writer.writerow(row_names)

    # create CSV file with all accident timestamps for testing
    # csv_all_name = os.path.join(os.path.dirname(output_path.rstrip(os.path.sep)), "accidents_ml_all.csv")
    # if not os.path.exists(csv_name):
    #    with open(csv_name, 'a', newline='') as file:
    #        writer = csv.writer(file)
    #        row_names = ['year', 'month', 'day', 'hour', 'minute', 'second', 'millisecond', 'camera', 'rosbag_name',
    #                     'image_name', 'accident']
    #        writer.writerow(row_names)

    # load accident detection model
    model = YOLO(config.model_path)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)

    # iterate over all cameras available in rosbag
    for i, camera_path in enumerate(subfolders):
        camera_name = os.path.basename(camera_path)

        # initialize variables
        accidents_timestamps.append([])
        timestamps.append([])
        j = 0  # index of accident used to access information in associated arrays/lists belonging
        images = os.listdir(camera_path)
        images = sorted(images)

        # iterate over all images extracted for a camera
        for image_name in images:
            image_path = os.path.join(camera_path, image_name)
            if image_name.endswith(".png") or image_name.endswith(".jpg") or image_name.endswith(".jpeg"):
                timestamps[i].append(os.path.basename(image_path))
                detection_path = output_path + "/detections"

                # perform forward pass
                results = model.predict(image_path, conf=config.confidence_threshold, imgsz=config.image_resolution, project=detection_path, save_txt=True)
                # handle results of forward pass
                for result in results:
                    if len(result.boxes) == 0:
                        continue

                    # prepare image creation if argument is set
                    image = np.asarray(Image.open(image_path))
                    if extract_images:
                        annotator = Annotator(image)

                    # iterate over all detected accidents in an image
                    for detection in result:
                        accidents[i][j] += 1
                        box = detection.boxes[0]
                        conf = detection.boxes.conf[0].item()
                        print(conf)
                        if extract_images:
                            conf = "{:.2f}".format(conf)
                            annotator.box_label(box.xyxy[0], "accident " + str(conf), (127, 0, 255))

                    # create image if argument is set
                    if extract_images:
                        image = annotator.result()
                        # convert BGR image to RGB
                        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                        # save image to output_path
                        cv2.imwrite(os.path.join(output_path, image_name), image)
                j += 1

        # initialize variables
        length = len(accidents[i])
        k = 0  # counts amount of jumps that have been made to correct the lower frequence of camera s40 near

        # check whether accident has been detected in 3 consecutive frames taken by the same camera (condition does not
        # have to be met if second last or last frame of rosbag)
        if max_number_frames >= 3:
            # check if accident is detected in first 3 frames (usually interested in change from 0 to 1 as that means
            # that there is now an accident that hasn't been there before, for first 3 frames of an accident this change
            # from 0 to 1 is not necessary to be handled as anc accident
            if accidents[i][0] > 0 and accidents[i][1] > 0 and accidents[i][2] > 0:
                accidents_timestamps[i].append(timestamps[i][0])
            else:
                accidents[i][0] = 0

            sequence = 0  # flag whether timestamp j is part of a series of accident detections
            for j in range(length - 3):
                # if camera = s040_camera_basler_north_near_16mm: correct timestamps camera has a lower frequency
                if camera_name == 's040_camera_basler_north_near_16mm' and j % (248 + k * 250) == 0:
                    k += 1
                    continue

                # look for sequence of detected accidents of a length 3
                if accidents[i][j] == 0:
                    sequence = 0
                if accidents[i][j] == 0 and accidents[i][j + 1] > 0 and accidents[i][j + 2] > 0 and accidents[i][
                    j + 3] > 0:
                    accidents_timestamps[i].append(timestamps[i][j + 1])
                    sequence = 1
                elif sequence != 1 and (
                        accidents[i][j + 1] == 0 or accidents[i][j + 2] == 0 or accidents[i][j + 3] == 0):
                    accidents[i][j + 1] = 0

            # look for sequence of detected accidents for last three timestamps
            if accidents[i][length - 3] == 0 and accidents[i][length - 2] > 0 and accidents[i][length - 1] > 0:
                accidents_timestamps[i].append(timestamps[i][length - 2])
            else:
                accidents[i][length - 2] = 0
            if accidents[i][length - 2] == 0 and accidents[i][length - 1] > 0:
                accidents_timestamps[i].append(timestamps[i][length - 1])
        else:
            # look for sequence of detected accidents if less than 3 images
            for j in range(length):
                if accidents[i][j] > 0:
                    accidents_timestamps[i].append(timestamps[i][j])

        # save accident information in CSV file and save image of accident
        for acc_ts in accidents_timestamps[i]:
            save_accident_to_csv(acc_ts, camera_name, rosbag_name, csv_name)
            source_path = os.path.join(output_path, "images", camera_name, acc_ts)
            destination_path = os.path.join(output_path, acc_ts)
            shutil.move(source_path, destination_path)

    # fuses "cleaned-up" accident detection results of all available cameras
    fused_accidents = np.zeros(max_number_frames, dtype=int)
    for i in range(max_number_frames):
        for j in range(len(accidents)):
            fused_accidents[i] += accidents[j][i]

    # calculate number of total accidents
    accident_counter = 0
    for i in range(fused_accidents.size - 1):
        # save timestamp in CSV file with all accident timestamps for testing
        # if i < len(timestamps[0]) and fused_accidents[i]>0:
        #    save_accident_to_csv(timestamps[0][i], "", rosbag_name, csv_all_name)
        if fused_accidents[i] == 0 and fused_accidents[i + 1] > 0:
            accident_counter += 1

    # remove extracted images (were only extracted from rosbag to feed them into model)
    shutil.rmtree(os.path.join(output_path, "images"))

    return accident_counter
