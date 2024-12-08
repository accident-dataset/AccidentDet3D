import os
import csv
import rosbag
import numpy as np
import subprocess as sub


# Description
# This module provides the implementation for extracting the images for a detected event

# get parameter for image extraction
def get_params_for_img_extraction(input_path):
    input_topics = []
    sensor_ids = []
    msg_count = 0

    # check for which cameras there are recordings in the rosbag
    with rosbag.Bag(input_path, 'r') as bag:
        for topic_name, topic_infos in bag.get_type_and_topic_info().topics.items():
            if 'image_raw/video' in topic_name:
                input_topics.append(topic_name)

                if '/s40/n/cam/near/' in topic_name:
                    sensor_ids.append('s040_camera_basler_north_near_16mm')
                elif '/s40/n/cam/far/' in topic_name:
                    sensor_ids.append('s040_camera_basler_north_far_50mm')
                elif '/s50/s/cam/near/' in topic_name:
                    sensor_ids.append('s050_camera_basler_south_near_16mm')
                elif '/s50/s/cam/far/' in topic_name:
                    sensor_ids.append('s050_camera_basler_south_far_50mm')
                else:
                    print('Could not find corresponding sensor ID.')
                msg_count = topic_infos.message_count
    return input_topics, sensor_ids, msg_count


# get timestamps where an event is detected (type of event is specified as argument)
def get_timestamp_of_event(actor, event):
    indices = np.asarray(actor[event] != 0).nonzero()
    event_indices = [indices[0][0]]
    for i in range(len(indices[0]) - 1):
        if indices[0][i] != indices[0][i + 1] - 1:
            event_indices.append(indices[0][i + 1])

    time_stamps_events = []
    for event_id in event_indices:
        time_stamps_events.append(actor['time_stamp'][event_id])
    return time_stamps_events


# launch rosbag that extracts images for given time stamp
def roslaunch_bag_extractor(time_stamp, input_path, output_path, input_topics, sensor_ids, msg_count):
    path_current_script = os.path.dirname(os.path.abspath(__file__))
    path_launch_file_relative = '../../bag_extractor/launch/image_extractor_cpp.launch'
    path_launch_file_absolute = os.path.join(path_current_script, path_launch_file_relative)

    # set ID of first and last ROS message for which images should be extracted
    start_frame = 0
    end_frame = -1
    if time_stamp != -1:  # if != -1:  that end_id is specified
        frame_rate = 25
        image_extraction_range = 4.8  # in seconds
        img_extr_range_frames = image_extraction_range * frame_rate  # as 25 frames per second
        statistic_image_offset_frames = 20 * frame_rate  # as there is an offset of 20 seconds between the timestamps of the rosbag/JSON files and the images
        start_frame = max(0, round(time_stamp / 0.04, 0) - img_extr_range_frames - statistic_image_offset_frames)
        end_frame = round(time_stamp / 0.04, 0) + img_extr_range_frames - statistic_image_offset_frames

        if end_frame <= 0:
            print("WARNING! Images for detected event in rosbag " + str(
                os.path.basename(input_path) + " at timestamp " + str(
                    time_stamp) + " can not be extracted due to the offset of 20 seconds between the raw data and the images."))
            return

    if end_frame >= msg_count:
        end_frame = msg_count - 1

    if len(sensor_ids) != len(input_topics):
        print("ERROR! Number of sensor ids and input ros topics does not match.")
        return

    # extract images for all cameras available in rosbag
    for i in range(len(sensor_ids)):
        temp_path = output_path + str(sensor_ids[i])
        if not os.path.exists(temp_path):
            os.mkdir(temp_path)
        temp_path += '/'
        roslaunch_cmd = ['roslaunch', path_launch_file_absolute, 'bag_file_path:=' + str(input_path),
                         'sensor_id:=' + str(sensor_ids[i]), 'input_ros_topic:=' + str(input_topics[i]),
                         'decoded_image_type:=jpg', 'decoded_image_output_path:=' + str(temp_path), 'msg_start:='
                         + str(start_frame), 'msg_end:=' + str(end_frame)]
        output = sub.run(roslaunch_cmd, capture_output=True, text=True)
        print(output.stdout)

        # TODO: run from IDE
        # convert to str
        # roslaunch_cmd = ' '.join(roslaunch_cmd)
        # pipe = sub.Popen(". %s; env" % roslaunch_cmd, stdout=sub.PIPE, shell=True)
        # output = pipe.communicate()[0]
        # print output of roslaunch command
        # print(output)


# save information about detected accidents in csv file
def save_accident_to_csv(input_path, time_stamp, file_name):
    path = input_path.split('/')
    path = [e.split('_') for e in path]
    path = [e[1] for e in path if e[0] in ['year', 'month', 'day', 'hour']]
    path.append(os.path.splitext(os.path.basename(input_path))[0])
    path.append(time_stamp)
    with open(file_name, 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(path)


# figure out timestamp of event and extract images of the 5 seconds before and after the event
def extract_images(actor, event, input_path, output_path, output_base_path, event_counter=0):
    if any(actor[event]):
        time_stamps_events = get_timestamp_of_event(actor, event)
        input_topics, sensor_ids, msg_count = get_params_for_img_extraction(input_path)

        # extract images for these timestamps
        for time_stamp in time_stamps_events:
            if event == 'in_accident':
                event = 'accident'
            event_nr = event + '_' + str(event_counter)
            if not os.path.exists(output_path):
                os.mkdir(output_path)
            output_path_timestamp = os.path.join(output_path, event_nr)
            if not os.path.exists(output_path_timestamp):
                os.mkdir(output_path_timestamp)
            output_path_timestamp += '/'

            roslaunch_bag_extractor(time_stamp, input_path, output_path_timestamp, input_topics, sensor_ids, msg_count)
            file_name = str(event) + 's.csv'
            file_name = os.path.join(output_base_path, file_name)
            if os.path.exists(file_name):
                save_accident_to_csv(input_path, time_stamp, file_name)
            event_counter += 1
    return event_counter


# extract all images of all cameras available in rosbag stored at input_path
def extract_all_images(input_path, output_path):
    input_topics, sensor_ids, msg_count = get_params_for_img_extraction(input_path)

    if not os.path.exists(output_path):
        os.mkdir(output_path)
    output_path = os.path.join(output_path, "images")
    if not os.path.exists(output_path):
        os.mkdir(output_path)
    output_path += '/'

    roslaunch_bag_extractor(-1, input_path, output_path, input_topics, sensor_ids, msg_count)
    return output_path
