import os
import argparse
import rosbag
import numpy as np
import time
import sys


if __name__ == '__main__':
    s_time = time.time()

    start_time = time.time()
    print("Parsing Python parameters.")
    parser = argparse.ArgumentParser()
    # Adding optional arguments
    parser.add_argument("-i", "--input_folder_path_rosbags", default="/media/hdd/rosbags/storage/",
                        help="Input folder path rosbags")
    parser.add_argument("-o", "--output_file_path",
                        default="/media/hdd/user/output/standing_classification.txt",
                        help="Output file path for the accident classification")
    parser.add_argument("-e", "--exit_code_1", action='store_true',
                        help="If there is only 1 rosbag and no standing vehicle detected exit with exit code 1")

    # Read arguments from command line
    args = parser.parse_args()
    opt = vars(parser.parse_args())
    print("--- %s seconds (parse arguments) ---" % (time.time() - start_time))

    # get all files of the input directory and all its subdirectories and store their names in a list
    start_time = time.time()
    files = []
    for folder_name, _, file_names in os.walk(opt["input_folder_path_rosbags"]):
        for file_name in file_names:
            file_path = os.path.join(folder_name, file_name)
            if os.path.isfile(file_path):
                files.append(file_path)
    files = sorted(files)
    print("--- %s seconds (get all files and sort them) ---" % (time.time() - start_time))

    for file in files:
        start_time = time.time()

        with open(opt["output_file_path"], 'a', newline='') as file_txt:
            print('\n' + file)
            dict = {}
            standing_detected = False
            input_topic = ''
            if file.endswith('.bag'):
                with (rosbag.Bag(file, 'r') as bag):
                    topics = bag.get_type_and_topic_info().topics
                    if '/s40/s50/tracker/estimates/throttled' in topics:
                        input_topic = '/s40/s50/tracker/estimates/throttled'
                    elif '/backend/fusion/estimates/interpolated/synchronized' in topics:
                        input_topic = '/backend/fusion/estimates/interpolated/synchronized'
                    elif '/backend/fusion/estimates' in topics:
                        input_topic = '/backend/fusion/estimates'

                    if input_topic == '':
                        print(
                            "ERROR! BackendOutput topic could not be found. No scenarios can be detected for rosbag " + str(
                                file) + ".")
                        continue

                    for _, msg, _ in bag.read_messages(topics=[input_topic]):  # /synchronized

                        for object in msg.object_list:
                            if np.linalg.norm(object.speed) < 0.04:
                                if int(object.object_ID) in dict:
                                    dict[int(object.object_ID)] += 1
                                else:
                                    dict[int(object.object_ID)] = 1

                                if dict[int(object.object_ID)] == 25:
                                    standing_detected = True
                                    file_txt.write(file + "\n")
                                    break
                        else:
                            # once a standing vehicle is detected break out of both loops and go to the next file
                            continue
                        break
        print("--- %s seconds (one rosbag) ---" % (time.time() - start_time))

    print("--- %s seconds (whole python file) ---" % (time.time() - s_time))

    # check if the file should be exited with exit code 1
    if len([file for file in files if file.endswith('.bag')]) == 1 and opt["exit_code_1"] and not standing_detected:
        print("Exit with exit code 1")
        sys.exit(1)
