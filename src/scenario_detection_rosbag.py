#!/usr/bin/python

# Import
import os
import re
import csv
import shutil
import time
import rosbag
import rospy
import json
import argparse
import matplotlib.pyplot as plt
import numpy as np

np.int = int

from read_write_rosbag import extract_scenarios_from_rosbag, scenario_to_rosbag
from feature_extraction.lane_id_extractor_scenario import lane_id_extractor
from feature_extraction.distance_lead_follow_vehicle import ExtractDistanceLeadFollowVehicle
from feature_extraction.average_velocity_extractor import (calculate_average_velocity,
                                                           calculate_average_velocity_per_frame)
from maneuver_detection.traffic_jam_detection import detect_traffic_jam
from maneuver_detection.lane_change_cut_in_cut_out_detector import lane_change_detector
from maneuver_detection.speeding_standing_detector import speeding_standing
from maneuver_detection.tailgate_detector import tail_gate_detector
from sba_statistics import scenario_statistics
from maneuver_detection.accident_detection import accident_detection
from maneuver_detection.accident_detection_ml import accident_detection_ml
from maneuver_detection.breakdown_detection import breakdown_detection

# Description:
# This module extracts scenarios from the digital twin that is stored in rosbag files

# get all parameters necessary for performing the image extraction
def get_params_for_img_extraction(opt, file):
    if opt["extract_images_mode"] == "none":
        extract_imgs = 0
    elif opt["extract_images_mode"] == "standing_shoulder":
        extract_imgs = 1
    elif opt["extract_images_mode"] == "accident":
        extract_imgs = 2
    elif opt["extract_images_mode"] == "breakdown":
        extract_imgs = 3
    elif opt["extract_images_mode"] == "accident_breakdown":
        extract_imgs = 4
    else:
        print("ERROR: images for event " + opt["extract_images_mode"] + " can not be extracted.")
        extract_imgs = 0

    input_path = file
    output_path = os.path.join(
        opt["output_folder_path_extracted_scenarios"],
        os.path.relpath(os.path.splitext(file)[0], opt["input_folder_path_rosbags"])
    )
    if not os.path.exists(output_path):
        os.mkdir(output_path)

    return extract_imgs, input_path, output_path


if __name__ == '__main__':
    s_time = time.time()
    rospy.init_node("scenario_detection_rosbag_node", anonymous=True)
    opt = {}
    try:
        opt["frame_threshold"] = int(rospy.get_param("~frame_threshold"))
        opt["input_folder_path_rosbags"] = str(rospy.get_param("~input_folder_path_rosbags"))
        opt["output_folder_path_extracted_scenarios"] = str(rospy.get_param("~output_folder_path_extracted_scenarios"))
        opt["write_statistics_json"] = bool(rospy.get_param("~write_statistics_json"))
        opt["plot_statistics"] = bool(rospy.get_param("~plot_statistics"))
        opt["write_scenarios_rosbag"] = bool(rospy.get_param("~write_scenarios_rosbag"))
        opt["extract_images_mode"] = str(rospy.get_param("~extract_images_mode"))
        opt["accident_detection_mode"] = str(rospy.get_param("~accident_detection_mode"))
    except KeyError:
        print("Parsing Python parameters.")
        parser = argparse.ArgumentParser()
        # Adding optional arguments
        parser.add_argument("--frame_threshold", type=int, default=25, help="Number of consecutive frames in which an objects needs to be detected")
        parser.add_argument("-i", "--input_folder_path_rosbags", default="/media/hdd/rosbags/storage",
                            help="Input folder path to rosbags")
        parser.add_argument("-o", "--output_folder_path_extracted_scenarios",
                            default="/media/hdd/rosbags/output",
                            help="Output folder path")
        # parser.add_argument("-v", "--Variation", help="Number of scenario variations")
        parser.add_argument("-s", "--write_statistics_json", action='store_true', help="Generate statistics (as .json files)")
        parser.add_argument("-pl", "--plot_statistics", action='store_true', help="Generate plots for statistics")
        parser.add_argument("-r", "--write_scenarios_rosbag", action='store_true', help="Append labels to rosbag file")
        parser.add_argument("-img", "--extract_images_mode", type=str, default="",
                    help="Extract images of specified event (standing_shoulder, accident, breakdown, accident_breakdown)")
        parser.add_argument("-adm", "--accident_detection_mode", type=str, default="rule_based",
                            help="Specify accident detection that should be used (rule_based or ml)")

        # Read arguments from command line
        args = parser.parse_args()
        opt = vars(parser.parse_args())

    if not os.path.exists(opt["output_folder_path_extracted_scenarios"]):
        os.mkdir(opt["output_folder_path_extracted_scenarios"])
    if not (opt["accident_detection_mode"] == "rule_based" or opt["accident_detection_mode"] == "ml"):
        print("ERROR! Argument of -adm argument must be \'rule_based\' or \'ml\'. Uses default value \'rule_based\' instead.")

    # get all files of the input directory and all its subdirectories and store their names in a list
    files = []
    for folder_name, _, file_names in os.walk(opt["input_folder_path_rosbags"]):
        for file_name in file_names:
            file_path = os.path.join(folder_name, file_name)
            if os.path.isfile(file_path):
                files.append(file_path)
    files = sorted(files)

    # create csv file if -img set and directory structure corresponds to year_X/month_X/day_X/hour_X/...
    path_pattern = r'year_[0-9][0-9][0-9][0-9]/month_[0-9][0-9]/day_[0-9]+/hour_[0-9]+/[a-z]+'
    if opt["extract_images_mode"] and len(files) > 0 and re.match(path_pattern, os.path.relpath(files[0], opt["input_folder_path_rosbags"])):
        if opt["extract_images_mode"] == "accident_breakdown":
            file_names = ["accident", "breakdown"]
        else:
            file_names = [opt["extract_images_mode"]]

        for event in file_names:
            file_name = event + 's.csv'
            file_name = os.path.join(opt["output_folder_path_extracted_scenarios"], file_name)
            if not os.path.exists(file_name):
                with open(file_name, 'a', newline='') as file:
                    writer = csv.writer(file)
                    row_names = ['year', 'month', 'day', 'hour', 'rosbag_name', 'time_stamp', event]
                    writer.writerow(row_names)

    for file in files:
        # create same subdirectories for rosbag as in input directory
        directory_path = os.path.relpath(file, opt["input_folder_path_rosbags"])
        directories = directory_path.split('/')
        file_name = os.path.splitext(directories.pop())[0]
        temp_path = opt["output_folder_path_extracted_scenarios"]
        for e in directories:
            temp_path = os.path.join(temp_path, e)
            if not os.path.exists(temp_path):
                os.mkdir(temp_path)

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
                    print("ERROR! BackendOutput topic could not be found. No scenarios can be detected for rosbag " + str(file) + ".")
                    continue

                msg_count = topics[input_topic][1]

            # Scenario Source
            start_time = time.time()
            scenario = extract_scenarios_from_rosbag(os.path.join(opt["input_folder_path_rosbags"], file), input_topic,
                                                     frame_threshold=opt["frame_threshold"])
            print("--- %s seconds (scenario extraction) ---" % (time.time() - start_time))

            # Feature Extraction
            start_time = time.time()
            scenario = lane_id_extractor(scenario)
            print("--- %s seconds (lane id extraction) ---" % (time.time() - start_time))

            start_time = time.time()
            scenario = ExtractDistanceLeadFollowVehicle.distance_lead_follow_vehicle(scenario)
            print("--- %s seconds (distance to lead and follow vehicle extraction) ---" % (time.time() - start_time))

            start_time = time.time()
            scenario = calculate_average_velocity(scenario)
            scenario = calculate_average_velocity_per_frame(scenario)
            print("--- %s seconds (average velocity extraction) ---" % (time.time() - start_time))

            # Maneuver Detection
            start_time = time.time()
            extract_imgs, input_path, output_path = get_params_for_img_extraction(opt, file)
            output_base_path = opt["output_folder_path_extracted_scenarios"]
            scenario = detect_traffic_jam(scenario)
            scenario = speeding_standing(scenario, extract_imgs, input_path, output_path, output_base_path)
            scenario = breakdown_detection(scenario, extract_imgs, input_path, output_path, output_base_path)
            scenario = lane_change_detector(scenario)
            scenario = tail_gate_detector(scenario)

            print("--- %s seconds (maneuver detection) ---" % (time.time() - start_time))

            # Accident Detection
            start_time = time.time()
            acc_detection_ml = 0
            number_of_accidents_ml = 0
            if opt["accident_detection_mode"] == "ml":
                # learning-based accident detection
                acc_detection_ml = 1
                extract_imgs_ml = 0
                if extract_imgs == 2:
                    extract_imgs_ml = 1
                number_of_accidents_ml = accident_detection_ml(input_path, output_path, extract_imgs_ml)
            else:
                # rule-based accident detection
                scenario = accident_detection(scenario, extract_imgs, input_path, output_path, output_base_path)

            print("--- %s seconds (accident detection) ---" % (time.time() - start_time))

            # remove directory for rosbag if no event was detected (directory is then empty)
            has_subdirectory = 0
            if opt["accident_detection_mode"] == "rule_based":
                for item in os.listdir(output_path):
                    item_path = os.path.join(output_path, item)
                    if os.path.isdir(item_path):
                        has_subdirectory = 1
                        break
                if has_subdirectory == 0 and os.path.isdir(output_path):
                    shutil.rmtree(output_path)

            # Scenario Statistics
            start_time = time.time()
            if opt["write_statistics_json"]:
                statistic = scenario_statistics(scenario, acc_detection_ml, number_of_accidents_ml, msg_count)

                # if the image extraction is on and images got extracted mirror the folder structure,
                # otherwise write them in a seperate folder
                if not (opt["extract_images_mode"] and has_subdirectory):
                    output_path = opt["output_folder_path_extracted_scenarios"] + "/statistics/"
                    if not os.path.exists(output_path):
                        os.mkdir(output_path)

                with open(os.path.join(output_path, file_name + "_statistics.json"), 'w') as f:
                    json.dump(statistic, f)
            print("--- %s seconds (statistics creation) ---" % (time.time() - start_time))

            # Plot Statistics
            start_time = time.time()
            if opt["plot_statistics"]:
                cm = 1 / 2.54  # centimeters in inches
                fig, ax = plt.subplots(16, figsize=(50 * cm, 80 * cm))
                # path, extent, xlim, ylim = sensor_stations_plot_parameters.S50()
                # img = plt.imread(os.path.join(os.path.dirname(__file__), path))
                # ax[0].imshow(img, extent=extent)
                ax[0].set_ylabel('Trajectories')
                ax[1].set_ylabel('Velocity Profiles')
                ax[2].set_ylabel('Lane IDs')
                ax[3].set_ylabel('Lane Changes Left')
                ax[4].set_ylabel('Lane Changes Right')
                ax[5].set_ylabel('Cut Ins Left')
                ax[6].set_ylabel('Cut Ins Right')
                ax[7].set_ylabel('Cut Outs Left')
                ax[8].set_ylabel('Cut Outs Right')
                ax[9].set_ylabel('Tail Gates')
                ax[10].set_ylabel('Speeding')
                ax[11].set_ylabel('Total Vehicles\nSpeeding')
                ax[12].set_ylabel('Standing')
                ax[13].set_ylabel('Total Vehicles\nStanding')
                ax[14].set_ylabel('Standing Shoulder')
                ax[15].set_ylabel('Total Standing Shoulder')

                for actor in scenario['actors']:
                    if opt["accident_detection_mode"] == "rule_based" and 1 in actor['in_accident']:
                        ax[0].plot(actor['path'][:, 0], actor['path'][:, 1], '-')
                        ax[1].plot(actor['time_stamp'], actor['velocities'], '-')
                        ax[2].plot(actor['time_stamp'], actor['lane_id'], '-')
                        if actor['lane_changes_left'] + actor['lane_changes_right'] >= 2:
                            ax[3].plot(actor['time_stamp'], actor['lane_change_left'], '-')
                            ax[4].plot(actor['time_stamp'], actor['lane_change_right'], '-')

                        ax[5].plot(actor['time_stamp'], actor['cut_in_left'], '-')
                        ax[6].plot(actor['time_stamp'], actor['cut_in_right'], '-')
                        ax[7].plot(actor['time_stamp'], actor['cut_out_left'], '-')
                        ax[8].plot(actor['time_stamp'], actor['cut_out_right'], '-')
                        ax[9].plot(actor['time_stamp'], actor['tail_gate'], '-')
                        ax[10].plot(actor['time_stamp'], actor['speeding'], '-')
                        # TODO: set total speeding and total standing vehicles
                        ax[11].plot(np.array(range(scenario['meta']['num_frames'])) / 25,
                                    scenario['meta']['speeding_vehicles_total'], '-')
                        ax[12].plot(actor['time_stamp'], actor['standing'], '-')
                        ax[13].plot(np.array(range(scenario['meta']['num_frames'])) / 25,
                                    scenario['meta']['standing_vehicles_total'], '-')
                        ax[14].plot(actor['time_stamp'], actor['standing_shoulder'], '-')
                        ax[15].plot(np.array(range(scenario['meta']['num_frames'])) / 25,
                                    scenario['meta']['standing_vehicles_shoulder'], '-')

                if not (opt["extract_images"] and has_subdirectory):
                    output_path = opt["output_folder_path_extracted_scenarios"] + "/plots/"
                    if not os.path.exists(output_path):
                        os.mkdir(output_path)

                plt.savefig(os.path.join(output_path, file_name + "_evaluation.png"), dpi=200)
                plt.close(fig)
            print("--- %s seconds (plot creation) ---" % (time.time() - start_time))

            if opt["write_scenarios_rosbag"]:
                scenario_to_rosbag(scenario, os.path.join(opt["input_folder_path_rosbags"], file))

            # remove all empty subdirectories in output directory
            for path, _, _ in os.walk(opt["output_folder_path_extracted_scenarios"], topdown=False):
                if len(os.listdir(path)) == 0 and path != opt["output_folder_path_extracted_scenarios"]:
                    shutil.rmtree(path)

    print("--- %s seconds (whole file) ---" % (time.time() - s_time))

