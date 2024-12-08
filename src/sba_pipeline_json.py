# Import
import os
import sys
import json
import inspect
import argparse
import time
import numpy as np
from sensorstations import sensor_stations_plot_parameters
import matplotlib.pyplot as plt
from json_labels_to_scenario import extract_scenarios_to_json
from scenario_variation import run_variation
from feature_extraction.lane_id_extractor_scenario import lane_id_extractor
from feature_extraction.distance_lead_follow_vehicle import ExtractDistanceLeadFollowVehicle
from maneuver_detection.lane_change_cut_in_cut_out_detector import lane_change_detector
from maneuver_detection.speeding_standing_detector import speeding_standing
from maneuver_detection.tailgate_detector import tail_gate_detector
from sba_statistics import scenario_statistics
from scenario_to_json_labels import json_writer
from scenario_writer.scenario_writer_esmini import scenario_writer_esmini
import rospy

# Description:
# This module extracts scenarios from the digital twin that is stored in JSON files

if __name__ == '__main__':
    rospy.init_node("json_scenario_extractor")
    parser = argparse.ArgumentParser()

    # Adding optional argument
    parser.add_argument("-i", "--input", help="Input file path")
    parser.add_argument("-o", "--output", help="Output file path")
    parser.add_argument("-v", "--variation", help="Number of scenario variations")
    parser.add_argument("-s", "--statistics", help="Create statistics")
    parser.add_argument("-pl", "--plot", help="Create statistics plot")
    parser.add_argument("-r", "--rosbag", help="Append labels to Rosbag")
    parser.add_argument("-osc", "--openscenario", help="Create OpenSCENARIO files")
    parser.add_argument("-j", "--json", help="Create JSON labels")

    # Read arguments from command line
    args = parser.parse_args()
    directory = args.input
    output = directory
    if args.output:
        output = args.output

    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    sys.path.insert(0, parentdir)

    subfolders = [f.path for f in os.scandir(directory) if f.is_dir()][2:]

    for folder in subfolders:
        if os.path.exists(os.path.join(folder, 'detections')) and os.path.exists(os.path.join(folder, 'images')):
            # Scenario Source
            start_time = time.time()
            scenario = extract_scenarios_to_json(folder)
            print("--- %s seconds ---" % (time.time() - start_time))

            # Scenario Variation
            if args.variation:
                n = args.variation
                scenarios = run_variation(scenario, n)
                original_and_augmented_scenarios = scenarios.append(scenario)

            # Feature Extraction
            for i in range(len(scenarios)):
                start_time = time.time()
                scenario1 = lane_id_extractor(scenarios[i])
                print("--- %s seconds ---" % (time.time() - start_time))
                start_time = time.time()
                scenarios[i] = ExtractDistanceLeadFollowVehicle.distance_lead_follow_vehicle(scenario1)
                print("--- %s seconds ---" % (time.time() - start_time))

            # Maneuver Detection
            start_time = time.time()
            speedlimit = 130
            for i in range(len(scenarios)):
                scenario = speeding_standing(scenarios[i], speedlimit)
                scenario = lane_change_detector(scenario)
                scenario = tail_gate_detector(scenario)
                scenarios[i] = scenario

            print("--- %s seconds ---" % (time.time() - start_time))

            # Scenario Statistics
            if args.statistics:
                for i in range(len(scenarios)):
                    statistic = scenario_statistics(scenarios[i])
                    with open(os.path.join(folder, "Statistics.json"), 'w') as f:
                        # where data is your valid python dictionary
                        json.dump(statistic, f)

            if args.plot:
                for scenario in scenarios:
                    cm = 1 / 2.54  # centimeters in inches
                    fig, ax = plt.subplots(14, figsize=(50 * cm, 100 * cm))
                    path, extent, xlim, ylim = sensor_stations_plot_parameters.S50()
                    img = plt.imread(path)
                    ax[0].imshow(img, extent=extent)
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

                    for actor in scenario['actors']:
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
                        # TODO: set total speeding and total standing
                        # ax[11].plot(np.array(range(scenario['meta']['num_frames'])) / 25,
                        #             scenario['meta']['total_speeding'], '-')
                        ax[11].plot(actor['time_stamp'], actor['standing'], '-')
                        # ax[13].plot(np.array(range(scenario['meta']['num_frames'])) / 25,
                        #             scenario['meta']['total_standing'], '-')

                    plt.savefig(os.path.join(folder, "Evaluation.png"), dpi=200)
                    plt.close(fig)

            if args.json:
                if not os.path.exists(os.path.join(folder, 'labels')):
                    os.makedirs(os.path.join(folder, 'labels'))
                json_writer(scenario, os.path.join(folder, 'labels'))

            if args.openscenario:
                open_scenario = scenario_writer_esmini(scenario)
                # write the OpenSCENARIO file as xosc using current folder name
                open_scenario.write_xml(os.path.join(directory, os.path.splitext(folder)[0] + "_esmini.xosc"))
