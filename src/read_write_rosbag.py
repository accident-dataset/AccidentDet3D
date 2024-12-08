import rospy
from std_msgs.msg import Header
from system_messages.msg import *

import rosbag
import pandas as pd
import numpy as np
import scenario_variation

pd.set_option('display.max_rows', None)
pd.set_option('display.max_columns', None)
pd.set_option('display.width', None)
pd.set_option('display.max_colwidth', None)


# Description:
# This module takes a rosbag file as input and extracts the scenarios within the ros messages of the digital twin

def extract_scenarios_from_rosbag(bag, input_topic, frame_threshold):
    frame_rate = 25
    dataset = []
    bag = rosbag.Bag(bag)
    print(bag)
    message_count = bag.get_message_count([input_topic])

    scenario = {'meta': {'num_frames': message_count, 'frame_rate': frame_rate,
                         'timestamp_secs': [], 'timestamp_nsecs': [], 'camera_channel': "", 'sequence': "",
                         'time_of_day': '', 'weather_type': 'SUNNY', 'standing_vehicles_shoulder': [],
                         'standing_vehicles_total': [], 'speeding_vehicles_total': [],
                         'average_velocity_north': 0, 'average_velocity_south': 0,
                         'average_velocity_per_frame_south_0': [], 'average_velocity_per_frame_south_250': [],
                         'average_velocity_per_frame_north_0': [], 'average_velocity_per_frame_north_250': [],
                         'traffic_jam_north': 0, 'traffic_jam_south': 0,
                         'slow_moving_traffic_north': 0, 'slow_moving_traffic_south': 0},
                'actors': []}

    scenario['meta']['speeding_vehicles_total'] = np.full((message_count,), -1.0)
    scenario['meta']['standing_vehicles_total'] = np.full((message_count,), -1.0)
    scenario['meta']['standing_vehicles_shoulder'] = np.full((message_count,), -1.0)
    scenario['meta']['average_velocity_per_frame_south_0'] = np.full((message_count,), -1.0)
    scenario['meta']['average_velocity_per_frame_south_250'] = np.full((message_count,), -1.0)
    scenario['meta']['average_velocity_per_frame_north_0'] = np.full((message_count,), -1.0)
    scenario['meta']['average_velocity_per_frame_north_250'] = np.full((message_count,), -1.0)
    index = 0
    msg_idx = 0
    sequences = []
    for _, msg, _ in bag.read_messages(topics=[input_topic]):  # /synchronized
        # print(index)
        if msg_idx % 1000 == 0:
            print("msg_idx:", str(msg_idx))
        msg_idx = msg_idx + 1
        if not msg._has_header:
            sequences.append(0)
            scenario["meta"]["timestamp_secs"].append(0)
            scenario["meta"]["timestamp_nsecs"].append(0)
        else:
            sequences.append(msg.header.seq)
            scenario['meta']['timestamp_secs'].append(msg.header.stamp.secs)
            scenario['meta']['timestamp_nsecs'].append(msg.header.stamp.nsecs)

        for object in msg.object_list:
            if not msg._has_header:
                data_row = {'time_stamp': index, 'secs': 0, 'nsecs': 0,
                            'object_ID': int(object.object_ID), 'object_class': int(object.object_class),
                            'x': object.position[0], 'y': object.position[1],
                            'z': object.position[2], 'yaw': 0.0, 'speed': np.linalg.norm(object.speed),
                            'speed_x': object.speed[0], 'speed_y': object.speed[1], 'speed_z': object.speed[2],
                            'length': object.dimensions[0], 'width': object.dimensions[1],
                            'height': object.dimensions[2]}
            else:
                data_row = {'time_stamp': index, 'secs': msg.header.stamp.secs, 'nsecs': msg.header.stamp.nsecs,
                        'object_ID': int(object.object_ID), 'object_class': int(object.object_class),
                        'x': object.position[0], 'y': object.position[1],
                        'z': object.position[2], 'yaw': 0.0, 'speed': np.linalg.norm(object.speed),
                        'speed_x': object.speed[0], 'speed_y': object.speed[1], 'speed_z': object.speed[2],
                        'length': object.dimensions[0], 'width': object.dimensions[1], 'height': object.dimensions[2]}
            dataset.append(data_row)
        index += 1 / scenario['meta']['frame_rate']
    bag.close()

    df = pd.DataFrame(dataset)

    for ID in df['object_ID'].unique():
        df_2 = df.loc[(df['object_ID'] == ID)]
        df_2 = df_2.sort_values(by=['time_stamp'])
        df_2 = \
            df_2.groupby(
                ['time_stamp', 'secs', 'nsecs', 'object_ID', 'object_class', 'length', 'width', 'height', 'yaw'],
                as_index=False)[['x', 'y', 'z', 'speed', 'speed_x', 'speed_y', 'speed_z']].mean()

        # process only vehicles that were tracked longer than selected threshold
        if len(df_2) > frame_threshold:
            actor = {}
            actor['id'] = ID
            actor['object_class'] = df_2['object_class']
            actor['color'] = np.array(scenario_variation.random_RGB())
            actor['frame_rate'] = frame_rate
            actor['offset'] = min(df_2['time_stamp'])
            actor['length'] = np.array(df_2['length'])
            actor['width'] = np.array(df_2['width'])
            actor['height'] = np.array(df_2['height'])
            actor['time_stamp'] = np.array(df_2['time_stamp'])
            dirty_trajectory = np.array((df_2['x'], df_2['y'], df_2['z'], df_2['yaw'])).T
            actor['velocities'] = np.array((df_2['speed']))
            actor['speed_x'] = np.array((df_2['speed_x']))
            actor['speed_y'] = np.array((df_2['speed_y']))
            actor['speed_z'] = np.array((df_2['speed_z']))

            path = dirty_trajectory[:, :2]
            _, index = np.unique(path, axis=0, return_index=True)
            reduced_path = path[np.sort(index)]

            actor['path'] = path

            if len(reduced_path) == 1:
                actor['path'] = np.zeros((len(actor['time_stamp']), 4))
                actor['path'][:, :2] = np.full((len(actor['time_stamp']), 2), reduced_path)
                velocities = np.full((len(actor['time_stamp'])), 0.0)
                actor['velocities'] = velocities

            if len(actor['path']) == len(reduced_path) != 1:
                # TODO: add parameter for path smoothing
                # final_trajectory = scenario_variation.path_smoothing_outlier_removal(actor['path'], actor['time_stamp'],
                #                                                                      100, 0)
                final_trajectory = actor['path']

                actor['path'] = np.zeros((final_trajectory.shape[0], final_trajectory.shape[1] + 2))
                actor['path'][:, :2] = final_trajectory
                actor['velocities'] = actor['velocities'][:len(actor['path'])]
                actor['time_stamp'] = actor['time_stamp'][:len(actor['path'])]

            if len(actor['path']) != len(reduced_path) != 1:
                # TODO: add parameter for path smoothing
                # final_trajectory = scenario_variation.path_smoothing_outlier_removal(actor['path'], actor['time_stamp'],
                #                                                                      100, 0)
                final_trajectory = actor['path']

                actor['path'] = np.zeros((final_trajectory.shape[0], final_trajectory.shape[1] + 2))
                actor['path'][:, :2] = final_trajectory
                actor['velocities'] = actor['velocities'][:len(actor['path'])]
                actor['time_stamp'] = actor['time_stamp'][:len(actor['path'])]

            angles = np.rad2deg(np.arctan2(np.abs(np.diff(actor['path'][:, 1], axis=0)),
                                np.abs(np.diff(actor['path'][:, 0], axis=0))))

            if angles.shape != (0,):
                angles = np.insert(angles, -1, angles[-1])
                actor['path'][:, 3] = angles

            scenario['actors'].append(actor)
            scenario['actors'] = scenario['actors']
    del df
    del df_2
    return scenario


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]


def scenario_to_rosbag(scenario, output_file_path_rosbag):
    backend_messages = []
    compression = rosbag.Compression.NONE
    # compression = rosbag.Compression.BZ2
    # compression = rosbag.Compression.LZ4

    for i in range(scenario['meta']['num_frames']):
        backend_msg = BackendOutputExtended()
        backend_msg.header = Header()
        backend_msg.header.stamp = rospy.Time()
        backend_msg.header.stamp.secs = int(scenario['meta']['timestamp_secs'][i])
        backend_msg.header.stamp.nsecs = int(scenario['meta']['timestamp_nsecs'][i])
        backend_msg.header.seq = i
        backend_msg.num_detected = 0
        object_list = []
        for actor in scenario['actors']:
            offset = int(np.round(actor['offset'] * actor['frame_rate']))
            if 0 <= i - offset < len(actor['path']):
                backend_msg.num_detected += 1

                object_data_msg = DetectedObjectExtended()
                object_data_msg.object_ID = int(actor["id"])
                x = actor["path"][i - offset, 0]
                y = actor["path"][i - offset, 1]
                z = actor["path"][i - offset, 2]
                object_data_msg.position = [x, y, z]
                length = actor["length"][i - offset]
                width = actor["width"][i - offset]
                height = actor["height"][i - offset]
                object_data_msg.dimensions = [length, width, height]
                yaw = actor["path"][i - offset, 3]
                pitch = 0.0
                roll = 0.0
                quaternion = get_quaternion_from_euler(np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw))
                object_data_msg.heading = quaternion
                speed_x = actor['speed_x'][i - offset]
                speed_y = actor['speed_y'][i - offset]
                speed_z = actor['speed_z'][i - offset]
                object_data_msg.speed = [speed_x, speed_y, speed_z]
                object_data_msg.object_class = actor['object_class'][i - offset]
                object_data_msg.speeding = bool(actor['speeding'][i - offset])
                object_data_msg.standing = bool(actor['standing'][i - offset])
                object_data_msg.standing_shoulder = bool(actor['standing_shoulder'][i - offset])
                object_data_msg.lane_id = int(actor['lane_id'][i - offset])
                object_data_msg.lane_change_left = bool(actor['lane_change_left'][i - offset])
                object_data_msg.lane_change_right = bool(actor['lane_change_right'][i - offset])
                object_data_msg.cut_in_left = bool(actor['cut_in_left'][i - offset])
                object_data_msg.cut_in_right = bool(actor['cut_in_right'][i - offset])
                object_data_msg.cut_out_left = bool(actor['cut_out_left'][i - offset])
                object_data_msg.cut_out_right = bool(actor['cut_out_right'][i - offset])
                object_data_msg.tail_gate = actor['tail_gate'][i - offset]
                object_list.append(object_data_msg)

        backend_msg.object_list = object_list
        backend_messages.append(backend_msg)

    dtwin_topic = '/s40/s50/tracker/estimates/throttled/extended'

    with rosbag.Bag(output_file_path_rosbag, 'a', compression=compression) as outbag:
        # iterate all dtwin_messages and write them to rosbag
        for frame_id, backend_msg in enumerate(backend_messages):
            outbag.write(dtwin_topic, backend_msg, t=backend_msg.header.stamp)
