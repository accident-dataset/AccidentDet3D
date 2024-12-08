import numpy as np
import pandas as pd

from scenario_variation import random_RGB
from feature_extraction.lane_id_extractor_scenario import extract_lane_id_for_actor
from feature_extraction.distance_lead_follow_vehicle import ExtractDistanceLeadFollowVehicle
from maneuver_detection.lane_change_cut_in_cut_out_detector import detect_cut_in_and_lane_change_for_actor
from maneuver_detection.speeding_standing_detector import detect_speeding_standing_for_actor
from maneuver_detection.tailgate_detector import detect_tail_gate_for_actor
from maneuver_detection.accident_detection import AccidentDetection


class FeatureExtractorDTwinLive:
    FRAME_RATE = 25

    def __init__(self,
                 cache_limit: int = 200,
                 frame_threshold: int = 25,
                 speed_limit: int = 130,  # kph
                 accident_velocity_diff_medium_th: int = 50,  # kph
                 accident_velocity_diff_severe_th: int = 70,  # kph
                 ):
        self.scenario_cache = {
            'meta': {
                'num_frames': 0, 'frame_rate': self.FRAME_RATE, 'images': "images",
                'timestamp_secs': [], 'timestamp_nsecs': [], 'camera_channel': "", 'sequence': "",
                'time_of_day': '', 'weather_type': 'SUNNY'},
            'actors': [],
            'dataset': []
        }

        self.index = 0.0
        self.cache_limit = cache_limit
        self.frame_threshold = frame_threshold
        self.speed_limit = speed_limit
        self.accident_detection = AccidentDetection(
            velocity_diff_medium_th=accident_velocity_diff_medium_th,
            velocity_diff_severe_th=accident_velocity_diff_severe_th
        )

    def get_actor_by_object_id(self, object_id):
        if object_id in self.scenario_cache["actors_indices"]:
            return self.scenario_cache["actors"][self.scenario_cache["actors_indices"][object_id]]

        return {}

    def extract_scenarios(self, msg):
        num_frames = self.scenario_cache["meta"]["num_frames"]

        if num_frames < self.cache_limit:
            num_frames += 1

        if num_frames == self.cache_limit:
            self.scenario_cache["meta"]["timestamp_secs"].pop(0)
            self.scenario_cache["meta"]["timestamp_nsecs"].pop(0)
            self.scenario_cache["dataset"].pop(0)

        self.scenario_cache["meta"]["num_frames"] = num_frames
        self.scenario_cache['meta']['timestamp_secs'].append(msg.header.stamp.secs)
        self.scenario_cache['meta']['timestamp_nsecs'].append(msg.header.stamp.nsecs)
        self.scenario_cache['actors'] = []
        self.scenario_cache['actors_indices'] = {}  # Map array indices to object ids

        dataset_msg = []
        for object in msg.object_list:
            data_row = {'time_stamp': self.index, 'secs': msg.header.stamp.secs, 'nsecs': msg.header.stamp.nsecs,
                        'object_ID': int(object.object_ID), 'object_class': int(object.object_class),
                        'x': object.position[0], 'y': object.position[1],
                        'z': object.position[2], 'yaw': 0.0, 'speed': np.linalg.norm(object.speed),
                        'speed_x': object.speed[0], 'speed_y': object.speed[1], 'speed_z': object.speed[2],
                        'length': object.dimensions[0], 'width': object.dimensions[1], 'height': object.dimensions[2]}
            dataset_msg.append(data_row)

        self.scenario_cache["dataset"].append(dataset_msg)
        self.index += 1 / self.scenario_cache['meta']['frame_rate']

        combined_dataset = []
        for data in self.scenario_cache["dataset"]:
            combined_dataset.extend(data)

        df = pd.DataFrame(combined_dataset)

        for ID in df['object_ID'].unique():
            obj_df = df.loc[(df['object_ID'] == ID)]
            obj_df = obj_df.sort_values(by=['time_stamp'])
            obj_df = obj_df.groupby(
                ['time_stamp', 'secs', 'nsecs', 'object_ID', 'object_class', 'length', 'width', 'height', 'yaw'],
                as_index=False,
            )[['x', 'y', 'z', 'speed', 'speed_x', 'speed_y', 'speed_z']].mean()

            # process only vehicles that were tracked longer than selected threshold
            if len(obj_df) > self.frame_threshold:
                actor = {'id': ID, 'object_class': obj_df['object_class'], 'color': np.array(random_RGB()),
                         'frame_rate': self.FRAME_RATE, 'offset': min(obj_df['time_stamp']),
                         'length': np.array(obj_df['length']), 'width': np.array(obj_df['width']),
                         'height': np.array(obj_df['height']), 'time_stamp': np.array(obj_df['time_stamp'])}

                dirty_trajectory = np.array((obj_df['x'], obj_df['y'], obj_df['z'], obj_df['yaw'])).T
                actor['velocities'] = np.array((obj_df['speed']))
                actor['speed_x'] = np.array((obj_df['speed_x']))
                actor['speed_y'] = np.array((obj_df['speed_y']))
                actor['speed_z'] = np.array((obj_df['speed_z']))

                path = dirty_trajectory[:, :2]
                _, index = np.unique(path, axis=0, return_index=True)
                reduced_path = path[np.sort(index)]

                actor['path'] = path

                if len(reduced_path) == 1:
                    actor['path'] = np.zeros((len(actor['time_stamp']), 4))
                    actor['path'][:, :2] = np.full((len(actor['time_stamp']), 2), reduced_path)
                    velocities = np.full((len(actor['time_stamp'])), 0.0)
                    actor['velocities'] = velocities

                if len(actor['path']) == len(reduced_path):
                    final_trajectory = actor['path']

                    actor['path'] = np.zeros((final_trajectory.shape[0], final_trajectory.shape[1] + 2))
                    actor['path'][:, :2] = final_trajectory
                    actor['velocities'] = actor['velocities'][:len(actor['path'])]
                    actor['time_stamp'] = actor['time_stamp'][:len(actor['path'])]

                if len(actor['path']) != len(reduced_path) != 1:
                    final_trajectory = actor['path']

                    actor['path'] = np.zeros((final_trajectory.shape[0], final_trajectory.shape[1] + 2))
                    actor['path'][:, :2] = final_trajectory
                    actor['velocities'] = actor['velocities'][:len(actor['path'])]
                    actor['time_stamp'] = actor['time_stamp'][:len(actor['path'])]

                angles = np.arctan2(np.diff(actor['path'][:, 1], axis=0), np.diff(actor['path'][:, 0], axis=0))
                angles = np.insert(angles, -1, angles[-1])
                actor['path'][:, 3] = angles

                extract_lane_id_for_actor(actor)
                ExtractDistanceLeadFollowVehicle.initialize_distance_features_for_actor(actor)
                self.scenario_cache['actors'].append(actor)
                self.scenario_cache['actors_indices'][ID] = len(self.scenario_cache['actors']) - 1

        # Extract features for scenario detection
        ExtractDistanceLeadFollowVehicle.extract_lead_follow_distances_for_msg(
            scenario=self.scenario_cache, msg_idx=num_frames-1
        )

        # Detect scenarios
        for t in self.scenario_cache['actors']:
            detect_speeding_standing_for_actor(actor=t, speed_limit=self.speed_limit)
            detect_cut_in_and_lane_change_for_actor(actor=t)
            detect_tail_gate_for_actor(actor=t)
            ExtractDistanceLeadFollowVehicle.extract_ttc_for_actor(actor=t)
            self.accident_detection.detect_accident_for_actor(actor=t)

        return self.scenario_cache
