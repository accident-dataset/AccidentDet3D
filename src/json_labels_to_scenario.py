import json
import os
import pandas as pd
import numpy as np
import scenario_variation

# Description:
# This module takes a folder of json files (detections) and camera images as input and extracts the scenarios.

def extract_scenarios_to_json(folder):
    frame_rate = 25
    dataset = []
    folder_name = os.path.basename(os.path.normpath(folder))
    detections = sorted(os.listdir(os.path.join(folder, 'detections')), key=lambda x: int(x.rsplit('_', 1)[-1][:6]))
    images = sorted(os.listdir(os.path.join(folder, 'images')))
    if len(images) == 0:
        images = list(np.full(len(detections), ''))
    if 0 != len(images) < len(detections):
        insertions = list(np.linspace(0, len(detections), len(detections) - len(images) + 1))
        [images.insert(int(im), images[int(im)]) for im in insertions[:-1]]

    scenario = {
        'meta': {'num_frames': len(detections), 'frame_rate': frame_rate, 'images': images, 'timestamp_secs': [],
                 'timestamp_nsecs': [], 'camera_channel': folder_name.split("_", 5)[-1], 'sequence': folder_name,
                 'time_of_day': '', 'weather_type': 'SUNNY'}, 'actors': []}
    for i in range(len(detections)):
        filename = detections[i]
        if filename.endswith('.json'):
            with open(os.path.join(folder, 'detections', filename)) as f:

                # returns JSON object as
                # a dictionary
                data = json.load(f)

                scenario['meta']['timestamp_secs'].append(data['timestamp_secs'])
                scenario['meta']['timestamp_nsecs'].append(data['timestamp_nsecs'])

                for j in range(len(data['labels'])):
                    # Titles
                    # Metadata titles
                    titles_data_general = [key for key in data.keys()]

                    # Labels category titles
                    titles_data_labels = [key for key in data['labels'][j].keys()]

                    # box3d category titles
                    titles_data_box3d = [key for key in data['labels'][j]['box3d'].keys()]

                    # box3d category titles
                    titles_data_box3d_dimensions = [key for key in data['labels'][j]['box3d']['dimension'].keys()]

                    # box3d category titles
                    titles_data_box3d_location = [key for key in data['labels'][j]['box3d']['location'].keys()]

                    # box3d category titles
                    titles_data_box3d_orientation = [key for key in data['labels'][j]['box3d']['orientation'].keys()]

                    # box3d category titles
                    titles_data_velocity = ['velocity_' + key for key in data['labels'][j]['velocity'].keys()]

                    # box3d category titles
                    titles_data_acceleration = ['acceleration_' + key for key in
                                                data['labels'][j]['acceleration'].keys()]

                    columns = (titles_data_general[:-1] + titles_data_labels[0:2] + titles_data_box3d_dimensions +
                               titles_data_box3d_location + titles_data_box3d_orientation + titles_data_velocity +
                               titles_data_acceleration)
                    ####################################################################################################

                    # Data
                    # Metadata titles
                    data_general = [value for value in data.values()]

                    # Labels category titles
                    data_labels = [value for value in data['labels'][j].values()]

                    # box3d category titles
                    data_box3d = [value for value in data['labels'][j]['box3d'].values()]

                    # box3d category titles
                    data_box3d_dimensions = [value for value in data['labels'][j]['box3d']['dimension'].values()]

                    # box3d category titles
                    data_box3d_location = [value for value in data['labels'][j]['box3d']['location'].values()]

                    # box3d category titles
                    data_box3d_orientation = [value for value in data['labels'][j]['box3d']['orientation'].values()]

                    # box3d category titles
                    data_velocity = [value for value in data['labels'][j]['velocity'].values()]

                    # box3d category titles
                    data_acceleration = [value for value in data['labels'][j]['acceleration'].values()]

                    data_row = (data_general[:-1] + data_labels[0:2] + data_box3d_dimensions + data_box3d_location +
                                data_box3d_orientation + data_velocity + data_acceleration)

                    # append to dataset
                    dataset.append(data_row)

    df = pd.DataFrame(dataset, columns=columns)
    df = df.drop(['name', 'timestamp_secs', 'timestamp_nsecs', 'timestamp_full'], axis=1)
    df = df.rename(columns={"index": "time_stamp"})
    df['time_stamp'] = df['time_stamp'] / frame_rate
    df = df.sort_values(by=['time_stamp'])

    for category in df['category'].unique():
        df_1 = df.loc[(df['category'] == category)]
        for ID in df_1['id'].unique():
            df_2 = df_1.loc[(df_1['id'] == ID)]
            df_2 = df_2.sort_values(by=['time_stamp'])
            df_2 = df_2.groupby(['time_stamp', 'id', 'category', 'width', 'length', 'height'],
                                as_index=False)[['x', 'y', 'z', 'rotationYaw', 'rotationPitch', 'rotationRoll',
                                                 'velocity_x', 'velocity_y', 'velocity_z', 'acceleration_x',
                                                 'acceleration_y', 'acceleration_z']].mean()

            if len(df_2) > 50:
                actor = {}
                actor['id'] = ID
                category = df_2['category'].iloc[0]
                actor['category'] = category
                actor['make'] = scenario_variation.random_vehicle(str(category))
                actor['color'] = np.array(scenario_variation.random_RGB())
                actor['frame_rate'] = frame_rate
                actor['offset'] = min(df_2['time_stamp'])
                actor['time_stamp'] = np.array(df_2['time_stamp'])
                dirty_trajectory = np.array((df_2['x'], df_2['y'], df_2['z'], df_2['rotationYaw'])).T
                actor['velocities'] = np.linalg.norm(np.array((df_2['velocity_x'], df_2['velocity_y'],
                                                               df_2['velocity_z'])), axis=0)

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
                    final_trajectory = scenario_variation.path_smoothing_outlier_removal(actor['path'],
                                                                                         actor['time_stamp'], 100, 0)

                    """velocities = np.sqrt(np.sum(np.diff(final_trajectory, axis=0) ** 2, axis=1)) * frame_rate
                    velocities = np.insert(velocities, -1, velocities[-1])
                    standing = np.where(velocities <= 0.001)

                    # path['velocities'] = scenario_variation.velocities_smoothing(velocities, path['time_stamp'], path['frame_rate'], 100, 20)
                    actor['velocities'] = velocities
                    actor['velocities'][standing] = 0"""

                    # final_trajectory = scenario_variation.interpolate_velocities(final_trajectory, path['velocities'], path['frame_rate'])
                    actor['path'] = np.zeros((final_trajectory.shape[0], final_trajectory.shape[1] + 2))
                    actor['path'][:, :2] = final_trajectory
                    actor['velocities'] = actor['velocities'][:len(actor['path'])]
                    actor['time_stamp'] = actor['time_stamp'][:len(actor['path'])]

                if len(actor['path']) != len(reduced_path) != 1:
                    outlier_free_trajectory = scenario_variation.path_smoothing_outlier_removal(actor['path'],
                                                                                                actor['time_stamp'],
                                                                                                100,
                                                                                                0)

                    """velocities = np.sqrt(np.sum(np.diff(outlier_free_trajectory, axis=0) ** 2, axis=1)) * frame_rate
                    velocities = np.insert(velocities, -1, velocities[-1])
                    standing = np.where(velocities <= 0.001)
                    # path['velocities'] = scenario_variation.velocities_smoothing(velocities, path['time_stamp'], path['frame_rate'], 100, 20)
                    actor['velocities'] = velocities
                    actor['velocities'][standing] = 0"""

                    # final_trajectory = scenario_variation.interpolate_velocities(outlier_free_trajectory, path['velocities'],path['frame_rate'])
                    final_trajectory = outlier_free_trajectory

                    actor['path'] = np.zeros((final_trajectory.shape[0], final_trajectory.shape[1] + 2))
                    actor['path'][:, :2] = final_trajectory
                    actor['velocities'] = actor['velocities'][:len(actor['path'])]
                    actor['time_stamp'] = actor['time_stamp'][:len(actor['path'])]

                angles = np.arctan2(np.diff(actor['path'][:, 1], axis=0),
                                    np.diff(actor['path'][:, 0], axis=0))
                angles = np.insert(angles, -1, angles[-1])
                actor['path'][:, 3] = angles

                scenario['actors'].append(actor)

    return scenario
