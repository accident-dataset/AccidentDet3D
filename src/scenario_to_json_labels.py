import json
import numpy as np


# Description:
# This module converts scenarios into a json structure and writes them into a json file.

def json_writer(scenario, filename):
    frame_rate = scenario['meta']['frame_rate']
    scenario_length = scenario['meta']['num_frames']

    for i in range(scenario_length):
        labels = []
        for t in scenario['actors']:
            offset = int(np.round(t['offset'] * frame_rate))
            if 0 <= i - offset < len(t['path']):
                dimension = {'width': 0.0, 'length': 0.0, 'height': 0.0}
                location = {'x': t['path'][i - offset][0], 'y': t['path'][i - offset][1],
                            'z': t['path'][i - offset][2]}
                orientation = {'rotationYaw': t['path'][i - offset][3], 'rotationPitch': 0, 'rotationRoll': 0}
                attributes = []
                box3d_projected = {}
                box3d = {'dimension': dimension, 'location': location, 'orientation': orientation}
                label = {'id': int(t['id']), 'category': t['category'], 'box3d': box3d, 'attributes': attributes,
                         'box3d_projected': box3d_projected,
                         'features': {'lane_id': int(t['lane_id'][i - offset]),
                                      'distance_lane_center': t['distance_lane_center'][i - offset],
                                      'distance_lead': t['distance_lead'][i - offset],
                                      'distance_following': t['distance_following'][i - offset],
                                      'velocity_lead': t['velocity_lead'][i - offset],
                                      'velocity_following': t['velocity_following'][i - offset]},
                         'maneuvers': {'speeding': int(t['speeding'][i - offset]),
                                       'standing': int(t['standing'][i - offset]),
                                       'lane_change_left': int(t['lane_change_left'][i - offset]),
                                       'lane_change_right': int(t['lane_change_right'][i - offset]),
                                       'cut_in_left': int(t['cut_in_left'][i - offset]),
                                       'cut_in_right': int(t['cut_in_right'][i - offset]),
                                       'cut_out_left': int(t['cut_out_left'][i - offset]),
                                       'cut_out_right': int(t['cut_out_right'][i - offset]),
                                       'tail_gate': int(t['tail_gate'][i - offset])}}
                labels.append(label)

        secs = int(1616762501 + i / frame_rate)
        nsecs = int((i / frame_rate) * (10 ** 9))
        to_json = {'index': i, 'image_file_name': scenario['meta']['images'][i],
                   'timestamp_secs': scenario['meta']['timestamp_secs'][i],
                   'timestamp_nsecs': scenario['meta']['timestamp_nsecs'][i],
                   "camera_channel": scenario['meta']['camera_channel'], "sequence": scenario['meta']['sequence'],
                   "time_of_day": "", "weather_type": "SUNNY", 'labels': labels}

        with open(filename + '/' + str(scenario['meta']['timestamp_secs'][i]) + '_' +
                  str(int(scenario['meta']['timestamp_nsecs'][i]) + 1000000000)[1:] + '_' +
                  str(scenario['meta']['camera_channel']) + '.json', 'w') as f:
            json.dump(to_json, f)
