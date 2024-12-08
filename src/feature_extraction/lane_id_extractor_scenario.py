import numpy as np
import config


def extract_lane_id_for_actor(actor):
    actor['lane_id'] = np.array(np.full(len(actor['path']), 0))
    actor['distance_lane_center'] = np.array(np.full(len(actor['path']), 0.0))
    actor['on_shoulder'] = np.array(np.full(len(actor['path']), 0))

    for i in range(len(actor['path'])):
        if config.lane_id_y_values[0] <= actor['path'][i, 1] < config.lane_id_y_values[1]:
            actor['lane_id'][i] = -1
            actor['distance_lane_center'][i] = np.abs(actor['path'][i, 1] - config.distance_center_lane[0])
        elif config.lane_id_y_values[1] <= actor['path'][i, 1] < config.lane_id_y_values[2]:
            actor['lane_id'][i] = -2
            actor['distance_lane_center'][i] = np.abs(actor['path'][i, 1] - config.distance_center_lane[1])
        elif config.lane_id_y_values[2] <= actor['path'][i, 1] < config.lane_id_y_values[3]:
            actor['lane_id'][i] = -3
            actor['distance_lane_center'][i] = np.abs(actor['path'][i, 1] - config.distance_center_lane[2])
        elif config.lane_id_y_values[3] <= actor['path'][i, 1] < config.lane_id_y_values[4]:
            actor['lane_id'][i] = -4
            actor['distance_lane_center'][i] = np.abs(actor['path'][i, 1] - config.distance_center_lane[3])
        elif config.lane_id_y_values[4] <= actor['path'][i, 1] < config.lane_id_y_values[5]:
            if config.lane_id_x_values[0] <= actor['path'][i, 0] < config.lane_id_x_values[1]:
                actor['on_shoulder'][i] = 1
            actor['lane_id'][i] = -5
            actor['distance_lane_center'][i] = np.abs(actor['path'][i, 1] - config.distance_center_lane[4])
        elif config.lane_id_y_values[5] <= actor['path'][i, 1] < config.lane_id_y_values[6]:
            actor['lane_id'][i] = -6
            actor['distance_lane_center'][i] = np.abs(actor['path'][i, 1] - config.distance_center_lane[5])
        elif -config.lane_id_y_values[1] <= actor['path'][i, 1] < -config.lane_id_y_values[0]:
            actor['lane_id'][i] = 1
            actor['distance_lane_center'][i] = np.abs(actor['path'][i, 1] + config.distance_center_lane[0])
        elif -config.lane_id_y_values[2] <= actor['path'][i, 1] < -config.lane_id_y_values[1]:
            actor['lane_id'][i] = 2
            actor['distance_lane_center'][i] = np.abs(actor['path'][i, 1] + config.distance_center_lane[1])
        elif -config.lane_id_y_values[3] <= actor['path'][i, 1] < -config.lane_id_y_values[2]:
            actor['lane_id'][i] = 3
            actor['distance_lane_center'][i] = np.abs(actor['path'][i, 1] + config.distance_center_lane[2])
        elif -config.lane_id_y_values[4] <= actor['path'][i, 1] < -config.lane_id_y_values[3]:
            actor['lane_id'][i] = 4
            actor['distance_lane_center'][i] = np.abs(actor['path'][i, 1] + config.distance_center_lane[3])
        elif -config.lane_id_y_values[5] <= actor['path'][i, 1] < -config.lane_id_y_values[4]:
            actor['lane_id'][i] = 5
            actor['distance_lane_center'][i] = np.abs(actor['path'][i, 1] + config.distance_center_lane[4])
        elif -config.lane_id_y_values[6] <= actor['path'][i, 1] < -config.lane_id_y_values[5]:
            actor['on_shoulder'][i] = 1
            actor['lane_id'][i] = 6
            actor['distance_lane_center'][i] = np.abs(actor['path'][i, 1] + config.distance_center_lane[5])

    return actor


def lane_id_extractor(scenario):
    for t in scenario['actors']:
        extract_lane_id_for_actor(t)

    return scenario
