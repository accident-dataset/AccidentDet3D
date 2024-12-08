import numpy as np
import config

from extract_images_of_event import extract_images


# Description
# This module provides the implementation for a speeding and standing maneuver detection.

def detect_speeding_standing_for_actor(actor, extract_imgs, input_path, output_path, output_base_path, event_counter):
    actor['speeding'] = np.array(np.full(len(actor['path']), 0))
    actor['standing'] = np.array(np.full(len(actor['path']), 0))
    actor['standing_shoulder'] = np.array(np.full(len(actor['path']), 0))

    actor['speeding'][np.where(actor['velocities'] > config.speedlimit)] = 1
    actor['standing'][np.where(actor['velocities'] < config.standingThreshold)] = 1
    actor['standing_shoulder'][np.where(actor['standing'] == 1)] = 1
    actor['standing_shoulder'][np.where(actor['on_shoulder'] == 1)] += 1

    # check if both conditions for standing shoulder are met
    actor['standing_shoulder'][np.where(actor['standing_shoulder'] != 2)] = 0
    actor['standing_shoulder'][np.where(actor['standing_shoulder'] == 2)] = 1

    indices = np.flatnonzero(np.diff(np.r_[0, actor['speeding'], 0]) != 0).reshape(-1, 2) - [0, 1]
    speeding_length = np.diff(indices, axis=1).flatten()
    indices = indices[np.where(speeding_length < actor['frame_rate'] * 1)[0]]
    for i in range(len(indices)):
        actor['speeding'][indices[i, 0]:indices[i, 1] + 1] = 0

    indices = np.flatnonzero(np.diff(np.r_[0, actor['standing'], 0]) != 0).reshape(-1, 2) - [0, 1]
    standing_length = np.diff(indices, axis=1).flatten()
    indices = indices[np.where(standing_length < actor['frame_rate'] * 1)[0]]
    for i in range(len(indices)):
        actor['standing'][indices[i, 0]:indices[i, 1] + 1] = 0

    indices = np.flatnonzero(np.diff(np.r_[0, actor['standing_shoulder'], 0]) != 0).reshape(-1, 2) - [0, 1]
    standing_shoulder_length = np.diff(indices, axis=1).flatten()
    indices = indices[np.where(standing_shoulder_length < actor['frame_rate'] * 1)[0]]
    for i in range(len(indices)):
        actor['standing_shoulder'][indices[i, 0]:indices[i, 1] + 1] = 0

    # extract images for standing shoulder event and return number of standing shoulder events detected in rosbag
    if extract_imgs == 1:
        event_counter = extract_images(actor, 'standing_shoulder', input_path, output_path, output_base_path, event_counter)

    return event_counter


def speeding_standing(scenario, extract_imgs, input_path, output_path, output_base_path):
    event_counter = 0

    # perform speeding_standing detection for each actor
    for actor in scenario['actors']:
        event_counter = detect_speeding_standing_for_actor(actor, extract_imgs, input_path, output_path, output_base_path, event_counter)
    return scenario
