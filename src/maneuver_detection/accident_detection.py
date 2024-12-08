import numpy as np
import config

from extract_images_of_event import extract_images


def detect_accident_for_actor(actor, extract_imgs, input_path, output_path, output_base_path,
                              event_counter, frame_rate):
    actor['near_collision'] = np.array(np.full(len(actor['path']), 0))
    actor['in_accident'] = np.array(np.full(len(actor['path']), 0))

    actor['near_collision'][np.where(actor["ttc_leading"] <= config.ttc_threshold)] = 1
    actor['near_collision'][np.where(actor["ttc_leading"] < 0)] = 0

    # check if accident with the car in front is happening
    actor['in_accident'][np.where(actor['velocities'] >= config.velocity_accident_threshold)] += 1

    # filter out accidents with false positive, since their bounding box often overlaps with a vehicle
    actor['in_accident'][np.where(actor['distance_lead'] >= config.distance_FP_threshold)] += 1
    # instead of taking the sqrt in distance lead, square the other side of the equation
    actor['in_accident'][np.where(actor['distance_lead'] <
                                  ((actor['velocities'] - actor['velocity_lead']) / config.distance_divisor)
                                  ** 2)] += 1
    actor['in_accident'][np.where(actor['velocity_lead'] < actor['velocities'])] += 1

    # check if all conditions above are met
    actor['in_accident'][np.where(actor['in_accident'] != 4)] = 0
    actor['in_accident'][np.where(actor['in_accident'] == 4)] = 1
    actor['in_accident'][np.where(actor['near_collision'] != 1)] = 0

    # check if the velocity after the accident is not increasing
    indices = np.where(actor['in_accident'] == 1)[0]
    for index in indices:
        velocity_before = actor['velocities'][index]
        for i in range(index + 1, len(actor['in_accident'])):
            if velocity_before < actor['velocities'][i]:
                actor['in_accident'][index] = 0
                break
            velocity_before = actor['velocities'][i]

    # extract images for accident and return number of accidents detected in rosbag
    if extract_imgs == 2 or extract_imgs == 4:
        event_counter = extract_images(actor, 'in_accident', input_path, output_path, output_base_path, event_counter)

    return event_counter


def accident_detection(scenario, extract_imgs, input_path, output_path, output_base_path):
    event_counter = 0
    frame_rate = scenario['meta']['frame_rate']

    # perform accident detection for each actor
    for actor in scenario['actors']:
        event_counter = detect_accident_for_actor(actor, extract_imgs, input_path, output_path,
                                                  output_base_path, event_counter, frame_rate)
    return scenario
