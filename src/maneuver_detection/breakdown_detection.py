import numpy as np
import config

from extract_images_of_event import extract_images


def detect_breakdown_for_actor(scenario, actor, extract_imgs, input_path, output_path, output_base_path,
                              event_counter, frame_rate):
    actor['breakdown_shoulder'] = np.array(np.full(len(actor['path']), 0))
    actor['breakdown_driving_lane'] = np.array(np.full(len(actor['path']), 0))
    actor['breakdown'] = np.array(np.full(len(actor['path']), 0))

    frame_threshold = frame_rate * config.breakdown_time_threshold
    count_shoulder = 0
    count_driving_lane = 0
    driving_side = 'north' if actor['path'][0][1] > 0 else 'south'
    for i in range(len(actor['path'])):
        # check if vehicle stands on shoulder lane for longer than 30s
        if actor['standing_shoulder'][i] == 1:
            count_shoulder += 1
            if count_shoulder >= frame_threshold:
                actor['breakdown_shoulder'][i] = 1

            count_driving_lane = 0
        else:
            count_shoulder = 0

            # check if vehicle stands on a driving lane for longer than 30s
            if actor['standing'][i] == 1:
                count_driving_lane += 1
                if count_driving_lane >= frame_threshold:
                    # check if the avg velocity on that side is below 20km/h
                    position = '_0' if actor['path'][i][0] < 250 else '_250'
                    if scenario['meta']['average_velocity_per_frame_' + driving_side + position][
                            int(actor['time_stamp'][i] * frame_rate)] > config.velocity_traffic_jam_threshold:
                        actor['breakdown_driving_lane'][i] = 1
            else:
                count_driving_lane = 0

    # combine all three rule based approaches into one
    actor['breakdown'] = np.maximum(actor['breakdown_shoulder'], actor['breakdown_driving_lane'])

    if extract_imgs == 3 or extract_imgs == 4:
        event_counter = extract_images(actor, 'breakdown', input_path, output_path, output_base_path, event_counter)
    return event_counter


def breakdown_detection(scenario, extract_imgs, input_path, output_path, output_base_path):
    event_counter = 0
    frame_rate = scenario['meta']['frame_rate']
    for actor in scenario['actors']:
        event_counter = detect_breakdown_for_actor(scenario, actor, extract_imgs, input_path, output_path,
                                                   output_base_path, event_counter, frame_rate)
    return scenario
