import config


def detect_traffic_jam_highway_side(scenario, side):
    frame_threshold = scenario['meta']['frame_rate'] * 30
    count_traffic_jam = 0
    count_slow_moving_traffic = 0
    for vel0, vel250 in zip(scenario['meta']['average_velocity_per_frame_' + side + '_0'],
                            scenario['meta']['average_velocity_per_frame_' + side + '_250']):

        # check if the average velocity on a highway side is either below 20km/h or 40km/h for at least 30s
        if vel0 < config.velocity_traffic_jam_threshold and vel250 < config.velocity_traffic_jam_threshold:
            count_traffic_jam += 1
            if count_traffic_jam >= frame_threshold:
                scenario['meta']['traffic_jam_' + side] = 1
        else:
            count_traffic_jam = 0

        if (config.velocity_traffic_jam_threshold < vel0 < config.velocity_slow_moving_traffic_threshold
                and config.velocity_traffic_jam_threshold < vel250 < config.velocity_slow_moving_traffic_threshold):
            count_slow_moving_traffic += 1
            if count_slow_moving_traffic >= frame_threshold:
                scenario['meta']['slow_moving_traffic_' + side] = 1
        else:
            count_slow_moving_traffic = 0

    return scenario


def detect_traffic_jam(scenario):
    scenario = detect_traffic_jam_highway_side(scenario, 'south')
    scenario = detect_traffic_jam_highway_side(scenario, 'north')

    return scenario
