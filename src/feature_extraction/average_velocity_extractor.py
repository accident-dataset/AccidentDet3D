from statistics import mean
import numpy as np


def calculate_average_velocity(scenario):
    # create an array containing all velocities of vehicle driving on towards north and one towards south direction
    velocities_north = [velocity for actor in scenario['actors'] for velocity in actor['velocities'] if
                        actor['path'][0][1] > 0]
    velocities_south = [velocity for actor in scenario['actors'] for velocity in actor['velocities'] if
                        actor['path'][0][1] < 0]
    scenario['meta']['average_velocity_north'] = mean(velocities_north)
    scenario['meta']['average_velocity_south'] = mean(velocities_south)
    return scenario


def calculate_average(array):
    return np.asarray(
        [mean(single_frame_velocities) if len(single_frame_velocities) != 0 else 0.0
         for single_frame_velocities in array])


def calculate_average_velocity_per_frame(scenario):
    # create a lists for all north/south split into [0,250] and [250,500] containing a list of velocities in each frame
    velocities_per_frame_north_0 = [[] for _ in range(scenario['meta']['num_frames'])]
    velocities_per_frame_north_250 = [[] for _ in range(scenario['meta']['num_frames'])]
    velocities_per_frame_south_0 = [[] for _ in range(scenario['meta']['num_frames'])]
    velocities_per_frame_south_250 = [[] for _ in range(scenario['meta']['num_frames'])]
    frame_rate = scenario['meta']['frame_rate']

    for actor in scenario['actors']:
        # check if vehicle is on north or south side
        if actor['path'][0][1] > 0:
            # north side
            for timestamp, velocity, path in zip(actor['time_stamp'], actor['velocities'], actor['path']):
                # check if it is between [0, 250] or [250, 500]
                if path[0] < 250:
                    velocities_per_frame_north_0[int(timestamp * frame_rate)].append(velocity)
                else:
                    velocities_per_frame_north_250[int(timestamp * frame_rate)].append(velocity)
        else:
            # south side
            for timestamp, velocity, path in zip(actor['time_stamp'], actor['velocities'], actor['path']):
                # check if it is between [0, 250] or [250, 500]
                if path[0] < 250:
                    velocities_per_frame_south_0[int(timestamp * frame_rate)].append(velocity)
                else:
                    velocities_per_frame_south_250[int(timestamp * frame_rate)].append(velocity)

    scenario['meta']['average_velocity_per_frame_south_0'] = calculate_average(velocities_per_frame_south_0)
    scenario['meta']['average_velocity_per_frame_south_250'] = calculate_average(velocities_per_frame_south_250)
    scenario['meta']['average_velocity_per_frame_north_0'] = calculate_average(velocities_per_frame_north_0)
    scenario['meta']['average_velocity_per_frame_north_250'] = calculate_average(velocities_per_frame_north_250)

    return scenario
