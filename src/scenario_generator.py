from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt
import scenario_variation


# Description:
# This module interpolates the position velocity of the digital twin to get a higher accuracy in the scenario variation.

def interpolator(id, category, path, velocities, frame_rate, time_offset=0, distance_offset=0):
    # Linear length along the line:
    distance = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
    distance = np.insert(distance, 0, 0)

    # Calculate distance travelled at each time step
    dist_vel = np.cumsum(velocities / frame_rate)
    dist_vel = np.insert(dist_vel, 0, 0)
    dist_vel += distance_offset
    mask = np.abs(dist_vel) <= distance[-1]
    dist_vel = dist_vel[mask]
    alpha = dist_vel

    actor = {}
    interpolator = interp1d(distance, path, kind='cubic', axis=0)
    trajectory = interpolator(alpha)
    actor['offset'] = time_offset / frame_rate
    actor['frame_rate'] = frame_rate
    velocities = np.array(np.insert(velocities, -1, velocities[-1]))
    actor['velocities'] = velocities[:len(trajectory)]
    actor['category'] = category
    actor['make'] = scenario_variation.random_vehicle(str(category))
    actor['color'] = np.array(scenario_variation.random_RGB())
    actor['id'] = id
    actor['time_stamp'] = np.array(range(time_offset, len(alpha) + time_offset)) / frame_rate
    angles = np.arctan2(np.diff(trajectory[:, 1], axis=0), np.diff(trajectory[:, 0], axis=0))
    angles = np.array([np.insert(angles, -1, angles[-1])])
    z = np.array([np.full(len(trajectory), 0)])
    actor['path'] = np.concatenate((trajectory, z.T, angles.T), axis=1)

    return actor


def interpolator2(id, category, path_time, frame_rate, start_vel, time_offset=0, distance_offset=0):
    # Linear length along the line:
    distance = np.cumsum(np.sqrt(np.sum(np.diff(path_time[:, 0:2], axis=0) ** 2, axis=1)))
    distance = np.insert(distance, 0, 0)
    distance += 10e-15

    path_time[:, 2] -= path_time[0, 2]
    velocities = np.sqrt(np.sum(np.diff(path_time[:, 0:2], axis=0) ** 2, axis=1)) / np.diff(path_time[:, 2])
    velocities = np.insert(velocities, 0, start_vel)

    interpolator_dist = interp1d(path_time[:, 2], velocities, kind='linear', axis=0)
    velocities = interpolator_dist(np.linspace(0, (path_time[-1, 2] - path_time[0, 2]),
                                               int((path_time[-1, 2] - path_time[0, 2]) * frame_rate)))

    fig, ax = plt.subplots()
    ax.plot(velocities, '-')
    plt.show()

    # Interpolation for different methods:
    dist_vel = np.cumsum(velocities / frame_rate)
    dist_vel += distance_offset
    mask = np.abs(dist_vel) <= distance[-1]
    dist_vel = dist_vel[mask]
    alpha = dist_vel

    actor = {}
    interpolator = interp1d(distance, path_time[:, 0:2], kind='cubic', axis=0)
    trajectory = interpolator(alpha)
    actor['offset'] = time_offset
    actor['frame_rate'] = frame_rate
    actor['velocities'] = velocities
    actor['category'] = 'CAR'
    actor['make'] = scenario_variation.random_vehicle(str(category))
    actor['color'] = np.array(scenario_variation.random_RGB())
    actor['id'] = id
    actor['time_stamp'] = np.array(range(time_offset, len(alpha) + time_offset)) / frame_rate
    angles = np.arctan2(np.diff(trajectory[:, 1], axis=0), np.diff(trajectory[:, 0], axis=0))
    angles = np.array([np.insert(angles, -1, angles[-1])])
    z = np.array([np.full(len(trajectory), 0)])
    actor['path'] = np.concatenate((trajectory, z.T, angles.T), axis=1)

    return actor
