import random
import datetime
import time
from scipy.interpolate import UnivariateSpline
from scipy.ndimage.filters import uniform_filter1d
import numpy as np


# Description:
# This module variates scenarios based on gaussian noise. Outliers are being removed and the path/trajectory of vehicles is smoothed.
# The velocity is also smoothed and interpolated here. Vehicles can be randomly varied based on a vehicle catalog.

def gaussian_noise(input, stanarddeviation=0.0, mean=0.0):
    # Gaussian noise
    noise = np.random.normal(mean, stanarddeviation, input.shape)
    output = input + noise
    return output


def offset_absolut(path, x_offset, y_offset):
    path[:, 0] += x_offset
    path[:, 1] += y_offset
    return path


def offset_relative(path, factor=0.0):
    alpha = range(0, len(path))

    interpolator = [UnivariateSpline(alpha, coords, k=3, s=0) for coords in path.T]
    path = np.stack([spl(alpha) for spl in interpolator], axis=0).T

    derivatives = [spl.derivative() for spl in interpolator]
    derivatives = np.stack([spl(alpha) for spl in derivatives], axis=0).T
    normal_vectors = np.array([derivatives[:, 1] * -1, derivatives[:, 0]]).T
    normal_vectors = normal_vectors / np.array([np.linalg.norm(normal_vectors, axis=1)]).T
    path = path + factor * normal_vectors
    return path


def path_smoothing(path, smoothing_factor=0.0):
    alpha = range(0, len(path))

    interpolator = [UnivariateSpline(alpha, coords, k=3, s=smoothing_factor) for coords in path.T]
    path = np.stack([spl(alpha) for spl in interpolator], axis=0).T
    return path


def path_smoothing_outlier_removal(path, time_stamps, smoothing_factor=0.0, n=0):
    angles = np.arctan2(np.diff(path[:, 1], axis=0), np.diff(path[:, 0], axis=0))
    angles = np.insert(angles, -1, angles[-1])
    difference = np.absolute(np.absolute(angles[:-1]) - np.absolute(angles[1:]))
    indices = np.where(difference[:-1] + difference[1:] > 4)[0]

    padded_indices = []
    for i in indices:
        padded_indices += list(range(i - n, i + n))
    padded_indices = [x for x in padded_indices if 0 <= x <= len(padded_indices)]
    indices = [x for x in list(range(0, len(path))) if x not in padded_indices]

    new_time_stamps = time_stamps[indices]
    new_path = path[indices]

    interpolator = [UnivariateSpline(new_time_stamps, coords, k=3, s=smoothing_factor) for coords in new_path.T]
    path1 = np.stack([spl(time_stamps) for spl in interpolator], axis=0).T

    return path1


def velocities_smoothing(velocities, time_stamps, frame_rate, smoothing_factor=0.0, moving_average=1):
    indices1 = np.where(velocities < 100)[0]
    indices = np.where(np.absolute(velocities - uniform_filter1d(velocities, size=2)) < 25 / frame_rate)[0]
    indices = np.concatenate((indices1, indices))
    outliers = np.array([x for x in list(range(0, len(velocities))) if x not in indices])

    if len(outliers) != 0:
        differences = (outliers.reshape(1, -1) - indices.reshape(-1, 1))
        outlier_corrected = indices[np.abs(differences).argmin(axis=0)]

        velocities[outliers] = velocities[outlier_corrected]
        velocities = uniform_filter1d(velocities, size=moving_average)

    interpolator = UnivariateSpline(time_stamps, velocities, k=3, s=smoothing_factor)
    new_velocities = interpolator(time_stamps)

    return uniform_filter1d(new_velocities, size=moving_average)


def interpolate_velocities(path, velocities, frame_rate):
    new_path = path[np.where(velocities > 0.001)]
    # Linear length along the line:
    distance = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
    distance = np.insert(distance, 0, 0)

    new_distance = np.cumsum(np.sqrt(np.sum(np.diff(new_path, axis=0) ** 2, axis=1)))
    new_distance = np.insert(new_distance, 0, 0)

    # Interpolation for different methods:
    dist_vel = np.cumsum(velocities / frame_rate)[:-1]
    mask = np.abs(dist_vel) <= distance[-1]
    dist_vel = dist_vel[mask]
    alpha = np.insert(dist_vel, 0, 0)

    interpolator = [UnivariateSpline(new_distance, coords, k=3, s=0) for coords in new_path.T]
    path = np.stack([spl(alpha) for spl in interpolator], axis=0).T

    return path


def random_RGB():
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    return r, g, b


def random_time():
    min_time = datetime.datetime(2020, 1, 1, 0, 0, 0)
    max_time = datetime.datetime.now()

    mintime_ts = int(time.mktime(min_time.timetuple()))
    maxtime_ts = int(time.mktime(max_time.timetuple()))

    random_ts = random.randint(mintime_ts, maxtime_ts)
    random_time = datetime.datetime.fromtimestamp(random_ts)
    return random_time


def random_vehicle(vehicle_type):
    vehicles = ['vehicle.audi.a2', 'vehicle.audi.etron', 'vehicle.audi.tt', 'vehicle.bh.crossbike',
                'vehicle.bmw.grandtourer', 'vehicle.carlamotors.carlacola', 'vehicle.carlamotors.firetruck',
                'vehicle.chevrolet.impala', 'vehicle.citroen.c3', 'vehicle.diamondback.century',
                'vehicle.dodge.charger_2020', 'vehicle.dodge.charger_police', 'vehicle.dodge.charger_police_2020',
                'vehicle.ford.ambulance', 'vehicle.ford.mustang', 'vehicle.gazelle.omafiets',
                'vehicle.harley-davidson.low_rider', 'vehicle.jeep.wrangler_rubicon', 'vehicle.kawasaki.ninja',
                'vehicle.lincoln.mkz_2017', 'vehicle.lincoln.mkz_2020', 'vehicle.mercedes.coupe',
                'vehicle.mercedes.coupe_2020', 'vehicle.mercedes.sprinter', 'vehicle.micro.microlino',
                'vehicle.mini.cooper_s', 'vehicle.mini.cooper_s_2021', 'vehicle.nissan.micra', 'vehicle.nissan.patrol',
                'vehicle.nissan.patrol_2021', 'vehicle.seat.leon', 'vehicle.tesla.cybertruck', 'vehicle.tesla.model3',
                'vehicle.toyota.prius', 'vehicle.vespa.zx125', 'vehicle.volkswagen.t2', 'vehicle.yamaha.yzf']
    index = random.randint(0, len(vehicles) - 1)
    make = vehicles[index]

    if any(s in vehicle_type for s in ['CAR', 'Car', 'car']):
        cars = ['vehicle.audi.a2', 'vehicle.audi.etron', 'vehicle.audi.tt', 'vehicle.bmw.grandtourer',
                'vehicle.chevrolet.impala', 'vehicle.citroen.c3', 'vehicle.dodge.charger_2020',
                'vehicle.dodge.charger_police', 'vehicle.dodge.charger_police_2020', 'vehicle.ford.mustang',
                'vehicle.jeep.wrangler_rubicon', 'vehicle.lincoln.mkz_2017', 'vehicle.lincoln.mkz_2020',
                'vehicle.mercedes.coupe', 'vehicle.mercedes.coupe_2020', 'vehicle.mercedes.sprinter',
                'vehicle.mini.cooper_s', 'vehicle.mini.cooper_s_2021', 'vehicle.nissan.micra', 'vehicle.nissan.patrol',
                'vehicle.nissan.patrol_2021', 'vehicle.seat.leon', 'vehicle.tesla.model3', 'vehicle.toyota.prius',
                'vehicle.volkswagen.t2']
        index = random.randint(0, len(cars) - 1)
        make = cars[index]

    if any(s in vehicle_type for s in ['MOTORCYCLE', 'CYCLIST', 'Cyclist', 'cyclist', 'Bike', 'Motorcyclist']):
        bikes = ['vehicle.bh.crossbike', 'vehicle.diamondback.century', 'vehicle.gazelle.omafiets',
                 'vehicle.harley-davidson.low_rider', 'vehicle.kawasaki.ninja', 'vehicle.micro.microlino',
                 'vehicle.vespa.zx125', 'vehicle.yamaha.yzf']
        index = random.randint(0, len(bikes) - 1)
        make = bikes[index]

    if any(s in vehicle_type for s in ['CAR', 'Car', 'car']):
        trucks = ['vehicle.carlamotors.carlacola', 'vehicle.carlamotors.firetruck', 'vehicle.ford.ambulance',
                  'vehicle.mercedes.sprinter', 'vehicle.tesla.cybertruck']
        index = random.randint(0, len(trucks) - 1)
        make = trucks[index]

    return make


def run_variation(scenario, n):
    new_scenarios = []
    for i in range(n):
        new_scenario = scenario.copy()
        new_actors = []
        for actor in scenario['actors']:
            new_actor = {}
            new_actor['id'] = actor['id']
            new_actor['category'] = actor['category']
            new_actor['make'] = random_vehicle(str(actor['category']))
            new_actor['color'] = np.array(random_RGB())
            new_actor['velocities'] = actor['velocities']
            new_actor['offset'] = actor['offset']
            new_actor['frame_rate'] = actor['frame_rate']
            new_actor['time_stamp'] = actor['time_stamp']

            path = actor['path'][:, :2]
            _, index = np.unique(path, axis=0, return_index=True)
            reduced_path = path[np.sort(index)]
            new_actor['path'] = path

            if len(reduced_path) == 1:
                new_actor['path'] = np.zeros((len(actor['time_stamp']), 4))
                new_actor['path'][:, :2] = np.full((len(actor['time_stamp']), 2), gaussian_noise(reduced_path, 0, 0))

            if len(reduced_path) == len(path):
                offset = offset_relative(path, np.random.normal(0, 0.2, 1))
                noisy = gaussian_noise(offset, 0.3, 0)
                smooth = path_smoothing(noisy, 50)
                complete = interpolate_velocities(smooth, actor['velocities'], actor['frame_rate'])

                z = np.array([np.full(len(complete), 0)])
                angles = np.arctan2(np.diff(complete[:, 1], axis=0), np.diff(complete[:, 0], axis=0))
                angles = np.array([np.insert(angles, -1, angles[-1])])
                new_actor['path'] = np.concatenate((complete, z.T, angles.T), axis=1)

                new_actor['velocities'] = actor['velocities'][:len(new_actor['path'])]
                new_actor['time_stamp'] = actor['time_stamp'][:len(new_actor['path'])]

                new_actors.append(new_actor)

        new_scenario['actors'] = new_actors
        new_scenarios.append(new_scenario)

    return new_scenarios
