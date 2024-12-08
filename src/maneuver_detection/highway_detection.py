import numpy as np
from shapely.geometry.polygon import Polygon

# Description
# Provides the implementation for speeding and standing maneuver detection.

polygons = []
tests = []

# start of the maneuver (before exiting the highway)
box101 = Polygon(
    [[125, 17], [115, 18.1], [105, 20.8], [115, 27.3], [125, 25.6], [135, 24], [145, 24], [220, 24], [290, 24],
     [320, 22], [365, 20.5], [450, 20.5], [450, 17], [220, 17]])
polygons.append(box101)

# end of the maneuver (after taking the highway exit)
box102 = Polygon(
    [[29, 92], [40, 92], [57, 65], [65, 55.4], [75, 46.6], [85, 40.2], [95, 34.7], [105, 30.5], [115, 27.3],
     [105, 20.8], [95, 24.4], [85, 29.8], [73, 36], [65, 43.7], [58, 50], [47, 60]])
polygons.append(box102)

tests.append([[box101, 101, 'start'], [box102, 102, 'end'], ['exit highway']])


def speeding(actor):
    allowed_speed = 130 / 3.6
    speeding_indices = np.where(actor['velocities'] > allowed_speed)
    speeding_array = np.zeros(actor['path'][:, 0].shape)
    speeding_array[speeding_indices] = 1
    actor['speeding'] = speeding_array


def standing(actor):
    standing_indices = np.where(actor['velocities'] < 0.01)
    standing_array = np.zeros(actor['path'][:, 0].shape)
    standing_array[standing_indices] = 1
    actor['standing'] = standing_array
