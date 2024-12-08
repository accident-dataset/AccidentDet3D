from __future__ import absolute_import
from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon
from shapely.ops import split

import itertools
import numpy as np

# Description:
# This module provides a method to extract scenarios from the live digital twin.

# Define polygons for all 11 lanes on the highway (6 north, 5 south)
lane_neg_1 = Polygon(
    [[-100, 1.8], [220, 1.8], [260, 1.8], [450, 1.8], [450, 5.6],
     [260, 5.6], [220, 5.6], [-100, 5.6], ])  # create polygon
lane_neg_2 = Polygon(
    [[-100, 5.6], [220, 5.6], [260, 5.6], [450, 5.6],
     [450, 9.2], [260, 9.2], [220, 9.2], [-100, 9.2]])  # create polygon
lane_neg_3 = Polygon(
    [[-100, 9.2], [220, 9.2], [260, 9.2], [450, 9.2], [450, 13],
     [260, 13], [220, 13], [-100, 13]])  # create polygon
lane_neg_4 = Polygon(
    [[-100, 13], [220, 13], [260, 13], [450, 13], [450, 17], [260, 17],
     [220, 17], [-100, 17]])  # create polygon
lane_neg_5 = Polygon(
    [[125, 17], [115, 18.1], [105, 20.8], [95, 24.4], [85, 29.8], [75, 36], [65, 43.7], [65, 49.6], [75, 41.4],
     [85, 34.6], [95, 29.2], [105, 25.1], [115, 23], [125, 21], [135, 20.5], [220, 20.5],
     [450, 20.5], [450, 17], [220, 17]])  # create polygon
lane_neg_6 = Polygon(
    [[125, 21], [115, 23], [105, 25.1], [95, 29.2], [85, 34.6], [75, 41.4], [65, 49.6], [65, 55.4], [75, 46.6],
     [85, 40.2], [95, 34.7], [105, 30.5], [115, 27.3], [125, 25.6], [135, 24], [145, 24], [220, 24], [290, 24],
     [320, 22], [365, 20.5], [320, 20.5], [220, 20.5], [135, 20.5]])  # create polygon
lane_neg_5_2 = Polygon([[-100, 17], [-30, 17], [-15, 19.8], [-5, 23.3], [5, 28.5], [15, 36.9], [20, 43.4], [20, 58],
                        [15, 46.7], [5, 35.2], [-5, 29.5], [-15, 26.4], [-40, 21.5], [-100, 20.5]])
lane_1 = Polygon(
    [[-100, -6.9], [0, -6.9], [120, -6.9], [350, -6.9], [450, -6.9], [450, -3],
     [350, -3], [120, -3], [0, -3], [-100, -3]])  # create polygon
lane_2 = Polygon(
    [[-100, -10.7], [0, -10.7], [120, -10.7], [350, -10.7], [450, -10.7], [450, -6.9],
     [350, -6.9], [120, -6.9], [0, -6.9], [-100, -6.9]])  # create polygon
lane_3 = Polygon(
    [[-100, -14.5], [8.26, -14.5], [120, -14.5], [350, -14.5], [450, -14.5], [450, -10.7],
     [350, -10.7], [120, -10.7], [0, -10.7], [-100, -10.7]])  # create polygon
lane_4 = Polygon(
    [[-100, -18.3], [0, -18.3], [120, -18.3], [350, -18.3], [450, -18.3], [450, -14.5],
     [350, -14.5], [120, -14.5], [0, -14.5], [-100, -14.5]])  # create polygon
lane_5 = Polygon(
    [[-100, -22.1], [0, -22.1], [120, -22.1], [350, -22.1], [450, -22.1], [450, -18.3],
     [350, -18.3], [120, -18.3], [0, -18.3], [-100, -18.3]])  # create polygon

polygons = [[lane_neg_1, -1], [lane_neg_2, -2], [lane_neg_3, -3], [lane_neg_4, -4], [lane_neg_5, -5],
            [lane_neg_6, -6], [lane_neg_5_2, -5],
            [lane_1, 1], [lane_2, 2], [lane_3, 3], [lane_4, 4], [lane_5, 5]]

# Define Lane Center Lines
lane_neg_1_center = LineString(traj_straight_neg1())
lane_neg_2_center = LineString(traj_straight_neg2())
lane_neg_3_center = LineString(traj_straight_neg3())
lane_neg_4_center = LineString(traj_straight_neg4())
lane_neg_5_center = LineString(traj_exit2())
lane_neg_6_center = LineString(traj_exit())

lane_1_center = LineString(traj_straight1())
lane_2_center = LineString(traj_straight2())
lane_3_center = LineString(traj_straight3())
lane_4_center = LineString(traj_straight4())
lane_5_center = LineString(traj_straight5())

lane_center_lines = {1: lane_1_center, 2: lane_2_center, 3: lane_3_center, 4: lane_4_center, 5: lane_5_center,
                     -1: lane_neg_1_center, -2: lane_neg_2_center, -3: lane_neg_3_center, -4: lane_neg_4_center,
                     -5: lane_neg_5_center, -6: lane_neg_6_center, 0: LineString(traj_straight_neg2())}


def extractor(objects):
    object_dict = {}
    for object in objects:
        object_dict[object.object_ID] = {}
        object_dict[object.object_ID]["track_id"] = object.object_ID
        object_dict[object.object_ID]["lane_id"] = 0
        object_dict[object.object_ID]["speed"] = np.linalg.norm(object.speed)
        object_dict[object.object_ID]["distance_lead"] = -1
        object_dict[object.object_ID]["distance_following"] = -1
        object_dict[object.object_ID]["velocity_lead"] = -1
        object_dict[object.object_ID]["velocity_following"] = -1
        object_dict[object.object_ID]["speeding"] = 0
        object_dict[object.object_ID]["standing"] = 0
        object_dict[object.object_ID]["tailgate1"] = 0
        object_dict[object.object_ID]["tailgate2"] = 0
        object_dict[object.object_ID]["tailgate3"] = 0
        object_dict[object.object_ID]["position"] = object.position

        point = Point(object_dict[object.object_ID]["position"][0],
                      object_dict[object.object_ID]["position"][1])  # create point

        for j in polygons:
            if j[0].contains(point) or point.within(j[0]):
                object_dict[object.object_ID]["lane_id"] = j[1]

        object_dict[object.object_ID]['distance_lane_center'] = point.distance(
            lane_center_lines[object_dict[object.object_ID]['lane_id']]
        )

    point_dict = {}

    lane = {1: {}, 2: {}, 3: {}, 4: {}, 5: {}, -1: {}, -2: {}, -3: {}, -4: {}, -5: {}, -6: {}}
    for key in object_dict.keys():
        if object_dict[key]["lane_id"] != 0:
            point_dict[key] = {'point': object_dict[key]["position"][0:2],
                               'velocity': np.linalg.norm(object_dict[key]["speed"])}
            lane[object_dict[key]["lane_id"]][key] = {'distance': {'lead': [], 'follow': []},
                                                      'velocity': {'lead': [], 'follow': []}}

    for lane_id in lane:
        vehicle_combinations = list(itertools.combinations(lane[lane_id], 2))
        line = lane_center_lines[lane_id]

        for pair in vehicle_combinations:
            point1 = line.interpolate(line.project(Point(point_dict[pair[0]]['point']))).buffer(0.0000001)
            point2 = line.interpolate(line.project(Point(point_dict[pair[1]]['point']))).buffer(0.0000001)
            length_1 = split(line, point1)[0]
            length_2 = split(line, point2)[0]

            distance_1 = np.sum(np.sqrt(np.sum(np.diff(length_1, axis=0) ** 2, axis=1)))
            distance_2 = np.sum(np.sqrt(np.sum(np.diff(length_2, axis=0) ** 2, axis=1)))
            distance = distance_1 - distance_2

            if distance >= 0:
                lane[lane_id][pair[0]]['distance']['follow'].append(abs(distance))
                lane[lane_id][pair[1]]['distance']['lead'].append(abs(distance))
                lane[lane_id][pair[0]]['velocity']['follow'].append(point_dict[pair[1]]['velocity'])
                lane[lane_id][pair[1]]['velocity']['lead'].append(point_dict[pair[0]]['velocity'])
            else:
                lane[lane_id][pair[0]]['distance']['lead'].append(abs(distance))
                lane[lane_id][pair[1]]['distance']['follow'].append(abs(distance))
                lane[lane_id][pair[0]]['velocity']['lead'].append(point_dict[pair[0]]['velocity'])
                lane[lane_id][pair[1]]['velocity']['follow'].append(point_dict[pair[0]]['velocity'])

        for index in lane[lane_id]:
            if len(lane[lane_id][index]['distance']['lead']) > 0:
                index2 = np.argmin(np.array(lane[lane_id][index]['distance']['lead']))
                object_dict[index]["distance_lead"] = lane[lane_id][index]['distance']['lead'][index2]
                object_dict[index]["velocity_lead"] = lane[lane_id][index]['velocity']['lead'][index2]

            if len(lane[lane_id][index]['distance']['follow']) > 0:
                index2 = np.argmin(np.array(lane[lane_id][index]['distance']['follow']))
                object_dict[index]["distance_following"] = lane[lane_id][index]['distance']['follow'][index2]
                object_dict[index]['velocity_following'] = lane[lane_id][index]['velocity']['follow'][index2]

    for key in object_dict.keys():
        object = object_dict[key]
        if object["speed"] * 3.6 * 0.5 > object["distance_lead"] != -1:  # Tailgate
            object["tailgate1"] = 1
        if 100 > object["speed"] * 3.6 >= 80 \
                and object["speed"] * 3.6 * 0.5 * 0.5 > object["distance_lead"] != -1:  # Tailgate
            object["tailgate2"] = 1
        if object["speed"] * 3.6 >= 100 \
                and object["speed"] * 3.6 * 0.5 * 0.5 > object["distance_lead"] >= object["speed"] * 3.6 * \
                0.5 * 0.3 and object["distance_lead"] != -1:  # Tailgate
            object["tailgate2"] = 1
        if object["speed"] * 3.6 >= 100 \
                and object["speed"] * 3.6 * 0.5 * 0.3 > object["distance_lead"] != -1:  # Tailgate
            object["tailgate3"] = 1

        if object["speed"] * 3.6 > 130:  # Speeding
            object["speeding"] = 1

        if object["speed"] < 0.05:  # Standing
            object["standing"] = 1

    return object_dict
