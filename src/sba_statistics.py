import numpy as np


# Description:
# Calculate scenario statistics based on extracted scenarios

def scenario_statistics(scenario, acc_detection_ml, number_of_accidents_ml, msg_count):
    total_vehicles = 0
    total_vehicle_classes = set()
    total_lane_changes_left = 0
    total_lane_changes_right = 0
    max_lane_changes = 0
    total_cut_ins_left = 0
    total_cut_ins_right = 0
    total_cut_outs_left = 0
    total_cut_outs_right = 0
    total_tail_gates_1 = 0
    total_tail_gates_2 = 0
    total_tail_gates_3 = 0
    total_speeding_vehicles = 0
    total_standing_vehicles = 0
    total_standing_vehicles_shoulder = 0
    top_speed = 0
    total_breakdowns_shoulder = 0
    total_breakdowns_driving_lane = 0
    total_breakdowns = 0
    total_accidents = 0

    for t in scenario['actors']:
        # total detected vehicle classes
        total_vehicle_classes.add(t['object_class'][0])

        if np.max(t['velocities']) > top_speed:
            top_speed = np.max(t['velocities'])
        if t['lane_changes_left'] + t['lane_changes_right'] > max_lane_changes:
            max_lane_changes = t['lane_changes_left'] + t['lane_changes_right']
        total_lane_changes_left += t['lane_changes_left']
        total_lane_changes_right += t['lane_changes_right']
        total_cut_ins_left += t['cut_ins_left']
        total_cut_outs_left += t['cut_outs_left']
        total_cut_ins_right += t['cut_ins_right']
        total_cut_outs_right += t['cut_outs_right']

        # Total Tailgating Vehicles
        total_tail_gates_1 += t['tail_gate_minor']
        total_tail_gates_2 += t['tail_gate_moderate']
        total_tail_gates_3 += t['tail_gate_severe']
        # Total Speeding Vehicles
        total_speeding_vehicles += 1 if 1 in t['speeding'] else 0
        # Total Standing Vehicles
        total_standing_vehicles += 1 if 1 in t['standing'] else 0
        # Total Standing Vehicles on shoulder lane
        total_standing_vehicles_shoulder += 1 if 1 in t['standing_shoulder'] else 0

        if acc_detection_ml == 0:
            total_breakdowns_actor_shoulder = t['breakdown_shoulder'][0]
            for i in range(1, len(t['breakdown_shoulder'])):
                if t['breakdown_shoulder'][i] == 1 and t['breakdown_shoulder'][i-1] == 0:
                    total_breakdowns_actor_shoulder += 1
            total_breakdowns_shoulder += int(total_breakdowns_actor_shoulder)

            total_breakdowns_actor_driving_lane = t['breakdown_driving_lane'][0]
            for i in range(1, len(t['breakdown_driving_lane'])):
                if t['breakdown_driving_lane'][i] == 1 and t['breakdown_driving_lane'][i - 1] == 0:
                    total_breakdowns_actor_driving_lane += 1
            total_breakdowns_driving_lane += int(total_breakdowns_actor_driving_lane)

            total_breakdowns_actor = t['breakdown'][0]
            for i in range(1, len(t['breakdown'])):
                if t['breakdown'][i] == 1 and t['breakdown'][i - 1] == 0:
                    total_breakdowns_actor += 1
            total_breakdowns += int(total_breakdowns_actor)

            total_accidents_actor: int = t['in_accident'][0]
            for i in range(1, len(t['in_accident'])):
                if t['in_accident'][i] == 1 and t['in_accident'][i-1] == 0:
                    total_accidents_actor += 1
            total_accidents += int(total_accidents_actor)
        else:
            total_accidents = number_of_accidents_ml

    # total lane changes
    total_lane_changes = total_lane_changes_left + total_lane_changes_right
    # Total Cut Ins
    total_cut_ins = total_cut_ins_left + total_cut_ins_right
    # Total Cut Outs
    total_cut_outs = total_cut_outs_left + total_cut_outs_right

    # total trajectories
    total_trajectories = len(scenario['actors'])

    # total detected vehicles
    total_vehicles = len(scenario['actors'])

    # total detected vehicle classes
    total_vehicle_classes = len(total_vehicle_classes)

    # get the average velocity of the rosbag
    average_velocity_north = scenario['meta']['average_velocity_north']
    average_velocity_south = scenario['meta']['average_velocity_south']

    # get traffic jams or slow moving traffic
    traffic_jam_north = scenario['meta']['traffic_jam_north']
    traffic_jam_south = scenario['meta']['traffic_jam_south']
    slow_moving_traffic_north = scenario['meta']['slow_moving_traffic_north']
    slow_moving_traffic_south = scenario['meta']['slow_moving_traffic_south']

    # level of interest
    level_of_interest = (total_lane_changes + max_lane_changes + total_cut_ins + total_cut_outs + total_tail_gates_1 + \
                        total_tail_gates_2 + total_tail_gates_3 + total_speeding_vehicles + total_standing_vehicles \
                        + round(top_speed) + total_trajectories + 600 * total_accidents + 300 * total_breakdowns + total_vehicles +\
                        average_velocity_north / 2 + average_velocity_south / 2 + 200 * traffic_jam_north + 200 * traffic_jam_south + \
                        10 * slow_moving_traffic_north + 10 * slow_moving_traffic_south) / (msg_count / 25)

    if acc_detection_ml == 0:
        statistics_dict = {'total_vehicles': total_vehicles,
                           'total_vehicle_classes': total_vehicle_classes,
                           'total_lane_changes_left': total_lane_changes_left,
                           'total_lane_changes_right': total_lane_changes_right,
                           'total_lane_changes': total_lane_changes,
                           'max_lane_changes': max_lane_changes, 'total_cut_ins_left': total_cut_ins_left,
                           'total_cut_ins_right': total_cut_ins_right, 'total_cut_ins': total_cut_ins,
                           'total_cut_outs_left': total_cut_outs_left, 'total_cut_outs_right': total_cut_outs_right,
                           'total_cut_outs': total_cut_outs, 'total_tail_gates_1': total_tail_gates_1,
                           'total_tail_gates_2': total_tail_gates_2, 'total_tail_gates_3': total_tail_gates_3,
                           'total_speeding_vehicles': total_speeding_vehicles,
                           'total_standing_vehicles': total_standing_vehicles,
                           'total_standing_vehicles_shoulder': total_standing_vehicles_shoulder,
                           'top_speed': top_speed,
                           'average_velocity_north': average_velocity_north,
                           'average_velocity_south': average_velocity_south,
                           'traffic_jam_north': traffic_jam_north,
                           'traffic_jam_south': traffic_jam_south,
                           'slow_moving_traffic_north': slow_moving_traffic_north,
                           'slow_moving_traffic_south': slow_moving_traffic_south,
                           'total_trajectories': total_trajectories,
                           'level_of_interest': level_of_interest,
                           'total_breakdowns_shoulder': total_breakdowns_shoulder,
                           'total_breakdowns_driving_lane': total_breakdowns_driving_lane,
                           'total_breakdowns': total_breakdowns,
                           'total_accidents': total_accidents
                           }
    else:
        # statistic if learning-based accident detection has been used (can't detect breakdowns)
        statistics_dict = {'total_vehicles': total_vehicles,
                           'total_vehicle_classes': total_vehicle_classes,
                           'total_lane_changes_left': total_lane_changes_left,
                           'total_lane_changes_right': total_lane_changes_right,
                           'total_lane_changes': total_lane_changes,
                           'max_lane_changes': max_lane_changes, 'total_cut_ins_left': total_cut_ins_left,
                           'total_cut_ins_right': total_cut_ins_right, 'total_cut_ins': total_cut_ins,
                           'total_cut_outs_left': total_cut_outs_left, 'total_cut_outs_right': total_cut_outs_right,
                           'total_cut_outs': total_cut_outs, 'total_tail_gates_1': total_tail_gates_1,
                           'total_tail_gates_2': total_tail_gates_2, 'total_tail_gates_3': total_tail_gates_3,
                           'total_speeding_vehicles': total_speeding_vehicles,
                           'total_standing_vehicles': total_standing_vehicles,
                           'total_standing_vehicles_shoulder': total_standing_vehicles_shoulder,
                           'top_speed': top_speed,
                           'average_velocity_north': average_velocity_north,
                           'average_velocity_south': average_velocity_south,
                           'traffic_jam_north': traffic_jam_north,
                           'traffic_jam_south': traffic_jam_south,
                           'slow_moving_traffic_north': slow_moving_traffic_north,
                           'slow_moving_traffic_south': slow_moving_traffic_south,
                           'total_trajectories': total_trajectories,
                           'level_of_interest': level_of_interest,
                           'total_accidents': total_accidents
                           }

    for stat in statistics_dict:
        print(str(stat) + ': ' + str(statistics_dict[stat]))

    return statistics_dict
