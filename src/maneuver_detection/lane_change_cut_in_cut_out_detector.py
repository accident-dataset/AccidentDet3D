import numpy as np


# Description
# This module provides the implementation for a lane change detection.

def detect_cut_in_and_lane_change_for_actor(actor):
    actor['lane_changes_left'] = 0
    actor['lane_changes_right'] = 0
    actor['cut_ins_left'] = 0
    actor['cut_outs_left'] = 0
    actor['cut_ins_right'] = 0
    actor['cut_outs_right'] = 0
    actor['lane_change_left'] = np.full(actor['path'][:, 0].shape, 0)
    actor['lane_change_right'] = np.full(actor['path'][:, 0].shape, 0)
    actor['cut_in_left'] = np.full(actor['path'][:, 0].shape, 0)
    actor['cut_in_right'] = np.full(actor['path'][:, 0].shape, 0)
    actor['cut_out_left'] = np.full(actor['path'][:, 0].shape, 0)
    actor['cut_out_right'] = np.full(actor['path'][:, 0].shape, 0)

    lane_crossings = np.where(actor['lane_id'][:-1] != actor['lane_id'][1:])[0] + 1

    close_to_center = np.where(actor['distance_lane_center'] < 1)[0]

    start_save = -1
    end_save = -1
    for lane_id_crossing in lane_crossings:
        if close_to_center.size != 0:
            start = np.where(close_to_center - lane_id_crossing < 0)[0]
            end = np.where(close_to_center - lane_id_crossing >= 0)[0]

            if start.size != 0 and end.size != 0:
                start = close_to_center[start.max()]
                end = close_to_center[end.min()]

                if start != start_save and end != end_save:
                    start_save = start
                    end_save = end
                    if actor['lane_id'][start] != actor['lane_id'][end] \
                            and abs(actor['lane_id'][start] - actor['lane_id'][end]) == 1:
                        if abs(actor['lane_id'][start]) - abs(actor['lane_id'][end]) > 0:
                            actor['lane_change_left'][start:(end + 1)] = 1
                            actor['lane_changes_left'] += 1

                            distance_following = actor['distance_following'][int(lane_id_crossing):(end + 1)]
                            velocity_following = actor['velocity_following'][int(lane_id_crossing):(end + 1)]
                            ttc = actor['ttc_following'][int(lane_id_crossing):(end + 1)]
                            include = np.where(distance_following >= 0)

                            # if len(np.where(distance_following[include] - (velocity_following[include] * 1.8) < 0)[0]) != 0:
                            if len(np.where((ttc[include] < 2) & (ttc[include] > 0.01))[0]) != 0:
                                actor['cut_in_left'][start:(end + 1)] = 1
                                actor['cut_ins_left'] += 1

                            distance_lead = actor['distance_lead'][start:int(lane_id_crossing)]
                            velocity = actor['velocities'][start:int(lane_id_crossing)]
                            ttc = actor['ttc_leading'][start:int(lane_id_crossing)]
                            include = np.where(distance_lead >= 0)

                            # if len(np.where(distance_lead[include] - (velocity[include] * 1.8) < 0)[0]) != 0:
                            if len(np.where((ttc[include] < 2) & (ttc[include] > 0.01))[0]) != 0:
                                actor['cut_out_left'][start:(end + 1)] = 1
                                actor['cut_outs_left'] += 1

                        else:
                            actor['lane_change_right'][start:(end + 1)] = 1
                            actor['lane_changes_right'] += 1

                            distance_following = actor['distance_following'][int(lane_id_crossing):(end + 1)]
                            velocity_following = actor['velocity_following'][int(lane_id_crossing):(end + 1)]
                            ttc = actor['ttc_following'][int(lane_id_crossing):(end + 1)]
                            include = np.where(distance_following >= 0)

                            # if len(np.where(distance_following[include] - (velocity_following[include] * 1.8) < 0)[0]) != 0:
                            if len(np.where((ttc[include] < 2) & (ttc[include] > 0.01))[0]) != 0:
                                actor['cut_in_right'][start:(end + 1)] = 1
                                actor['cut_ins_right'] += 1

                            distance_lead = actor['distance_lead'][start:int(lane_id_crossing)]
                            velocity = actor['velocities'][start:int(lane_id_crossing)]
                            ttc = actor['ttc_leading'][start:int(lane_id_crossing)]
                            include = np.where(distance_lead >= 0)

                            # if len(np.where(distance_lead[include] - (velocity[include] * 1.8) < 0)[0]) != 0:
                            if len(np.where((ttc[include] < 2) & (ttc[include] > 0.01))[0]) != 0:
                                actor['cut_out_right'][start:(end + 1)] = 1
                                actor['cut_outs_right'] += 1


def lane_change_detector(scenario):
    for t in scenario['actors']:
        detect_cut_in_and_lane_change_for_actor(actor=t)

    return scenario

