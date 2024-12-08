import numpy as np


# Description
# This module provides the implementation for tail gate maneuver detection.


def detect_tail_gate_for_actor(actor):
    tail_gate = np.full(actor['path'][:, 0].shape, 0)

    # Min Distance Highway / Rural
    required_distance = actor['velocities'] * 3.6 / 2
    # Min Distance Urban
    # required_distance = t['velocities']
    tail_gate[np.where(actor['distance_lead'] - required_distance < 0)[0]] = 1
    tail_gate[np.where((actor['distance_lead'] - (required_distance * 0.5) < 0) & ((actor['velocities'] * 3.6) > 80) &
                       ((actor['velocities'] * 3.6) < 100))[0]] = 2
    tail_gate[np.where((actor['distance_lead'] - (required_distance * 0.5) < 0) & ((actor['velocities'] * 3.6) > 100) &
                       (actor['distance_lead'] - (required_distance * 0.3) >= 0))[0]] = 2
    tail_gate[
        np.where((actor['distance_lead'] - (required_distance * 0.3) < 0) & ((actor['velocities'] * 3.6) > 100))[0]] = 3
    tail_gate[np.where(actor['distance_lead'] == -1)[0]] = 0

    tailgate_severity = []
    length = 0
    for array in np.split(tail_gate, np.nonzero(((tail_gate > 0)[1:] != (tail_gate > 0)[:-1]))[0] + 1):
        length += len(array)
        tailgate_severity.append(np.max(array))
        if len(array) < actor['frame_rate'] * 1:
            tail_gate[length - len(array): length] = 0

    actor['tail_gate'] = tail_gate
    actor['tail_gate_minor'] = tailgate_severity.count(1)
    actor['tail_gate_moderate'] = tailgate_severity.count(2)
    actor['tail_gate_severe'] = tailgate_severity.count(3)


def tail_gate_detector(scenario):
    for t in scenario['actors']:
        detect_tail_gate_for_actor(actor=t)

    return scenario
