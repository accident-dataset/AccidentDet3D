import numpy as np
from shapely.geometry import LineString
from base_trajectories.base_trajectories_highway import BaseTrajectoriesHighway


class ExtractDistanceLeadFollowVehicle:
    # This module extracts the distance to the lead/following vehicle (in meters) based on the Lane ID.
    # The OpenDRIVE standard was used to assign lane IDs to vehicles using positive lane ids for the south direction
    # and negative lane ids for the north direction.

    @staticmethod
    def initialize_distance_features_for_actor(actor):
        actor['distance_lead'] = np.full(actor['path'][:, 0].shape, -1.0)
        actor['distance_following'] = np.full(actor['path'][:, 0].shape, -1.0)
        actor['velocity_lead'] = np.full(actor['path'][:, 0].shape, -1.0)
        actor['velocity_following'] = np.full(actor['path'][:, 0].shape, -1.0)
        actor['ttc_following'] = np.full(actor['path'][:, 0].shape, -1.0)
        actor['ttc_leading'] = np.full(actor['path'][:, 0].shape, -1.0)

    @staticmethod
    def get_scenario_length(scenario):
        return scenario['meta']['num_frames']

    @staticmethod
    def calculate_edm(p, q):
        """Euclidean square distance matrix.

        Inputs:
        x: (N,) numpy array
        y: (N,) numpy array

        Ouput:
        (N, N) Euclidean square distance matrix:
        r_ij = p_ij^2 - q_ij^2
        """

        p2 = np.einsum('ij,ij->i', p, p)[:, np.newaxis]
        q2 = np.einsum('ij,ij->i', q, q)[:, np.newaxis].T

        pq = np.dot(p, q.T)

        return np.abs(p2 + q2 - 2. * pq)

    @staticmethod
    def assign_distance_velocity(scenario, msg_idx, lead, follow, distance):
        offset_vehicle_lead = int(
            np.round(scenario['actors'][lead['id']]['offset'] * scenario['actors'][lead['id']]['frame_rate'])
        )
        offset_vehicle_follow = int(
            np.round(scenario['actors'][follow['id']]['offset'] * scenario['actors'][follow['id']]['frame_rate'])
        )

        scenario['actors'][lead['id']]['distance_following'][msg_idx - offset_vehicle_lead] = \
            abs(distance)
        scenario['actors'][follow['id']]['distance_lead'][msg_idx - offset_vehicle_follow] = \
            abs(distance)

        scenario['actors'][lead['id']]['velocity_following'][msg_idx - offset_vehicle_lead] = \
            follow['velocity']
        scenario['actors'][follow['id']]['velocity_lead'][msg_idx - offset_vehicle_follow] = \
            lead['velocity']

        return scenario

    @staticmethod
    def any_following_vehicle(vehicle):
        return len(vehicle['distance_following'][(vehicle['distance_following'] > 0)]) != 0

    @staticmethod
    def any_lead_vehicle(vehicle):
        return len(vehicle['distance_lead'][(vehicle['distance_lead'] > 0)]) != 0

    @staticmethod
    def extract_ttc_for_actor(actor):
        if ExtractDistanceLeadFollowVehicle.any_following_vehicle(actor):
            actor['ttc_following'][(actor['distance_following'] > 0)] = actor['distance_following'][(
                    actor['distance_following'] > 0)] / (
                                                                                (actor['velocity_following'][(actor[
                                                                                                                  'distance_following'] > 0)] + 0.000001) -
                                                                                actor['velocities'][
                                                                                    (actor['distance_following'] > 0)]
                                                                        )
            actor['ttc_following'][actor['ttc_following'] < 0] = -1

        if ExtractDistanceLeadFollowVehicle.any_lead_vehicle(actor):
            actor['ttc_leading'][(actor['distance_lead'] > 0)] = actor['distance_lead'][(actor['distance_lead'] > 0)] / \
                                                                 ((actor['velocities'][(actor['distance_lead'] > 0)]
                                                                   + 0.000001) -
                                                                  actor['velocity_lead'][(actor['distance_lead'] > 0)]
                                                                  )
            actor['ttc_leading'][actor['ttc_leading'] < 0] = -1

    @staticmethod
    def extract_lead_follow_distances_for_msg(scenario, msg_idx):
        # positive lane IDs: towards southern direction
        # negative lane IDs: towards northern direction
        lane = {1: [], 2: [], 3: [], 4: [], 5: [], 6: [], -1: [], -2: [], -3: [], -4: [], -5: [], -6: []}
        for actor_idx in range(len(scenario['actors'])):
            offset = int(
                np.round(scenario['actors'][actor_idx]['offset'] * scenario['actors'][actor_idx]['frame_rate']))
            if (0 <= msg_idx - offset < len(scenario['actors'][actor_idx]['path'])) \
                    and (scenario['actors'][actor_idx]['lane_id'][msg_idx - offset] != 0):
                lane[scenario['actors'][actor_idx]['lane_id'][msg_idx - offset]].append({
                    'point': scenario['actors'][actor_idx]['path'][msg_idx - offset][:2],
                    'velocity': scenario['actors'][actor_idx]['velocities'][msg_idx - offset],
                    'id': actor_idx
                })

        for lane_id, data in lane.items():
            # sort all vehicles in one line by the x value and only compare the distance between
            # the vehicle in front and in the back
            data.sort(key=lambda elem: elem['point'][0])

            for i in range(len(data) - 1):
                vehicle_1 = data[i]
                vehicle_2 = data[i+1]
                distance = ExtractDistanceLeadFollowVehicle.calculate_edm(
                    np.reshape(vehicle_1['point'], (1, 2)),
                    np.reshape(vehicle_2['point'], (1, 2))
                )[0]

                # Vehicle 1 is lead, vehicle 2 is following
                if lane_id < 0:
                    scenario = ExtractDistanceLeadFollowVehicle.assign_distance_velocity(
                        scenario, msg_idx, lead=vehicle_1, follow=vehicle_2, distance=distance
                    )
                # Vehicle 1 is following, vehicle 2 is lead
                else:
                    scenario = ExtractDistanceLeadFollowVehicle.assign_distance_velocity(
                        scenario, msg_idx, lead=vehicle_2, follow=vehicle_1, distance=distance
                    )

    @staticmethod
    def distance_lead_follow_vehicle(scenario):
        for actor in scenario['actors']:
            ExtractDistanceLeadFollowVehicle.initialize_distance_features_for_actor(actor)

        msg_counter = 0
        for i in range(ExtractDistanceLeadFollowVehicle.get_scenario_length(scenario=scenario)):
            if msg_counter % 1000 == 0:
                print("msg idx:", str(i))
            msg_counter += 1
            ExtractDistanceLeadFollowVehicle.extract_lead_follow_distances_for_msg(scenario=scenario, msg_idx=i)

        for actor in scenario['actors']:
            ExtractDistanceLeadFollowVehicle.extract_ttc_for_actor(actor)

        return scenario
