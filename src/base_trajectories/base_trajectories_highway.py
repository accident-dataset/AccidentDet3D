from __future__ import absolute_import

import numpy as np


class BaseTrajectoriesHighway:
    # This class defines the key points (polygons) for each possible maneuver on the highway.
    # These polygons are used to detect maneuvers of vehicles.


    @staticmethod
    def traj_lane_change():
        return np.array([[0, 11.6], [55, 11.4], [93, 11.7], [130, -16.6], [140, -16.5], [175, -16.5],
                         [215, -16.4], [275, -16.5], [340, -16.6], [400, -16.4], [450, -16.6]])

    @staticmethod
    def traj_lane_change3():
        return np.array([[0, -19.6], [55, -19.4], [93, -17.7], [130, -16.6], [140, -16.5], [175, -16.5],
                         [215, -16.4], [275, -16.5], [340, -16.6], [400, -16.4], [450, -16.6]])

    @staticmethod
    def traj_straight6():
        # TODO: adjust polygones for shoulder lane
        trajectory = BaseTrajectoriesHighway.traj_straight5()
        trajectory[:, 1] -= 3.5
        return trajectory

    @staticmethod
    def traj_straight5():
        return np.array([[0, -19.9], [55, -19.7], [130, -19.9], [195, -19.8], [275, -19.8], [350, -19.9],
                         [450, -19.9]])

    @staticmethod
    def traj_straight4():
        trajectory = BaseTrajectoriesHighway.traj_straight5()
        trajectory[:, 1] += 3.8
        return trajectory

    @staticmethod
    def traj_straight3():
        trajectory = BaseTrajectoriesHighway.traj_straight5()
        trajectory[:, 1] += 7.6
        return trajectory

    @staticmethod
    def traj_straight2():
        trajectory = BaseTrajectoriesHighway.traj_straight5()
        trajectory[:, 1] += 11.4
        return trajectory

    @staticmethod
    def traj_straight1():
        trajectory = BaseTrajectoriesHighway.traj_straight5()
        trajectory[:, 1] += 15.2
        return trajectory

    @staticmethod
    def traj_straight_neg1():
        return np.array([[450, 3.6], [350, 3.4], [275, 3.6], [195, 3.5], [130, 3.5], [55, 3.6],
                         [0, 3.6]])

    @staticmethod
    def traj_straight_neg2():
        trajectory = BaseTrajectoriesHighway.traj_straight_neg1()
        trajectory[:, 1] += 3.8
        return trajectory

    @staticmethod
    def traj_straight_neg3():
        trajectory = BaseTrajectoriesHighway.traj_straight_neg1()
        trajectory[:, 1] += 7.6
        return trajectory

    @staticmethod
    def traj_straight_neg4():
        trajectory = BaseTrajectoriesHighway.traj_straight_neg1()
        trajectory[:, 1] += 11.4
        return trajectory

    @staticmethod
    def traj_exit():
        return np.array(
            [[450, 18.5], [400, 18.5], [350, 18.6], [300, 21.3], [275, 21.9], [140, 22], [130, 22.3], [115, 24.8],
             [90, 34.6], [75, 43.7], [60, 57.5], [38, 91]])

    @staticmethod
    def traj_exit2():
        return np.array([[450, 18.5], [400, 18.5], [350, 18.6], [255, 18.6], [175, 18.6], [140, 18.6], [125, 19],
                         [93, 28], [65, 47], [45, 70], [33, 92]])

    @staticmethod
    def traj_special():
        return np.array([[450, 4.5], [350, 7.6], [255, 12.6], [175, 18.6], [141.2, 20.6], [125, 21], [93, 28],
                         [65, 46], [45, 69], [33, 92]])

    @staticmethod
    def traj_lane_change2():
        return np.array([[-50, -19.6], [50, -19.4], [95, -17.7], [130, -16.6], [140, -16.5], [175, -16.5],
                         [215, -16.4], [280, -16.5], [350, -16.6], [400, -16.4], [450, -16.6]])

    @staticmethod
    def traj_custom1():
        return np.array([[0, -7.6], [50, -9.6], [100, -11.7], [150, -13.5],
                         [200, -15.3], [280, -15.5], [350, -15.6], [400, -15.5], [450, -15.6]])

    @staticmethod
    def traj_custom2():
        return np.array([[0, -14.4], [50, -13.7], [100, -12.8], [150, -12.1],
                         [200, -11.5], [280, -11.6], [350, -11.6], [400, -11.5], [450, -11.6]])

    @staticmethod
    def traj_custom3():
        return np.array([[0, -12], [50, -12], [100, -12], [150, -12],
                         [200, -12], [280, -12], [350, -12.6], [400, -13.2], [450, -13.8]])

    @staticmethod
    def traj_custom4():
        return np.array([[0, -10.7], [50, -11.9], [100, -13.1], [150, -14.2],
                         [200, -14.9], [280, -15.3], [350, -15.4], [400, -15.4], [450, -15.6]])

    @staticmethod
    def traj_custom5():
        return np.array([[0, -11.4], [50, -11.9], [100, -12.6], [150, -13.2],
                         [200, -13.9], [280, -14.3], [350, -14.5], [400, -14.7], [450, -14.8]])

    @staticmethod
    def traj_custom6():
        return np.array([[0, -11.9], [50, -12.1], [100, -12.8], [150, -13.9],
                         [200, -14.9], [280, -15.1], [350, -15.4], [400, -15.4], [450, -15.6]])

    @staticmethod
    def traj_custom7():
        return np.array([[450, 7.4], [350, 8.6], [250, 9.9], [150, 11.1], [100, 11.3], [55, 11.4],
                         [0, 11.5]])

    @staticmethod
    def traj_custom8():
        return np.array([[450, 15.4], [350, 13.1], [280, 11.3], [250, 9.6], [200, 7.5], [155, 7.5], [100, 9.5],
                         [50, 11], [0, 13.4]])

    @staticmethod
    def traj_custom9():
        return np.array([[450, 7.4], [350, 7.6], [200, 7.6], [150, 6.4], [100, 5.1], [55, 4.2], [0, 4]])

    @staticmethod
    def traj_custom10():
        return np.array([[450, 7.4], [350, 7.6], [300, 7.6], [250, 6.4], [200, 5.1], [155, 4.2], [0, 4]])

    @staticmethod
    def traj_custom11():
        return np.array(
            [[450, 15.4], [350, 15.1], [280, 15.3], [250, 15.2], [170, 14.1], [140, 11.2], [100, 11.5], [50, 13],
             [0, 14.4]])

    @staticmethod
    def traj_custom12():
        return np.array([[450, 15.4], [350, 15.1], [300, 15.1], [250, 15.1], [200, 15.3], [160, 15.3], [150, 15.3],
                         [140, 15.2], [120, 20.1], [100, 19.2], [60, 12.5], [50, 14], [0, 15.4]])

    @staticmethod
    def traj_custom13():
        return np.array([[450, 15.4], [350, 15.1], [300, 15.1], [250, 15.5], [200, 15.9], [150, 16.8], [120, 18.5],
                         [100, 19.3], [70, 15.2], [60, 12.1], [40, 14.2], [20, 16.5], [10, 16], [0, 15.4]])

    @staticmethod
    def traj_custom14():
        return np.array([[0, -4.6], [50, -4.6], [100, -5.5], [150, -6.9],
                         [200, -8.1], [250, -7.8], [350, -6.1], [400, -4.5], [450, -4.6]])

    @staticmethod
    def traj_custom15():
        return np.array([[0, -4.6], [50, -9.6], [100, -15.5], [150, -6.9],
                         [200, -11.1], [250, -5.8], [350, -11.1], [400, -19.5], [450, -13.6]])

    @staticmethod
    def traj_animal1():
        return np.array([[350, -25.4], [350.5, -20.6], [350.9, -15.6], [350.7, -10.4], [350.7, -5.1], [350.6, -4],
                         [346.5, -4.4], [346.9, -8.6], [348.1, -11.6],
                         [348.7, -14], [349, -16.1], [349.3, -20], [349.6, -22], [349.7, -30], [349, -33], [344, -34],
                         [355, -32]])

    @staticmethod
    def traj_animal2():
        return np.array([[351, -29.4], [350.7, -20.6], [351.2, -15.6], [351.2, -10.4], [351.3, -4.2], [351.1, -4],
                         [346.9, -4.3], [347.3, -8.9], [348.4, -11.9], [348.9, -14], [349.1, -16.1], [349.3, -20],
                         [349.6, -22], [354, -30], [356, -33], [359, -34], [355, -32]])

    @staticmethod
    def traj_animal3():
        return np.array([[351, -29.4], [350.7, -20.6], [351.2, -15.6], [352.2, -10.4], [353.3, -5.3], [355.1, -6.9],
                         [355.9, -7.8], [356.3, -8.3], [356.1, -7.6], [352.2, -7.8], [352.4, -9.6], [353, -15],
                         [353.5, -19], [353, -21], [351.4, -26], [352, -30], [355, -32]])
