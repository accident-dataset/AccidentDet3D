#!/usr/bin/env python3
from __future__ import absolute_import
from std_msgs.msg import Header

import rospy
import time
import sys
import numpy as np

from system_messages.msg import BackendOutputExtended
from system_messages.msg import DetectedObjectExtended
from system_messages.msg import BackendOutput
from system_messages.msg import DTwinOutput
from feature_extractor_dtwin_live import FeatureExtractorDTwinLive
from read_write_rosbag import get_quaternion_from_euler
import argparse

class ScenarioDigitalTwin:
    """
    This module converts scenarios to ROS messages. It uses the BackendOutput messages as baseline and extends it with
    the extracted scenarios. The new ros messages are being published together with the digital twin.
    """

    FRAME_CACHE_LIMIT = 200
    VEHICLE_TRACKING_FRAME_THRESHOLD = 50
    SPEED_LIMIT = 130

    def __init__(self, input_topic_dtwin, output_topic):
        self.input_topic_dtwin = input_topic_dtwin
        self.output_topic = output_topic

        self.starttime = time.time()
        self.pub = None

        self.feature_extractor = FeatureExtractorDTwinLive(
            cache_limit=self.FRAME_CACHE_LIMIT,
            frame_threshold=self.VEHICLE_TRACKING_FRAME_THRESHOLD,
            speed_limit=self.SPEED_LIMIT,
        )

    def convert_scenarios_to_msg(self, dtwin_msg):
        backend_msg = BackendOutputExtended()
        backend_msg.header = Header()
        backend_msg.header.stamp = rospy.Time()
        backend_msg.header.stamp.secs = dtwin_msg.header.stamp.secs
        backend_msg.header.stamp.nsecs = dtwin_msg.header.stamp.nsecs
        backend_msg.header.seq = dtwin_msg.header.seq
        backend_msg.num_detected = dtwin_msg.num_detected
        object_list = []

        num_speeding = 0
        num_standing = 0
        num_lane_changes = 0
        num_cut_in_right = 0
        num_cut_in_left = 0
        num_cut_out_right = 0
        num_cut_out_left = 0
        num_minor_tailgates = 0
        num_moderate_tailgates = 0
        num_severe_tailgates = 0

        for obj in dtwin_msg.object_list:
            object_data_msg = DetectedObjectExtended()
            object_data_msg.object_ID = obj.object_ID
            object_data_msg.position = obj.position
            object_data_msg.dimensions = obj.dimensions
            object_data_msg.speed = obj.speed
            object_data_msg.object_class = obj.object_class
            obj_scenarios = self.feature_extractor.get_actor_by_object_id(object_id=obj.object_ID)

            if not obj_scenarios:
                continue

            yaw = obj_scenarios["path"][-1, 3]
            pitch = 0.0
            roll = 0.0
            quaternion = get_quaternion_from_euler(np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw))
            object_data_msg.heading = quaternion

            object_data_msg.speeding = bool(obj_scenarios['speeding'][-1])
            object_data_msg.standing = bool(obj_scenarios['standing'][-1])
            object_data_msg.lane_id = int(obj_scenarios['lane_id'][-1])
            object_data_msg.lane_change_left = any(obj_scenarios['lane_change_left'])
            object_data_msg.lane_change_right = any(obj_scenarios['lane_change_right'])
            object_data_msg.cut_in_left = any(obj_scenarios['cut_in_left'])
            object_data_msg.cut_in_right = any(obj_scenarios['cut_in_right'])
            object_data_msg.cut_out_left = any(obj_scenarios['cut_out_left'])
            object_data_msg.cut_out_right = any(obj_scenarios['cut_out_right'])

            if any(obj_scenarios['lane_change_left']) or any(obj_scenarios['lane_change_right']):
                num_lane_changes += 1

            if bool(obj_scenarios['speeding'][-1]):
                num_speeding += 1

            if bool(obj_scenarios['standing'][-1]):
                num_standing += 1

            num_cut_in_right += obj_scenarios['cut_ins_right']
            num_cut_in_left += obj_scenarios['cut_ins_left']
            num_cut_out_right += obj_scenarios['cut_outs_right']
            num_cut_out_left += obj_scenarios['cut_outs_left']

            if obj_scenarios['tail_gate_minor'] > 0:
                object_data_msg.tail_gate = 1
                num_minor_tailgates += 1
            elif obj_scenarios['tail_gate_moderate'] > 0:
                object_data_msg.tail_gate = 2
                num_moderate_tailgates += 1
            elif obj_scenarios['tail_gate_severe'] > 0:
                object_data_msg.tail_gate = 4
                num_severe_tailgates += 1
            else:
                object_data_msg.tail_gate = 0
            object_list.append(object_data_msg)

        backend_msg.object_list = object_list
        backend_msg.num_lane_changes = num_lane_changes
        backend_msg.num_cut_in_right = num_cut_in_right
        backend_msg.num_cut_in_left = num_cut_in_left
        backend_msg.num_cut_out_right = num_cut_out_right
        backend_msg.num_cut_out_left = num_cut_out_left
        backend_msg.num_speeding = num_speeding
        backend_msg.num_standing = num_standing
        backend_msg.num_tailgates = num_minor_tailgates + num_moderate_tailgates + num_severe_tailgates
        backend_msg.num_minor_tailgates = num_minor_tailgates
        backend_msg.num_moderate_tailgates = num_moderate_tailgates
        backend_msg.num_severe_tailgates = num_severe_tailgates

        return backend_msg

    def dtwin_callback(self, dtwin_msg):
        start_time = time.time()
        self.feature_extractor.extract_scenarios(msg=dtwin_msg)
        backend_output_msg = self.convert_scenarios_to_msg(dtwin_msg)
        print("scenario detection time: ", str(time.time() - start_time))
        self.pub.publish(backend_output_msg)

    def create_publisher(self):
        self.pub = rospy.Publisher(self.output_topic, BackendOutputExtended, queue_size=1)

    def create_listener(self):
        # when accessing remote dtwin (-> use DTwinOutput)
        # when accessing live dtwin or rosbag dtwin (-> use BackendOutput)
        if input_topic_dtwin == "/global/fusion/estimates":
            rospy.Subscriber(self.input_topic_dtwin, DTwinOutput, self.dtwin_callback)
        else:
            rospy.Subscriber(self.input_topic_dtwin, BackendOutput, self.dtwin_callback)

        rospy.spin()

    @staticmethod
    def get_param(name):
        full_param_name = rospy.search_param(name)
        param = rospy.get_param(full_param_name)
        return param


if __name__ == '__main__':
    rospy.init_node('scenarios_dtwin_live', anonymous=True)

    try:
        input_topic_dtwin = rospy.get_param("~input_topic_dtwin")
    except KeyError:
        argparser = argparse.ArgumentParser(
            description=__doc__)
        argparser.add_argument(
            '--input_topic_dtwin',
            default='/s40/s50/tracker/estimates/throttled',
            help='Input digital twin ROS topic (default: /s40/s50/tracker/estimates/throttled)')
        args = argparser.parse_args()
        input_topic_dtwin = args.input_topic_dtwin

    output_topic = input_topic_dtwin + '/extended'

    scenario_digital_twin = ScenarioDigitalTwin(input_topic_dtwin=input_topic_dtwin, output_topic=output_topic)
    scenario_digital_twin.create_publisher()
    scenario_digital_twin.create_listener()
