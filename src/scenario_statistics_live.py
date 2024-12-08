#!/usr/bin/env python3
from __future__ import absolute_import
import rospy
from std_msgs.msg import String
from system_messages.msg import BackendOutput
from system_messages.msg import DTwinOutput
from system_messages.msg import ScenarioStatistics
import requests
import numpy as np
from detection_live import extractor
import time
import sys

# Description:
# This module creates a live scenario statistics, publishes the statistics and sends it to the web server for visualization

starttime = time.time()
pub = None


def do_post(data):
    global pub
    global starttime

    speeding = 0
    standing = 0
    tailgate_1 = 0
    tailgate_2 = 0
    tailgate_3 = 0

    object_list = extractor(data.object_list)

    for key in object_list.keys():
        object = object_list[key]
        speeding += object["speeding"]
        standing += object["standing"]
        tailgate_1 += object["tailgate1"]
        tailgate_2 += object["tailgate2"]
        tailgate_3 += object["tailgate3"]

    data = {"fields": ["header", "speeding", "standing", "tailgate1", "tailgate2", "tailgate3"],
            "data": [{"header": "Statistics", "speeding": "Speeding", "standing": "Standing",
                      "tailgate1": "Minor Tailgate", "tailgate2": "Moderate Tailgate", "tailgate3": "Extreme Tailgate"},
                     {"header": "Number of Vehicles", "speeding": speeding, "standing": standing,
                      "tailgate1": tailgate_1, "tailgate2": tailgate_2, "tailgate3": tailgate_3}]}

    url = 'http://127.0.0.1:42000/pub?id=scenario_statistics'
    # x = requests.post(url, json=data)

    # Check for correctness
    stat = ScenarioStatistics()
    stat.speeding = speeding
    stat.standing = standing
    stat.tailgate1 = tailgate_1
    stat.tailgate2 = tailgate_2
    stat.tailgate3 = tailgate_3

    # publish statistics data
    pub.publish(stat)


def create_publisher(output_topic):
    global pub
    pub = rospy.Publisher(output_topic, ScenarioStatistics, queue_size=1)


def create_listener(input_topic):
    # when accessing remote dtwin (-> use DTwinOutput)
    # when accessing live dtwin or rosbag dtwin (-> use BackendOutput)
    if input_topic == "/global/fusion/estimates":
        rospy.Subscriber(input_topic, DTwinOutput, do_post)
    else:
        rospy.Subscriber(input_topic, BackendOutput, do_post)

    rospy.spin()


def get_param(name):
    full_param_name = rospy.search_param(name)
    param = rospy.get_param(full_param_name)
    return param


if __name__ == '__main__':
    print("start listener...")
    rospy.init_node('scenarios', anonymous=True)

    try:
        input_topic = rospy.get_param("~input_topic")
    except KeyError:
        input_topic = sys.argv[1]

    output_topic = input_topic + '/scenario_statistics'

    create_publisher(output_topic)
    create_listener(input_topic)
