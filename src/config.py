# lane ID configs
lane_id_y_values = [2.1, 5.9, 9.4, 13.15, 16.9, 20.65, 23.9]
lane_id_x_values = [-10, 105]
distance_center_lane = [4, 7.65, 11.275, 15.025, 18.775, 22.275]

# speeding and standing configs
speedlimit = 130 / 3.6
standingThreshold = 0.04

# breakdown configs
breakdown_time_threshold = 30

# traffic jam configs
velocity_traffic_jam_threshold = 20 / 3.6
velocity_slow_moving_traffic_threshold = 40 / 3.6

# accident configs
ttc_threshold = 0.1
velocity_accident_threshold = 15 / 3.6
distance_FP_threshold = 0.1
distance_divisor = 30
model_path = "src/maneuver_detection/accident_detection_model.pt"
image_resolution = 640
confidence_threshold = 0.8
