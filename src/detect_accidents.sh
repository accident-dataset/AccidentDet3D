#!/usr/bin/env bash

# Start the globus connection
cd /home/user/globusconnectpersonal-3.2.3/ || exit
./globusconnect -start -restrict-paths /media/hdd/rosbags/storage/ &

# Change into the correct directory to start the extraction later on
cd /home/user/ros_ws || exit
source devel/setup.bash

# Establish globus link
globus login --no-local-server

# Poll connection status
echo "Current globus session...";
globus session show

# Transfer files from upload folder in loop, analyze then and afterwards delete them
echo "Initiating transfer...";
src_ep="3fa12976-ca0e-11eb-bde6-5111456017d9:/"
dst_ep="a8250990-a0e4-11ee-8801-a52c65340a88:/media/hdd/rosbags/storage/"
download_start=""

echo "Start globus ls"
file_list="$(globus ls --recursive-depth-limit 4 -r ${src_ep})"
echo "Finished globus ls"

for filename in ${file_list}; do
  # check if filename is a path to a rosbag
  IFS='.' read -ra splits_path <<< "$filename"
  if [ "${splits_path[-1]}" != "bag" ]; then
    echo "Skipping ${filename}, since it is not a rosbag!"
    continue
  fi

  # check if the rosbag has already previously been analyzed and should therefore get skipped
  if [[ "$filename" < "$download_start" ]]; then
    echo "${filename} is smaller than ${download_start}. Skipping rosbag"
    continue
  fi

  # check if it is too dark in the rosbag according to the time
  sun=true
  IFS='/' read -ra splits_path <<< "$filename"
  case "${splits_path[2]}" in
    "month_01")
      if [[ "${splits_path[4]}" < "hour_07"  ||  "${splits_path[4]}" > "hour_17" ]]; then
        sun=false
      fi
      ;;
    "month_02")
      if [[ "${splits_path[4]}" < "hour_06"  ||  "${splits_path[4]}" > "hour_18" ]]; then
        sun=false
      fi
      ;;
    "month_03" | "month_04")
      if [[ "${splits_path[4]}" < "hour_05"  ||  "${splits_path[4]}" > "hour_20" ]]; then
        sun=false
      fi
      ;;
    "month_05" | "month_06" | "month_07" | "month_08")
      if [[ "${splits_path[4]}" < "hour_05"  ||  "${splits_path[4]}" > "hour_21" ]]; then
        sun=false
      fi
      ;;
    "month_09")
      if [[ "${splits_path[4]}" < "hour_06"  ||  "${splits_path[4]}" > "hour_20" ]]; then
        sun=false
      fi
      ;;
    "month_10")
      if [[ "${splits_path[4]}" < "hour_06"  ||  "${splits_path[4]}" > "hour_19" ]]; then
        sun=false
      fi
      ;;
    "month_11")
      if [[ "${splits_path[4]}" < "hour_06"  ||  "${splits_path[4]}" > "hour_17" ]]; then
        sun=false
      fi
      ;;
    "month_12")
      if [[ "${splits_path[4]}" < "hour_07"  ||  "${splits_path[4]}" > "hour_16" ]]; then
        sun=false
      fi
      ;;
  esac

  if [ "$sun" = false ]; then
    echo "The sun is not yet out. Skipping rosbag!"
    continue
  fi

  # Download the rosbag
  echo "Transferring ${filename} to ${dst_ep}"
  task_id="$(globus transfer ${src_ep}${filename} ${dst_ep}${filename} --jmespath 'task_id' --format=UNIX --encrypt --verbose --sync-level checksum --notify off)"
  globus task wait ${task_id} --polling-interval 5 --heartbeat --verbose

  if [ $? -eq 0 ]; then
    echo "Completed download successfully";

    # Extract standing events in the downloaded rosbags
    echo "Execute first_accident_classification_rosbags.py";
    python src/scenarios/src/standing_detection_rosbag.py -e
    if [ $? -eq 0 ]; then
      # Extract accidents in the found standing rosbags
      echo "Standing vehicle found! Execute scenario_detection_rosbag.py to look for accidents";
      python src/scenarios/src/scenario_detection_rosbag.py -s -img accident_breakdown
    else
      echo "No standing vehicle found in the rosbag!"
    fi
  else
    echo "Failed download!";
  fi

  # Remove the rosbag after the extraction finished
  echo "Remove the rosbag!"
  IFS=':' read -ra splits_path <<< "$dst_ep"
  rm ${splits_path[1]}${filename}
done

echo "Exiting..."
exec "$@"
