#!/bin/bash

# stupid simple script to open each ros terminal in tabs for ardrone project
# to use:
# chmod +x <this script>
# ./<this script>

#use WEBCAM
#gnome-terminal \
#--tab -t "roscore" -e "bash -c 'roscore'" \
#--tab -t "gscam" -e "bash -c 'sleep 1; rosrun gscam gscam'" \
#--tab -t "ar_recog" -e "bash -c 'sleep 5; cd ~/ros/brown-ros-pkg/experimental/ar_recog/bin; pwd; rosrun ar_recog ar_recog image:=/gscam/image_raw camera_info:=/gscam/camera_info'" \
#--tab -t "image_view" -e "bash -c 'sleep 1; rosrun image_view image_view image:=/ar/image'" \
#--tab -t "cmd_vel" -e "bash -c 'sleep 1; rostopic echo cmd_vel'" \
#--tab -t "tags" -e "bash -c 'sleep 1; rostopic echo tags'" \
#--tab -t "drone_driver" -e "bash -c 'sleep 5; rosrun drone_driver drone_driver.py; read'"
 
#use ardrone
gnome-terminal \
--tab -t "roscore" -e "bash -c 'roscore'" \
--tab -t "ardrone_driver" -e "bash -c 'sleep 1; rosrun ardrone_brown ardrone_driver'" \
--tab -t "ar_recog" -e "bash -c 'sleep 5; cd ~/ros/brown-ros-pkg/experimental/ar_recog/bin; pwd; rosrun ar_recog ar_recog image:=/ardrone/image_raw camera_info:=/ardrone/camera_info'" \
--tab -t "image_view" -e "bash -c 'sleep 1; rosrun image_view image_view image:=/ar/image'" \
--tab -t "cmd_vel" -e "bash -c 'sleep 1; rostopic echo cmd_vel'" \
--tab -t "tags" -e "bash -c 'sleep 1; rostopic echo tags'" \
--tab -t "drone_driver" -e "bash -c 'sleep 5; rosrun drone_driver drone_driver.py; read'"


