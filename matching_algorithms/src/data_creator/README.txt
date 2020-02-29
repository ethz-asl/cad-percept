#Data Creator

Read Data from sequence and save them as single frames in bag-files (we are only interested in initial position, no movement). Instruction:
rosrun matching_algorithms dataReader

Should then be easy to read ground thruth position from a text file to get test cases.

Additionally, one can find a launch file to visualize points. Instructions:
First terminal:
roslaunch visualize_data.launch
Second terminal:
rosparam set use_sim_time true
rosbag play -l scanxxx.bag

