# !/bin/bash

BLACK="\033[30m"
RED="\033[31m"
GREEN="\033[32m"
YELLOW="\033[33m"
BLUE="\033[34m"
PINK="\033[35m"
CYAN="\033[36m"
WHITE="\033[37m"
NORMAL="\033[0;39m"

MAX_RUNS=3

ROBOTS=4
WORLD="warehouse008"

# Start Framework
roslaunch tuw_multi_robot_demo demo.launch room:=$WORLD  nr_of_robots:=$ROBOTS &
PID_FRAMEWORK=$!

# Sleep for some time to start framework properly
sleep 25

for i in $(seq 1 $MAX_RUNS);
do
    echo -e $PINK Run $i of $MAX_RUNS
    
    # Start rostopic to ensure planner has finished
    rostopic echo -n 1 /planner_status &
    PID_ROSTOPIC=$!
    sleep 5
    # Start a goal saver
    echo -e $NORMAL "Starting goal saver"
    rosrun tuw_multi_robot_goal_generator goals_saver _file_name:=/home/robot/goals$i.txt &
    PID_GOAL_SAVER=$!

    sleep 15

    # Start a goal generator
    rosrun tuw_multi_robot_goal_generator goals_random _nr_of_robots:=$ROBOTS _distance_boundary:=0.6 _distance_to_map_border:=0.2 _nr_of_avaliable_robots:=$ROBOTS &
    PID_GOAL_GENERATOR=$!

    sleep 15

    wait $PID_GOAL_SAVER $PID_GOAL_GENERATOR $PID_ROSTOPIC
    # wait $PID_GOAL_SAVER
done

sleep 25
kill $PID_FRAMEWORK