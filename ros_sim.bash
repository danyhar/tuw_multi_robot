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

MAX_RUNS=50
SLEEP_TIME=60
ROBOTS=8
WORLD="warehouse008"

source /home/robot/projects/mrrp/ws02/devel/setup.bash

# Start Framework
roslaunch tuw_multi_robot_demo demo.launch room:=$WORLD  nr_of_robots:=$ROBOTS &
PID_FRAMEWORK=$!

# Sleep for some time to start framework properly
sleep 30

# for i in $(seq 0 $ROBOTS);
# do
#     rostopic pub -1  /robot_$i/ctrl std_msgs/String "stop"
# done 


for i in $(seq 1 $MAX_RUNS);
do
    echo -e $PINK Run $i of $MAX_RUNS $NORMAL

    # # Start Framework
    # roslaunch tuw_multi_robot_demo demo.launch room:=$WORLD  nr_of_robots:=$ROBOTS &
    # PID_FRAMEWORK=$!

    # # Sleep for some time to start framework properly
    # sleep 5

    # for j in $(seq 0 $ROBOTS);
    # do
    #     rostopic pub -1  /robot_$j/ctrl std_msgs/String "stop"
    # done 

    echo -e $BLUE
    # Start rostopic to ensure planner has finished
    rostopic echo -n 1 -p /planner_status | tee -a ~/customlaunch/result/r8/PR_SR_CRout$i.txt &
    PID_ROSTOPIC=$!
    sleep 5

    # Start a goal saver
    echo -e $CYAN "Starting goal saver"
    rosrun tuw_multi_robot_goal_generator goals_saver _file_name:=/home/robot/customlaunch/result/r8/PR_SR_CRgoals$i.txt &
    PID_GOAL_SAVER=$!
    sleep 5
    
    echo -e $NORMAL

    # Start a goal generator
    rosrun tuw_multi_robot_goal_generator goals_random _nr_of_robots:=$ROBOTS _distance_boundary:=0.6 _distance_to_map_border:=0.2 _nr_of_avaliable_robots:=$ROBOTS &
    #wait $!
    PID_GOAL_GENERATOR=$!

    sleep $SLEEP_TIME
    kill $PID_ROSTOPIC $PID_GOAL_SAVER $PID_GOAL_GENERATOR
    #kill $PID_FRAMEWORK $PID_ROSTOPIC $PID_GOAL_SAVER
    # rosservice call /reset_routes
    # sleep 5
    # rosservice call /reset_positions
    sleep 10
    # wait $PID_GOAL_SAVER $PID_GOAL_GENERATOR $PID_ROSTOPIC
    # wait $PID_GOAL_SAVER
    
done;

# sleep 25
kill $PID_FRAMEWORK
echo -e $GREEN Everything finished $NORMAL