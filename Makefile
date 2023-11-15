all: help

help:
	@echo "make create 			- creates both container of the images"
	@echo "make start  			- starts tuw-bridge and rosmaster container"
	@echo "make attach-master		- attaches a consol session to rosmaster container"
	@echo "make stop   			- stops tuw-bridge and rosmaster container"
	@echo "make remove 			- removes tuw-bridge and rosmaster container"

create-bridge:
	@docker run -it -d --name tuw-bridge --net=host registry.auto.tuwien.ac.at/roblab/docker/focal/galactic-tuw-bridge /bin/bash -c "ros2 run ros1_bridge dynamic_bridge --bridge-all-topics"

create-eval-bridge:
	@docker run -it -d --name tuw-eval-bridge --net=host registry.auto.tuwien.ac.at/roblab/docker/focal/eval-tuw-bridge /bin/bash -c "ros2 run ros1_bridge dynamic_bridge --bridge-all-topics"

create-rosmaster:
	@docker run --privileged -ti --network="host" --name rosmaster --env="DISPLAY" -v /home/daniel/rosprojects/customlaunch:/home/robot/customlaunch -v /home/daniel/rosprojects/ws02:/home/robot/projects/mrrp/ws02  registry.auto.tuwien.ac.at/roblab/docker/focal/noetic-mrrp-desktop

create: create-bridge create-rosmaster

create-eval: create-eval-bridge create-rosmaster

start: start-eval-bridge start-rosmaster

start-rosmaster:
	@docker start rosmaster

start-bridge:
	@docker start tuw-bridge

start-eval-bridge:
	@docker start tuw-eval-bridge

attach-master:
	@echo "You can run on of the following commands in the container:"
	@echo "	roslaunch tuw_multi_robot_demo demo.launch room:=cave  nr_of_robots:=2"
	@echo "	roslaunch tuw_multi_robot_demo demo.launch room:=warehouse032  nr_of_robots:=14"
	@echo "	rosrun tuw_multi_robot_goal_generator goals_random _nr_of_robots:=32 _distance_boundary:=0.6 _distance_to_map_border:=0.2 _nr_of_avaliable_robots:=14"
	@docker exec -it rosmaster /bin/bash 

build-bridge:
	@docker build -t registry.auto.tuwien.ac.at/roblab/docker/focal/eval-tuw-bridge .

stop:
	@docker stop tuw-bridge rosmaster

remove: stop
	@docker rm tuw-bridge rosmaster
