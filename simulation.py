import subprocess
import time
import sys

def launch_goal_saver(file_name: str, number: int):
    base_command = "rosrun tuw_multi_robot_goal_generator goals_saver _file_name:="

    command = base_command + file_name + str(number) + ".txt"
    process = subprocess.Popen(command, shell=True)

    return process

def launch_random_goal(nr_of_robots: int):
    command = "rosrun tuw_multi_robot_goal_generator goals_random _nr_of_robots:=" + str(nr_of_robots) + " _distance_boundary:=0.6 _distance_to_map_border:=0.2 _nr_of_avaliable_robots:=" + str(nr_of_robots)
    process = subprocess.Popen(command, shell=True, start_new_session=True)

    return process

def publish_stop_command(n: int):
    command = "rostopic pub -1  /robot_" + str(n) + "/ctrl std_msgs/String \"stop\""
    subprocess.run(command, shell=True)

def launch_mrrp(nr_of_robots: int, world: str):
    base_command = "roslaunch tuw_multi_robot_demo demo.launch"


    controller_command = " launch_local_controller_one_for_all:=true"
    world_command = " room:=" + world
    robot_command = " nr_of_robots:=" + str(nr_of_robots)
    command = base_command + controller_command + world_command + robot_command
    process = subprocess.Popen(command, shell=True, start_new_session=True)

    return process

def main():
    nr_of_runs = 3
    nr_of_robots = 8
    world = "warehouse008"

    # mrrp_process = launch_mrrp(nr_of_robots, world)
    # time.sleep(10)

    # for i in range(nr_of_robots):
    #     publish_stop_command(i)

    for i in range(nr_of_runs):

        print(f"Run {i + 1} of {nr_of_runs}", file=sys.stderr)

        goal_saver_process = launch_goal_saver("/home/robot/goals/goals", i)
        time.sleep(5)

        # Publish goals
        #random_goals_process = launch_random_goal(nr_of_robots)        
        #time.sleep(60)
        

        # Get Success msg from planner
        #subprocess.run("rostopic echo -n 1 /planner_status", shell=True)

        # Reset stage
        #subprocess.Popen("rosservice call /reset_positions", shell=True)

        # Check if all processes finished or kill them
        if goal_saver_process.poll() is not None:
            print("The goal_saver_process has terminated.")
        else:
            print("The goal_saver_process is still running.")
            goal_saver_process.terminate()
        
        # if random_goals_process.poll() is not None:
        #     print("The random_goals_process has terminated.")
        # else:
        #     print("The random_goals_process is still running.")
        #     random_goals_process.terminate()

        time.sleep(1)
        goal_saver_process.wait()
        #random_goals_process.wait()

    # if mrrp_process.poll() is not None:
    #     print("The mrrp_process has terminated.")
    # else:
    #     print("The mrrp_process is still running.")
    #     mrrp_process.terminate()


if __name__ == "__main__":
    main()