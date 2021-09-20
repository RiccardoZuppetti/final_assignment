/*Program that implements an user interface to select the next action that the robot should do*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "move_base_msgs/MoveBaseActionGoal.h"

// main

int main(int argc, char **argv){
    ros::init(argc, argv, "commands");
    int state=0;
    while(1){
        printf("Next action to take \n 1. Reach a random position \n 2. Choose next position \n 3. Start to follow the wall \n 4. Stop in the last position\n");
   		std::cin >> state;
        ros::param::set("/state", state); // update the state
    }
    ros::spin();
    return 0;
}
