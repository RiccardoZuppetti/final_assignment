/*Program that implemets the robot behaviour*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "final_assignment/random.h"
#include "final_assignment/wall_follow.h"


// Global variables

using namespace nav_msgs;

ros::Publisher publisher;
ros::ServiceClient client1;
ros::ServiceClient client2;

move_base_msgs::MoveBaseActionGoal goal_position;	// position to reach
nav_msgs::Odometry curr_position;   // current position of the robot
nav_msgs::Odometry target_position; // target (in order to calculate the distance between the terget itself (which corresponds to the goal) and the current position of the robot)

float distance_along_x=0;
float distance_along_y=0;

// states of an FSM (in order to be able to change the state when a command is given)
int prev_state=0;
int curr_state=0;

int init;
int count=1; // useful to print the distances at each specific iterations


// Functions

int update_state(void);
void positionCallback(const Odometry:: Ptr& x);

// main

int main(int argc, char **argv){
    ros::init(argc, argv, "robot_interface");

    ros::NodeHandle n1;
    ros::NodeHandle n2;
    ros::NodeHandle n3;
    ros::NodeHandle n4;

    // initialization of the 4 nodes needed
	// 2 nodes corresponds to the clients of the custom services implemented
	// publisher needed for the /move_base/goal topic
	// subscriber needed in order to retrieve the current position of the robot (positionCallback)

    publisher=n1.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1);
	client1=n2.serviceClient<final_assignment::random>("random");
	client2=n3.serviceClient<final_assignment::wall_follow>("wall_follow");
	ros::Subscriber robot_position=n4.subscribe("odom", 1000, positionCallback);

    ros::spin();
    return 0;
}

// function that retrieves the parameter /state and put it into an integer variable 

int update_state(void){
	int newstate;
	ros::param::get("/state", newstate);
	return newstate;
}

// function called when new data are available on the /odom topic, in order to perform an action by the robot

void positionCallback(const Odometry:: Ptr& x){
    float target_along_x;
    float target_along_y;
    // every time that the callback is recalled it is needed to know the current position of the robot
    curr_position.pose.pose.position.x=x->pose.pose.position.x;
    curr_position.pose.pose.position.y=x->pose.pose.position.y;
    count=count+1;
    // first state --> go to a random position
    if(prev_state==1){
        // initialization operations
        if(init==1){
            ROS_INFO("State 1 - Go to a Random Position");
            // init operations should be done only once
            init=0;
            // computing the random position to reach
            final_assignment::random srv;
            srv.request.min=0;
            srv.request.max=5;
            client1.call(srv);
            // initialization of the goal_pose for the move_base algorithm
            goal_position.goal.target_pose.header.frame_id="map";
			goal_position.goal.target_pose.pose.orientation.w=1;
			goal_position.goal.target_pose.pose.position.x=srv.response.a;
			goal_position.goal.target_pose.pose.position.y=srv.response.b;
            // update the target in order to compute the distances
            target_position.pose.pose.position.x=goal_position.goal.target_pose.pose.position.x;
            target_position.pose.pose.position.y=goal_position.goal.target_pose.pose.position.y;
            publisher.publish(goal_position); 
        }
        // compute the distances
        distance_along_x=(target_position.pose.pose.position.x)-(curr_position.pose.pose.position.x);
        distance_along_y=(target_position.pose.pose.position.y)-(curr_position.pose.pose.position.y);
        // print on the screen the distance each 500 iterations
        if(count%500==0)
            ROS_INFO("Distances: %f \t %f", distance_along_x,distance_along_y);
        // check if the random target is reached by the robot
        if((distance_along_x>=-1 && distance_along_x<=1) && (distance_along_y>=-1 && distance_along_y<=1)){
            prev_state=4; // go to stop state
            init=1; // ready to execute a new action
            ROS_INFO("Random position reached!");
        }
    }
    // second state --> go to a specific position
    else if(prev_state==2){
        if(init==1){
            ROS_INFO("State 2 - Insert a position to reach");
            init=0;
            printf("Choose between:\n(-4,-3)\t(-4,2)\t(-4,7)\t(5,-7)\t(5,-3)\t(5,-1)\nx:");
            std::cin >> target_along_x;
		    printf("\ny:");
			std::cin >> target_along_y;
            // check if the target is feasible
            if(target_along_x==-4){
                if((target_along_y==-3) || (target_along_y==2) || (target_along_y==7)){
                    printf("Position Valid");
                    // set the goal
                    goal_position.goal.target_pose.header.frame_id="map";
			        goal_position.goal.target_pose.pose.orientation.w=1;
			        goal_position.goal.target_pose.pose.position.x=target_along_x;
			        goal_position.goal.target_pose.pose.position.y=target_along_y;
                    // update the target
                    target_position.pose.pose.position.x=goal_position.goal.target_pose.pose.position.x;
                    target_position.pose.pose.position.y=goal_position.goal.target_pose.pose.position.y;
                    publisher.publish(goal_position);
                    init=0;
                }
                else
                    printf("\n Position not valid: chose between:\n(-4,-3)\t(-4,2)\t(-4,7)\t(5,-7)\t(5,-3)\t(5,-1)\n");
            }
            if(target_along_x==5){
                if((target_along_y==-7) || (target_along_y==-3) || (target_along_y==-1)){
                    printf("Position Valid");
                    // set the goal
                    goal_position.goal.target_pose.header.frame_id="map";
			        goal_position.goal.target_pose.pose.orientation.w=1;
			        goal_position.goal.target_pose.pose.position.x=target_along_x;
			        goal_position.goal.target_pose.pose.position.y=target_along_y;
                    // update the target
                    target_position.pose.pose.position.x=goal_position.goal.target_pose.pose.position.x;
                    target_position.pose.pose.position.y=goal_position.goal.target_pose.pose.position.y;
                    publisher.publish(goal_position);
                    init=0;
                }
                else
                    printf("\n Position not valid: chose between:\n(-4,-3)\t(-4,2)\t(-4,7)\t(5,-7)\t(5,-3)\t(5,-1)\n");
            }
        }
        // compute the distances
        distance_along_x=(target_position.pose.pose.position.x)-(curr_position.pose.pose.position.x);
        distance_along_y=(target_position.pose.pose.position.y)-(curr_position.pose.pose.position.y);
        // print on the screen the distance each 500 iterations
        if(count%500==0)
            ROS_INFO("Distances: %f \t %f", distance_along_x,distance_along_y);
        // check if the random target is reached by the robot
        if((distance_along_x>=-1 && distance_along_x<=1) && (distance_along_y>=-1 && distance_along_y<=1)){
            prev_state=4; // go to stop state
            init=1; // ready to execute a new action
            ROS_INFO("Selected position reached!");
        }
    }
    // third state --> start to follow the wall
    else if(prev_state==3){
        if(init==1){
            ROS_INFO("State 3 - Follow the wall");
            init=0;
            int res;
            // call the wall_follow srv to start to follow the wall
            final_assignment::wall_follow srv_init;
            srv_init.request.data=1;
            client2.call(srv_init);
            res=srv_init.response.success;
        }
        else{
            curr_state=update_state(); //update the state
            // the state is not the same as before (if it is equal continue to follow the wall)
            if(curr_state!=prev_state){ 
                prev_state=4; // go to stop state
                init=1; // new action to do
                // recall the wall_follow srv in order to stop to follow the wall
                final_assignment::wall_follow srv_stop;
                srv_stop.request.data=0;
                client2.call(srv_stop);
                ROS_INFO("Stop to Follow the wall");
            }
            else{
                // do nothing
            }
        }
    }
    // fourth state --> stop in the last position
    else if(prev_state==4){
        if(init==1){
            ROS_INFO("State 4 - Stop in the last position");
            init=0;
            // set the goal as the current position of the robot
            goal_position.goal.target_pose.header.frame_id="map";
			goal_position.goal.target_pose.pose.orientation.w=1;
			goal_position.goal.target_pose.pose.position.x=curr_position.pose.pose.position.x;
			goal_position.goal.target_pose.pose.position.y=curr_position.pose.pose.position.y;
            // no need to update the target because there is no distance to compute
        }
        else{
            curr_state=update_state(); // update the state
            // the state is not the same (if it is equal continue to stay in that position)
            if(curr_state!=prev_state){
                prev_state=curr_state;
                init=1; // new action to do
            }
            else{
                // do nothing
            }
        }
    }
    // if previous state is not any of the states before update the state
    else{
        curr_state=update_state();
        prev_state=curr_state;
    }
}







