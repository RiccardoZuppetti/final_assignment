/*Program that allows the robot to follow the wall*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "final_assignment/wall_follow.h" 
#include "nav_msgs/Odometry.h"
#include <string>
#include "sensor_msgs/LaserScan.h"


// Structure

struct regions{
    float right;
	float fright;
	float front;
    float fleft;
	float left;
};

// Global variables

using namespace nav_msgs;
int init;
ros::Publisher publisher;
geometry_msgs::Twist msg;
int curr_state=0;
int stop_mode=0;
regions regions_;

// Functions

bool wall_follow_mode(final_assignment::wall_follow::Request &req, final_assignment::wall_follow::Response &res);
void clbk_laser(const sensor_msgs::LaserScan:: Ptr& x);
float calculateMinimum(float max, sensor_msgs::LaserScan array, int indexMin, int indexMax);
void next_state();
void change_state(int new_state);

void find_wall();
void turn_left();
void follow_the_wall();
void stop();

// main
int main(int argc, char **argv){
    ros::init(argc, argv, "wall_follower");

    ros::NodeHandle n1;
	ros::NodeHandle n2;
	ros::NodeHandle n3;

    // subscriber to the topic /scan
	// publisher to the topic /cmd_vel
	// server /wall_follow

    ros::Subscriber robot_position=n1.subscribe("/scan", 1000, clbk_laser);
	publisher=n2.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::ServiceServer service=n3.advertiseService("/wall_follow", wall_follow_mode);
    ros::Rate loop_rate(10);
    // loop that execute the required action following the wall
    while(ros::ok()){
        // action received --> follow the wall
        if(init==1){
            if(curr_state==0){
                find_wall();
            }
            if(curr_state==1){
                turn_left();
            }
            if(curr_state==2){
                follow_the_wall();
            }
        }
        // different action received --> stop to follow the wall
        else{
            if(stop_mode==1){
                stop();
                stop_mode=0;
            }
            loop_rate.sleep();
        }
        ros::spinOnce();
    }
    return 0;
}

// function that reads the input from the laser sensor in order to establish which will be the next state

void next_state(){
    regions test_region;
    test_region=regions_;
    msg.linear.x=0;
	msg.angular.z=0;
	int d0=1;
	int d=1.5;
    // depending on the input received from the sensors and computed in 'clbk_laser', the next state is decided
    if ((regions_.front>d0) & ( regions_.fleft>d) & (regions_.fright>d)){
		//nothing
		change_state(0);
	}
	else if ((regions_.front<d0) & (regions_.fleft>d) & (regions_.fright>d)){
		//front
		change_state(1);
	}
	else if ((regions_.front>d0) & ( regions_.fleft>d)& (regions_.fright<d)){
		//fright
		change_state(2);
	}		
	else if ((regions_.front>d0) & (regions_.fleft<d) & (regions_.fright>d)){
	    //fleft
		change_state(0);
	}
	else if ((regions_.front<d0) & (regions_.fleft>d) & (regions_.fright<d)){
		//front and fright
		change_state(1);
	}
	else if ((regions_.front<d0) & (regions_.fleft<d) & (regions_.fright>d)){
		//front and fleft
		change_state(1);
	}
	else if ((regions_.front<d0) & (regions_.fleft<d) & (regions_.fright<d)){
		//front and fleft and fright
		change_state(1);
	}
	else if ((regions_.front>d0) & (regions_.fleft<d) & (regions_.fright<d)){
		//fleft and fright
		change_state(1);
	}
	else{
		//unknown
	}
}

// function when a call from the client is received and the "init" variable changes
// init == 1 --> follow the wall / init == 0 --> stop follow the wall

bool wall_follow_mode(final_assignment::wall_follow::Request &req, final_assignment::wall_follow::Response &res){
    bool val;
    init=req.data;
    ROS_INFO("Init: %d", init); // Show the request
    res.success=1;
    ROS_INFO("Switched mode in following the wall");
	stop_mode=1;
	return true;
}

// function that calculates the minimum between max and some elements of an array

float CalculateMinimum(float max, sensor_msgs::LaserScan array, int indexMin, int indexMax){	
	float TempMin=max;
	int i;
	for (i=indexMin; i<=indexMax; i++){
			if (array.ranges[i]<=TempMin)
				{
					TempMin=array.ranges[i];
				}
		}
	return TempMin;
}

// function called when new data are published on the topic /scan

void clbk_laser(const sensor_msgs::LaserScan:: Ptr& x){	
	sensor_msgs::LaserScan rangeTemporary;
	rangeTemporary.ranges=x->ranges;
	// for each region I read the inputs from the laser and assign the minimum of them
	regions_.right=CalculateMinimum(10, rangeTemporary,0,143);
	regions_.fright=CalculateMinimum(10, rangeTemporary,144,287);	
	regions_.front=CalculateMinimum(10, rangeTemporary,288,431);
	regions_.fleft=CalculateMinimum(10, rangeTemporary,432,575);		
	regions_.left=CalculateMinimum(10, rangeTemporary,576,713);
	next_state();
}

// function that changes the current state

void change_state(int new_state){
    // show on screen the new_state, if the new_state is different from the current one
    if(new_state!=curr_state){
        if(new_state==0){
            ROS_INFO("Wall follower - Find the wall");
        }
        if(new_state==1){
            ROS_INFO("Wall follower - Turn left");
        }
        if(new_state==2){
            ROS_INFO("Wall follower - Follow the wall");
        }
    }
    curr_state=new_state; // update the state
}

// function that allows the robot to go forward and rotate to the right

void find_wall(){
	msg.linear.x=0.3;
	msg.angular.z=-0.6;
	publisher.publish(msg);
}

// function to rotate to the left

void turn_left(){
	msg.angular.z=0.8;
	publisher.publish(msg);
}

// function to go forward

void follow_the_wall(){
	msg.linear.x=0.5;
	publisher.publish(msg);
}

// function that stops the robot

void stop(){
	msg.linear.x=0;
	msg.angular.z=0;
	publisher.publish(msg);
}








