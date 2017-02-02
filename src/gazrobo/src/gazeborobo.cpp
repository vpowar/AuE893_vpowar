#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;



const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 10.0;
const double y_max = 10.0;

const double PI = 3.14159265359;

void move(double speed, double distance, bool isForward);
void rotate (double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
//void setDesiredOrientation (double desired_angle_radians);

//void moveGoal(turtlesim::Pose  goal_pose, double distance_tolerance);
void gridClean();
//void spiralClean();

int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "gazrobo");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;


	velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);
	//pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	//ros::Rate loop_rate(10);



	/** test your code here **/
	
	ros::Rate loop_rate(1);
	/*setDesiredOrientation(degrees2radians(120));
	loop_rate.sleep();
	setDesiredOrientation(degrees2radians(-60));
	loop_rate.sleep();
	setDesiredOrientation(degrees2radians(0));*/

	/*turtlesim::Pose goal_pose;
	goal_pose.x=1;
	goal_pose.y=1;
	goal_pose.theta=0;
	moveGoal(goal_pose, 0.01);
	loop_rate.sleep();
	 */

	gridClean();

	//spiralClean();

	//ros::spin();

	return 0;
}

/**
 *  makes the robot move with a certain linear velocity for a
 *  certain distance in a forward or backward straight direction.
 */
void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);

}


void rotate (double angular_speed, double relative_angle, bool clockwise){

	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise)
		vel_msg.angular.z =-abs(angular_speed);
	else
		vel_msg.angular.z =abs(angular_speed);

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle<relative_angle);

	vel_msg.angular.z =0;
	velocity_publisher.publish(vel_msg);

}

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}


double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}



void gridClean(){

	ros::Rate loop(1);

	loop.sleep();
	//horizontal line across the grid
	move(3, 5, true);
	loop.sleep();
	rotate(degrees2radians(5), degrees2radians(90), true);
	loop.sleep();	
	move(3,5,true);
	loop.sleep();	
	rotate(degrees2radians(5), degrees2radians(90), true);
	loop.sleep();
	move(3, 5, true);
	rotate(degrees2radians(5), degrees2radians(90), false);
	loop.sleep();
	move(3,5,true);
	loop.sleep();
	rotate(degrees2radians(5), degrees2radians(90), false);


	/*vertical line	
	move(1, 9, true);


	rotate(degrees2radians(5), degrees2radians(90), false);
	loop.sleep();
	move(0.5, 1, true);
	rotate(degrees2radians(5), degrees2radians(90), false);
	loop.sleep();
	move(0.5, 9, true);

	rotate(degrees2radians(5), degrees2radians(90), true);
	loop.sleep();
	move(0.5, 1, true);
	rotate(degrees2radians(5), degrees2radians(90), true);
	loop.sleep();
	move(0.5, 9, true);
	


	rotate(degrees2radians(5), degrees2radians(90), false);
	loop.sleep();
	move(0.5, 1, true);
	rotate(degrees2radians(5), degrees2radians(90), false);
	loop.sleep();
	move(0.5, 9, true);

	rotate(degrees2radians(5), degrees2radians(90), true);
	loop.sleep();
	move(0.5, 1, true);
	rotate(degrees2radians(5), degrees2radians(90), true);
	loop.sleep();
	move(0.5, 9, true);
*/
	//double distance = getDistance(turtlesim_pose.x, turtlesim_pose.y, x_max, y_max);

 }



