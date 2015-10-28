#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;	// to determine the position for turning the robot in an absolute orientation --> in the setDesiredOrientation fn
turtlesim::Pose turtlesim_pose;

const double PI = 3.14159265359;

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool cloclwise);	//this will rotate the turtle at specified angle from its current angle
double degrees2radians(double angle_in_degrees);		
double setDesiredOrientation(double desired_angle_radians);	//this will rotate the turtle at an absolute angle, whatever its current angle is
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);	//Callback fn everytime the turtle pose msg is published over the /turtle1/pose topic.

int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "turtlesim_cleaner");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);	//call poseCallback everytime the turtle pose msg is published over the /turtle1/pose topic.
	//ros::Rate loop_rate(10);

	//	/turtle1/cmd_vel is the Topic name
	//	/geometry_msgs::Twist is the msg type 
	ROS_INFO("\n\n\n ********START TESTING*********\n");
	/*
	cout<<"enter speed: ";
	cin>>speed;
	cout<<"enter distance: ";
	cin>>distance;
	cout<<"forward?: ";
	cin>>isForward;
	move(speed, distance, isForward);
						//this is to move and rotate the robot as the user.
	cout<<"enter angular velocity: ";
	cin>>angular_speed;
	cout<<"enter angle: ";
	cin>>angle;
	cout<<"Clockwise?: ";
	cin>>clockwise;
	rotate(degrees2radians(angular_speed), degrees2radians(angle), clockwise);
	*/

	setDesiredOrientation(degrees2radians(120));
	ros::Rate loop_rate(0.5);
	loop_rate.sleep();
	setDesiredOrientation(degrees2radians(-60));
	loop_rate.sleep();
	setDesiredOrientation(degrees2radians(0));

	ros::spin();

	return 0;
}

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

	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(1000);
	   do{
		   velocity_publisher.publish(vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = angular_speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
		   //cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
	   }while(current_angle<relative_angle);
	   vel_msg.angular.z =0;
	   velocity_publisher.publish(vel_msg);
}

/**
 *  converts angles from degree to radians  
 */

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

/**
 *  turns the robot to a desried absolute angle  
 */
double setDesiredOrientation(double desired_angle_radians)
{	
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	//if we want to turn at a perticular orientation, we subtract the current orientation from it
	bool clockwise = ((relative_angle_radians<0)?true:false);
	//cout<<desired_angle_radians <<","<<turtlesim_pose.theta<<","<<relative_angle_radians<<","<<clockwise<<endl;
	rotate (abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
}

/**
 *  callback function to update the pose of the robot  
 */

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}


