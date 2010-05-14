//the controller.cpp
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/String.h>
#include <sstream>
#include <math.h>
// OpenCV libraries
#include <cv.h>
#include <highgui.h>

#include "/home/jn/svn4/legodetect.h"

#define PI 3.14159265

//distance to object in front of robot
double wallDistForward=0;
//distance to object to the left of robot
double wallDistLeft=0;
//distance to object to the right of robot
double wallDistRight=0;

//robot's angular orientation
double angle=0;
//robot's x position
double xPos=0;
//robot's y position
double yPos=0;


//callback to store laser data
void recordLaser(const sensor_msgs::LaserScan::ConstPtr& data){
	//get numer of laser samples
	double numAng=(data->angle_max-data->angle_min)/data->angle_increment;
	wallDistForward=data->ranges[(int)(numAng/2)];
	wallDistRight=data->ranges[(int)(numAng/2-360)];
	wallDistLeft=data->ranges[(int)(numAng/2+360)];
}

//callback to store 0dometry data
void recordOdom(const nav_msgs::Odometry::ConstPtr& data){
	xPos=data->pose.pose.position.x;
	yPos=data->pose.pose.position.y;
	//get Quaternion anglular information
	double x=data->pose.pose.orientation.x;
	double y=data->pose.pose.orientation.y;
	double z=data->pose.pose.orientation.z;
	double w=data->pose.pose.orientation.w;
	//convert to pitch
	angle=atan2(2*(y*x+w*z),w*w+x*x-y*y-z*z);
}

int main(int argc, char** argv)
{
	//init node
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;
	//connect to topic "cmd_vel" to control robot velocity
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	//these start the threads that call the recordLaser and recordOdom when the data is ready
	ros::Subscriber sub_laser = n.subscribe("base_scan", 100, recordLaser);
	ros::Subscriber sub_odom = n.subscribe("base_pose_ground_truth", 100, recordOdom);


	//################################# MODIFY HERE #######################################################

	//set up loop to run at 5Hz
	ros::Rate loop_rate(3);
	int done=0;
	double dist = 0;
	while (ros::ok()&&!done)
	{


		//print the location and laser data
		ROS_INFO("Bot at x=%f y=%f ang=%f distance forward=%f left=%f right=%f",xPos,yPos,angle/PI*180,wallDistForward,wallDistLeft,wallDistRight);
		geometry_msgs::Twist cmd;
		cmd.linear.x=0;
		cmd.angular.z=0;

		if (dist<10){
			cmd.linear.x=1;
		}else{
			cmd.linear.x=0;
		}
		//get image from ros
		//IplImage* imgMsgToCv(sensor_msgs::Image::ConstPtr image_message, string cv_encoding="passthrough");
		getLegoPosition();
		//printf("DEBUGGGGG!\n");

		//send command
		pub.publish(cmd);
		dist+=cmd.linear.x/2.0;

		//allow other threads a chance and wait for next loop iteration
		ros::spinOnce();
		loop_rate.sleep();

	}
	//#####################################################################################################
}
