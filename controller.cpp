#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
//#include "nav_msgs/Odometry.h"
//#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <math.h>
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "SegmentList.h"

using namespace std;

const int IMAGE_WIDTH = 320;
const int IMAGE_HEIGHT = 240;
const int PRE_MEDIAN_SIZE = 5;
const int MEDIAN_SIZE = 15;
//const string SIGMA = "0.5";
//const string K = "500";
//const string MIN = "50";
const char* COMMAND = "./segment 1.00 500 50 temp.ppm temp2.ppm";

const int UPPER_RECT = 200;
const int LOWER_RECT = 160;
const int LEFT_BOUND = 50;
const int RIGHT_BOUND = 270;
const int MIN_COUNT = 40;

int xyToIndex3(int x, int y){
	return 3 * (x + y * IMAGE_WIDTH);
}

IplImage* image = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
IplImage* segment;
//IplImage* preblur = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3);
IplImage* blur = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_8U, 3);

enum Command {FORWARD, REVERSE, LEFT, RIGHT, HALT};

Command command = HALT;
bool Ready = true;

//callback to store image
	void recordImage(const sensor_msgs::Image::ConstPtr& data){
		if(Ready)
			return;
		sensor_msgs::CvBridge bridge_;
		//  cvReleaseImage(&image);
		image = bridge_.imgMsgToCv(data, "bgr8");

		cvSmooth(image, preblur, CV_MEDIAN, PRE_MEDIAN_SIZE);

		cvSaveImage("temp.ppm", preblur);
		//  system("./segment " + SIGMA + " " + K + " " + MIN + " temp.ppm temp2.ppm");
		system(COMMAND);

		segment = cvLoadImage("temp2.ppm");

		cvSmooth(segment, blur, CV_MEDIAN, MEDIAN_SIZE);

		//  cvShowImage("Input", image);
		//  cvShowImage("Preblur", preblur);
		//  cvShowImage("Segment", segment);
		cvShowImage("Blur", blur);

		// begin object detection code
		bool leftOK = true;
		bool frontOK = true;
		bool rightOK = true;

		SegmentList* list = new SegmentList();
		for(int j = LOWER_RECT; j < UPPER_RECT; j++)
			for(int i = 0; i < IMAGE_WIDTH; i++){
				unsigned int colour = 0;
				int index = xyToIndex3(i, j);
				colour = colour | blur->imageData[index];
				colour = colour | (blur->imageData[index + 1] << 8);
				colour = colour | (blur->imageData[index + 2] << 16);
				list->add(colour, i, j);
			}


		SegmentNode* floor = list->first;
		SegmentNode* temp = list->first;
		while(temp != NULL){
			if(temp->size() > floor->size())
				floor = temp;
			temp = temp->next;
		}

		list->remove(floor->colour);
		temp = list->first;
		while(temp != NULL){
			if(temp->size() > MIN_COUNT){
				if(temp->right > LEFT_BOUND && temp->left < RIGHT_BOUND)
					frontOK = false;
				if(temp->left < LEFT_BOUND)
					leftOK = false;
				if(temp->right > RIGHT_BOUND)
					rightOK = false;
			}
			temp = temp->next;
		}

		if(frontOK){
			command = FORWARD;
			cout << "FORWARD\n";
		}else if(leftOK){
			command = LEFT;
			cout << "LEFT\n";
		}else{
			command = RIGHT;
			cout << "RIGHT\n";
		}

		// end object detection code

		cvWaitKey(30);
		Ready = true;
	}

int main(int argc, char** argv)
{
	//init node
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;
	//connect to topic "cmd_vel" to control robot velocity
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	//these start the threads that call the recordLaser and recordOdom when the data is ready
	//  ros::Subscriber sub_laser = n.subscribe("base_scan", 100, recordLaser);
	//  ros::Subscriber sub_odom = n.subscribe("base_pose_ground_truth", 100, recordOdom);

	ros::Subscriber sub_image = n.subscribe("images", 100, recordImage);

	//################################# MODIFY HERE #######################################################

	//set up loop to run at 5Hz
	ros::Rate loop_rate(15);
	int done=0;
	//  int count=0;

	while (ros::ok()&&!done)
	{
		/*
		//print the location and laser data
		ROS_INFO("Bot at x=%f y=%f ang=%f distance forward=%f left=%f right=%f",xPos,yPos,angle/PI*180,wallDistForward,wallDistLeft,wallDistRight);

*/
		geometry_msgs::Twist cmd;

		switch(command){
			case FORWARD:
				cmd.angular.z = 0;
				cmd.linear.x = 5;
				cout << "*FORWARD\n";
				break;
			case REVERSE:
				cmd.angular.z = 0;
				cmd.linear.x = -5;
				break;
			case LEFT:
				cmd.angular.z = -0.4;
				cmd.linear.x = 0;
				cout << "*LEFT\n";
				break;
			case RIGHT:
				cmd.angular.z = 0.4;
				cmd.linear.x = 0;
				cout << "*RIGHT\n";
				break;
			case HALT:
				cmd.angular.z = 0;
				cmd.linear.x = 0;
				break;
		}


		/*
		   cmd.angular.z=0;

		   if(count<=20)
		   cmd.linear.x=.1;
		   else if(count>20 && count<=40)
		   cmd.linear.x=-.1;
		   else
		   cmd.linear.x=0;

		   count=count+1;
		   */
		//	cmd.angular.z=0;
		/*
		   if (angle>-PI/2){
		//turn right ie. turn clockwise at a rate of .1 rad/sec
		cmd.angular.z=-.1;
		//while the robot still has more distance to travel keep going
		}else if( yPos>-4){
		//move forward 
		cmd.linear.x=1;
		}else{            
		//stop the robot
		done=1;
		}
		*/
		//send command
		if(Ready)
			pub.publish(cmd);
		Ready = false;
		//allow other threads a chance and wait for next loop iteration
		ros::spinOnce();
		loop_rate.sleep();

	}
	//#####################################################################################################

	cvReleaseImage(&image);
	cvReleaseImage(&segment);
	cvReleaseImage(&preblur);
	cvReleaseImage(&blur);


}
