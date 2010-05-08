#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
//#include "sensor_msgs/LaserScan.h"
#include <std_msgs/String.h>
#include <sstream>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define PI 3.14159265

//robot's angular orientation
double w=0;
//robot's x position
double Vx=0;
//robot's y position
double Vy=0;

double cmds[10]; //list of command values to take
int cmdTypes[10];//list of what type of command to take(move straight, rotate, or reset vision)
int numCmds;     //number of commands that are queued  
double VxLast;

//callback to store 0dometry data
void recordOdom(const nav_msgs::Odometry::ConstPtr& data){
    Vx=data->twist.twist.linear.x - VxLast;
    Vy+=data->twist.twist.linear.y;
    //get Quaternion anglular information
    //double x=data->pose.pose.orientation.x;
    //double y=data->pose.pose.orientation.y;
    //double z=data->pose.pose.orientation.z;
    //double w=data->pose.pose.orientation.w;
    //convert to pitch
    //angle=atan2(2*(y*x+w*z),w*w+x*x-y*y-z*z);
    w+= data->twist.twist.angular.z;
}

//Motion Planning, move lego at p1 to p2
void moveLego(double p1x, double p1y, double p2x, double p2y){

//TODO: set motion commands here


}

void resetEnc()
{
    VxLast += Vx;
    Vy = 0;
    w  = 0;
}

int main(int argc, char** argv)
{

  VxLast = 0;
  //init node
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  //connect to topic "cmd_vel" to control robot velocity
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  //these start the threads that call the recordLaser and recordOdom when the data is ready
  //ros::Subscriber sub_laser = n.subscribe("base_scan", 100, recordLaser);
  ros::Subscriber sub_odom = n.subscribe("base_pose_ground_truth", 100, recordOdom);
  resetEnc();
  srand ( time(NULL) ); //initialize random seed

  double rate = 3;
  numCmds = 0;		//set to no current commands

  //set up loop to run at <<rate>>Hz
  ros::Rate loop_rate(rate);
  int done=0;
  double dist = 0;
  double angle= 0;
  while (ros::ok()&&!done)
  {
    //Update encoder localization
    dist=Vx;
    angle=w;

    //print the location and reset cmds
    ROS_INFO("Bot at dist=%f angle=%f",dist,angle);
    geometry_msgs::Twist cmd;
    cmd.linear.x=0;
    cmd.angular.z=0;

    //If there are no commands to take, do image processing and call high level planning
    if (numCmds <= 0 || cmdTypes[numCmds-1] == 2){

    numCmds = 2;
    cmdTypes[0] = 0;
    cmds[0]     = 0.61;
    cmdTypes[1] = 1;
    cmds[1]     = PI*((double)(rand()%10-5))/5;
    //TODO: call image processing function, call planner
    //High level planner must end up setting at least one command to process
    }

    //Execute current command
    if (cmdTypes[numCmds-1] == 0){ //go forward or backward a certain distance
        if(cmds[numCmds-1]>0){ //going forward
            if (dist>=cmds[numCmds-1]){ //if has gone far enough
                resetEnc();
                numCmds--;
            }else{ //otherwise keep going
                cmd.linear.x = 0.002;
            }
        }else{ //going backwards
            if (dist<=cmds[numCmds-1]){ //if has gone far enough
                resetEnc();
                numCmds--;
            }else{ //otherwise keep going
                cmd.linear.x = -0.002;
            }
        }

    }else if(cmdTypes[numCmds-1] == 1){//rotate by a certain angle
        if(cmds[numCmds-1]>0){ //roatating ccw
            if (angle>=cmds[numCmds-1]){ //if has gone far enough
                resetEnc();
                numCmds--;
            }else{ //otherwise keep going
                cmd.angular.z = .05;
            }
        }else{ //rotating cw
            if (angle<=cmds[numCmds-1]){ //if has gone far enough
               resetEnc();
               numCmds--;
            }else{ //otherwise keep going
                cmd.angular.z = -.05;
            }
       }
    }

    //send command 
    pub.publish(cmd);

    //allow other threads a chance and wait for next loop iteration
    ros::spinOnce();
    loop_rate.sleep();
  }
  //#####################################################################################################}
