#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
//#include "sensor_msgs/LaserScan.h"
#include <std_msgs/String.h>
#include <sstream>
#include <stdlib.h>
#include <math.h>
#include <time.h>
// OpenCV libraries
#include <cv.h>
#include <highgui.h>
using namespace std;

#include "legodetect.h"


#define PI 3.14159265





//robot's angular orientation
double w=0;
//robot's x position
double Vx=0;
//robot's y position
double Vy=0;

int tmpBool = 0;

//Kalman filter params
double A[3][3] = {{1.0, 0.0, 0.0},
                  {0.0, 1.0, 0.0},
                  {0.0, 0.0, 1.0}};
double B[3][3] = {{0.0, 0.0, 0.0},
                  {0.0, 0.0, 0.0},
                  {0.0, 0.0, 0.0}};
double I[3][3] = {{1.0, 0.0, 0.0},
                  {0.0, 1.0, 0.0},
                  {0.0, 0.0, 1.0}};
double H[3][3] = {{1.0, 0.0, 0.0},
                  {0.0, 1.0, 0.0},
                  {0.0, 0.0, 1.0}};
double Q[3][3] = {{05.20, 01.02, 0.02},
                  {01.02, 05.20, 0.02},
                  {0.02, 0.02, 0.20}};
double R[3][3] = {{300.10, 100.01, 05.1},
                  {100.01, 300.10, 05.1},
                  {05.1,  05.1, 25.50}};
double P[3][3] = {{10.10, 0.01, 0.01},
                  {0.01, 10.10, 0.01},
                  {0.01, 0.01, 1.10}};
double x[3][1] = {{0.0},{0.0},{0.0}};

double result[3][3],tmpArray[3][3],tmpArray1[3][3],tmpArray2[3][3];
double resultRow[3][1],tmpRow[3][1],tmpRow1[3][1],tmpRow2[3][1],xP[3][1],wA,legoX,legoY,legoA;
double legoYL1 = 0;
double legoYL2 = 0;
double angles[10] = {0,0,0,0,0,0,0,0,0,0};
int countA = 0;

double cmds[10]; //list of command values to take
int cmdTypes[10];//list of what type of command to take(move straight, rotate, or reset vision)
int numCmds;     //number of commands that are queued  
double VxLast,wLast,xI,yI,wI,xB,yB,wB;

//Encoder factors
  float factor = 0*0.75;
  float factorW= 0*3.0;
  //R3
  //float factor = 1.4;
  //float factorW= 3.00;
  //R5
  //float factor = 0.92;
  //float factorW= 1.90;
  // 4.25 	R2 at 3Hz
  // 1.5	R6 at 3Hz


void avgAngle()
{
  double ys = 0.0;
  double xs = 0.0;
  for (int i=0;i<10;i++){
    ys += sin(angles[i]);
    xs += cos(angles[i]);
  }
  wA = atan2(ys,xs);
}


//Motion Planning, move lego at p1 to p2
void moveLego(double p1x, double p1y, double p2x, double p2y){
//TODO: set motion commands here
}


// set legoX,legoY,legoA
void getLego(){
  legoYL2 = legoYL1;
  legoYL1 = legoY;
  getLegoPosition();
  legoX = legoPos[0];
  legoY = legoPos[1];
  legoA = atan2(legoY,legoX)-PI/2.0;      
  ROS_INFO("Lego: x=%f, y=%f, a=%f", legoX,legoY,legoA);
}


//Computes the inverse of tmpArray and stores it in result
void compInv()
{
double determintmpArraynt =    +tmpArray[0][0]*(tmpArray[1][1]*tmpArray[2][2]-tmpArray[2][1]*tmpArray[1][2])-tmpArray[0][1]*(tmpArray[1][0]*tmpArray[2][2]-tmpArray[1][2]*tmpArray[2][0])+tmpArray[0][2]*(tmpArray[1][0]*tmpArray[2][1]-tmpArray[1][1]*tmpArray[2][0]);

double invdet = 1/determintmpArraynt;
result[0][0] =  (tmpArray[1][1]*tmpArray[2][2]-tmpArray[2][1]*tmpArray[1][2])*invdet;
result[1][0] = -(tmpArray[0][1]*tmpArray[2][2]-tmpArray[0][2]*tmpArray[2][1])*invdet;
result[2][0] =  (tmpArray[0][1]*tmpArray[1][2]-tmpArray[0][2]*tmpArray[1][1])*invdet;
result[0][1] = -(tmpArray[1][0]*tmpArray[2][2]-tmpArray[1][2]*tmpArray[2][0])*invdet;
result[1][1] =  (tmpArray[0][0]*tmpArray[2][2]-tmpArray[0][2]*tmpArray[2][0])*invdet;
result[2][1] = -(tmpArray[0][0]*tmpArray[1][2]-tmpArray[1][0]*tmpArray[0][2])*invdet;
result[0][2] =  (tmpArray[1][0]*tmpArray[2][1]-tmpArray[2][0]*tmpArray[1][1])*invdet;
result[1][2] = -(tmpArray[0][0]*tmpArray[2][1]-tmpArray[2][0]*tmpArray[0][1])*invdet;
result[2][2] =  (tmpArray[0][0]*tmpArray[1][1]-tmpArray[1][0]*tmpArray[0][1])*invdet;
}

//Calc dot product of tmpArray1 and tmpArray2 and store in result
void matrixDot()
{
for (int i=0;i<=2;i++){
  for (int j=0;j<=2;j++){
    result[i][j] = tmpArray1[i][j]*tmpArray2[i][j];
  }
}
}

//Calc outer product of tmpArray1 and tmpArray2 and store in result
void matrixMult()
{
for (int i=0;i<=2;i++){
  for (int j=0;j<=2;j++){
    result[i][j] = 0;
    for (int i2=0;i2<=2;i2++){
        result[i][j] += tmpArray1[i][i2]*tmpArray2[i2][j];
    }
  }
}
}


//Calc transpose of tmpArray and store in result
void transpose()
{
for (int i=0;i<=2;i++){
  for (int j=0;j<=2;j++){
    result[i][j] = tmpArray[j][i];
  }
}
}

//Calc outer product of tmpArray and tmpRow and store in resultRow
void matrixMultRow()
{
for (int i=0;i<=2;i++){
  resultRow[i][0] = 0;
    for (int i2=0;i2<=2;i2++){
        resultRow[i][0] += tmpArray1[i][i2]*tmpRow[i2][0];
    }
}
}

//Calc addition of tmpArray1 and tmpArray2 and store in result
void matrixAdd(double factor)
{
for (int i=0;i<=2;i++){
  for (int j=0;j<=2;j++){
    result[i][j] = tmpArray1[i][j]+factor*tmpArray2[i][j];
  }
}
}
//Calc addition of tmpRow1 and tmpRow2 and store in resultRow
void matrixAddRow(double factor)
{
for (int i=0;i<=2;i++){
  resultRow[i][0] = tmpRow1[i][0]+factor*tmpRow2[i][0];
}
}

void calcB(double tmpA)
{
B[0][0] = sin(tmpA); 
B[0][1] = sin(tmpA);
B[0][2] = 0.0;
B[1][0] = -sin(tmpA);
B[1][1] =  cos(tmpA);
B[1][2] = 0.0;
B[2][0] = 0.0;
B[2][1] = 0.0;
B[2][2] = 1.0;
}

void updateKalman()
{

double z[3][1] = {{xI},{yI},{wI}};
double U[3][1]    = {{Vx-VxLast},{0.0},{w-wLast}};

//A*x
memcpy(tmpArray1, A, sizeof(double) * 9);
memcpy(tmpRow, x, sizeof(double) * 3);
matrixMultRow();
memcpy(tmpRow1, resultRow, sizeof(double) * 3);

//x = A*x+B*u
calcB(wI);
memcpy(tmpArray1, B, sizeof(double) * 9);
memcpy(tmpRow, U, sizeof(double) * 3);
matrixMultRow();
memcpy(tmpRow2, resultRow, sizeof(double) * 3);
matrixAddRow(1.0);
memcpy(x, resultRow, sizeof(double) * 3);

//P = A*P*AT + Q
memcpy(tmpArray, A, sizeof(double) * 9);
transpose();
memcpy(tmpArray2, result, sizeof(double) * 9);
memcpy(tmpArray1, P, sizeof(double) * 9);
matrixMult();
memcpy(tmpArray2,result, sizeof(double) * 9);
memcpy(tmpArray1,A, sizeof(double) * 9);
matrixMult();
memcpy(tmpArray1,result, sizeof(double) * 9);
memcpy(tmpArray2,Q, sizeof(double) * 9);
matrixAdd(1.0);
memcpy(P,result, sizeof(double) * 9);

//K = P*HT*inv(P*HT + R)
memcpy(tmpArray1, P, sizeof(double) * 9);
memcpy(tmpArray,  H, sizeof(double) * 9);
transpose();
memcpy(tmpArray2, result, sizeof(double) * 9);
matrixMult();
memcpy(tmpArray1, result, sizeof(double) * 9);
memcpy(tmpArray2, R, sizeof(double) * 9);
matrixAdd(1.0);
memcpy(tmpArray, result, sizeof(double) * 9);
compInv();
memcpy(tmpArray2, result, sizeof(double) * 9);
memcpy(tmpArray,  H, sizeof(double) * 9);
transpose();
memcpy(tmpArray1, result, sizeof(double) * 9);
matrixMult();
memcpy(tmpArray2, result, sizeof(double) * 9);
memcpy(tmpArray1, P, sizeof(double) * 9);
matrixMult();
double K[3][3];
memcpy(K, result, sizeof(double) * 9);

//x = x + K*(z-H*x)
memcpy(tmpRow, x, sizeof(double) * 3);
memcpy(tmpArray1, H, sizeof(double) * 9);
matrixMultRow();
memcpy(tmpRow2, resultRow, sizeof(double) * 3);
memcpy(tmpRow1, z, sizeof(double) * 3);
matrixAddRow(-1.0);
memcpy(tmpRow, resultRow, sizeof(double) * 3);
memcpy(tmpArray1, K, sizeof(double) * 9);
matrixMultRow();
memcpy(tmpRow2, resultRow, sizeof(double) * 3);
memcpy(tmpRow1, x, sizeof(double) * 3);
matrixAddRow(1.0);
memcpy(x, resultRow, sizeof(double) * 3);

//P = (1-K*H)*P
memcpy(tmpArray2, H, sizeof(double) * 9);
memcpy(tmpArray1, K, sizeof(double) * 9);
matrixMult();
memcpy(tmpArray2, result, sizeof(double) * 9);
memcpy(tmpArray1, I, sizeof(double) * 9);
matrixAdd(-1.0);
memcpy(tmpArray1, result, sizeof(double) * 9);
memcpy(tmpArray2, P, sizeof(double) * 9);
matrixMult();
memcpy(P, result, sizeof(double) * 9);
}

void resetEnc()
{
    VxLast = Vx;
    Vy = 0;
    wLast  = w;
}

void resetLastPos()
{
    xP[0][0] = xI;
    xP[1][0] = yI;
    xP[2][0] = wI;
}


//callback to store 0dometry data
void recordOdom(const nav_msgs::Odometry::ConstPtr& data){
    xI = data->pose.pose.position.x;
    yI = data->pose.pose.position.y;
    wI = data->pose.pose.orientation.z;
    Vx = factor*data->twist.twist.linear.x;
    w  = factorW*data->twist.twist.angular.z;
}

void goToZero()
{
  numCmds += 2;
  cmdTypes[numCmds-2] = 5;
  cmds[numCmds-2]     = 0.0;
  cmdTypes[numCmds-1] = 3;
  cmds[numCmds-1]     =  atan2(x[1][0],x[0][0]);
  //if (cmds[0] > 50){
   //  cmds[0] = 20;
  //}
  if (cmds[numCmds-2] < 5){
     cmds[numCmds-2] = 0;
     cmds[numCmds-1] = 0;
  }
  //cmds[1] = 0;
  //cmds[0] = 0;
}

void moveToLego()
{
  if(legoY > 0){
    numCmds++;
    cmdTypes[numCmds-1] = 2;
    cmds[numCmds-1] = 0;  
  }else{
    numCmds++;
    cmdTypes[numCmds-1] = 4;
  }
}

void search()
{ 
  if(legoY<0){
  numCmds++;
  cmdTypes[numCmds-1] = 2345;
  numCmds++;
  cmdTypes[numCmds-1] = 2345;
  numCmds++;
  cmdTypes[numCmds-1] = 4;
  }
}

void backup()
{
  numCmds++;
  cmdTypes[numCmds-1] = 0;
  cmds[numCmds-1] = -25;
}

void rotate(double a)
{
  numCmds++;
  cmdTypes[numCmds-1] = 1;
  cmds[numCmds-1] = a;
}

void goToZero2()
{
  numCmds = 2;
  cmdTypes[0] = 0;
  cmds[0]     = x[0][0];
  cmdTypes[1] = 2;
  cmds[1]     = x[1][0];
}

int main(int argc, char** argv)
{
  //init node
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  //connect to topic "cmd_vel" to control robot velocity
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  //these start the threads that call the recordLaser and recordOdom when the data is ready
  //ros::Subscriber sub_laser = n.subscribe("base_scan", 100, recordLaser);
  ros::Subscriber sub_odom = n.subscribe("base_pose_ground_truth", 100, recordOdom);
  srand ( time(NULL) ); //initialize random seed

  double rate = 2;
  numCmds = 0;		//set to no current commands

  //set up loop to run at <<rate>>Hz
  ros::Rate loop_rate(rate);
  int done=0;
  double dist = 0;
  double angle= 0;
  xI = 0.0;
  yI = 0.0;
  wI = 0.0;
  xB = 0.0;
  yB = 0.0;
  wB = 0.0;
  resetEnc();
  numCmds = 0;
    //2ft = 0.61m
  double thresh = 0.20;
  double threshWalk = 0.65;

  while (ros::ok()&&!done)
  {
    angles[countA%10] = wI;
    avgAngle();
    getLego();
    countA++;
    resetEnc();
    updateKalman();
    
    //Update travel info
    dist=  sqrt((xI-xP[0][0])*(xI-xP[0][0])+(yI-xP[1][0])*(yI-xP[1][0]));
    angle= wI;

    //print the location and reset cmds
    ROS_INFO("Bot at x=%f y=%f z=%f,angleWant=%f",x[0][0],x[1][0],x[2][0],cmds[numCmds-1]);
    ROS_INFO("Ovr at x=%f y=%f z=%f dist=%f,angle=%f",xI,yI,wI,dist,angle);
    geometry_msgs::Twist cmd;
    cmd.linear.x=0;
    cmd.linear.y=0;
    cmd.linear.z=0;
    cmd.angular.z=0;


    if(numCmds == 0){
      numCmds++;
      cmdTypes[numCmds-1] = 4;
      if(legoY>0){
        moveToLego();
      }    
    /*
     goToZero();
     numCmds++;
     cmdTypes[numCmds-1] = 2345;
     numCmds++;
     cmdTypes[numCmds-1] = 2345;
     numCmds++;
     cmdTypes[numCmds-1] = 2345;
     numCmds++;
     cmdTypes[numCmds-1] = 2345;
     numCmds++;
     cmdTypes[numCmds-1] = 2345;
     numCmds++;
     cmdTypes[numCmds-1] = 2345;
    */
    } 

    //If there are no commands to take, end.
    if (numCmds <= 0){
      done = 1;
    }


    //cmdTypes[1] = 1;
    //cmds[1]     = PI*((double)(rand()%10-5))/5;
    //TODO: call image processing function, call planner
    //High level planner must end up setting at least one command to process


    //Execute current command
    if (cmdTypes[numCmds-1] == 0){ //go forward or backward a certain distance
        if(cmds[numCmds-1]>=0){ //going forward
            if (dist>=cmds[numCmds-1]){ //if has gone far enough
                numCmds--;
                resetLastPos();
            }else{ //otherwise keep going
                cmd.linear.x = 5;
            }
        }else{ //going backwards
            if (dist>=-cmds[numCmds-1]){ //if has gone far enough
                numCmds--;
                resetLastPos();
            }else{ //otherwise keep going
                cmd.linear.x = -5;
            }
        }
    }else if(cmdTypes[numCmds-1] == 1){//rotate to a certain angle
        if(cmds[numCmds-1] > PI){
            cmds[numCmds-1] -= 2.0*PI;
        }else if(cmds[numCmds-1] < - PI){
            cmds[numCmds-1] += 2.0*PI;
        }
        if(angle-cmds[numCmds-1]>0){
            if(angle-cmds[numCmds-1]<thresh){
                numCmds--;
                resetLastPos();
            }else if(angle-cmds[numCmds-1]<threshWalk){
                cmd.linear.y = 3;
            }else{
                cmd.angular.z = -5;
            }
        }else{
            if(angle-cmds[numCmds-1] > -thresh){
                numCmds--;
                resetLastPos();
            }else if(angle-cmds[numCmds-1]>-threshWalk){
                cmd.linear.y =  -3;
            }else{
                cmd.angular.z = 5;
            } 
        }
        numCmds++;
        cmdTypes[numCmds-1] = 2340; 
    }else if(cmdTypes[numCmds-1] == 2){// move towards lego

        if(cmds[numCmds-1] > PI){
            cmds[numCmds-1] -= 2.0*PI;
        }else if(cmds[numCmds-1] < - PI){
            cmds[numCmds-1] += 2.0*PI;
        }
        if(legoA>0){
            if(legoA<thresh/2.0){

            }else{
                cmd.linear.y = -3;
            }
        }else{
            if(legoA > -thresh/2.0){

            }else{
                cmd.linear.y =  3;
            } 
        }
        if (legoY > 8){
          if(cmd.linear.y==0){
            cmd.linear.x = 5;
          }else{
            numCmds++;
            cmdTypes[numCmds-1] = 12784;
          }
        }else if(legoYL1>0 || legoYL2>0){
          numCmds--;
          resetLastPos();
          backup();
          goToZero();
        }else{
          numCmds--;
          resetLastPos();
        }  
    }else if(cmdTypes[numCmds-1] == 3){//rotate around to point at origin
      if (tmpBool==0){
          cmds[numCmds-1] = 3.0*PI/2.0 + atan2(yI,xI);
          tmpBool = 1;
        };
        if(cmds[numCmds-1] > PI){
            cmds[numCmds-1] -= 2.0*PI;
        }else if(cmds[numCmds-1] < - PI){
            cmds[numCmds-1] += 2.0*PI;
        }
        if(angle-cmds[numCmds-1]>0){
            if(angle-cmds[numCmds-1]<thresh){
                numCmds--;
                resetLastPos();
                tmpBool = 0;
            }else{
                cmd.linear.y = 3;
            }
        }else{
            if(angle-cmds[numCmds-1] > -thresh){
                numCmds--;
                resetLastPos();
                tmpBool = 0;
            }else{
                cmd.linear.y =  -3;
            } 
        }
    }else if(cmdTypes[numCmds-1] == 4){//rotate just a little
       cmd.angular.z =  -3;
       cmdTypes[numCmds-1] = 2340;
       numCmds++;
       cmdTypes[numCmds-1] = 2340; 
       resetLastPos();
    }else if (cmdTypes[numCmds-1] == 5){ //go distance to zero-zero
        if (tmpBool==0){
          cmds[numCmds-1] = sqrt(x[0][0]*x[0][0]+x[1][0]*x[1][0])/1.5;
          if (cmds[numCmds-1] > 50){
          cmds[numCmds-1] = 50.0;
          }
          tmpBool = 1;
        }
        if(cmds[numCmds-1]>=0){ //going forward
            if (dist>=cmds[numCmds-1]){ //if has gone far enough
                numCmds--;
                resetLastPos();
                tmpBool = 0;
            }else{ //otherwise keep going
                cmd.linear.x = 5;
            }
        }else{
           numCmds--;
           resetLastPos();
           tmpBool = 0;
        }
    }else{
         numCmds--;
    }

ROS_INFO("Bot cmd x=%f y=%f z=%f",cmd.linear.x,cmd.linear.y,cmd.angular.z);
    
    //send command 
    pub.publish(cmd);

    //allow other threads a chance and wait for next loop iteration
    ros::spinOnce();
    loop_rate.sleep();

  //If there are no commands to take, end
    if (numCmds < 0){
    done = 1;
    }

  }
  //#####################################################################################################}
