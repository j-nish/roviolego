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

using namespace std;

#define PI 3.14159265

// global cv variables
IplImage* dst =0;
IplImage* img =0;
IplImage* temp =0;

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

// for the blob code
struct coordinate {
	unsigned int x, y;
	void * data;
};

struct lineBlob {
	unsigned int min, max;
	unsigned int blobId;
	bool attached;
};

struct blob {
	//unsigned int blobId;
	coordinate min, max;
	coordinate center;
};

// function that does the actual blob detection
void detectBlobs(IplImage* frame, IplImage* finalFrame) {
	int blobCounter = 0;
	map<unsigned int, blob> blobs;

    unsigned char threshold = 235;

    vector< vector<lineBlob> > imgData(frame->width);
	for(int row = 0; row < frame->height; ++row) {
		for(int column = 0; column < frame->width; ++column) {
			//unsigned char byte = (unsigned char) imgStream.get();
			// this is the condition for being a blob pixel
			unsigned char byte = (unsigned char) frame->imageData[(row*frame->width)+ column];
			if(byte > threshold) {
				int start = column;
				for(;byte >= threshold; byte = (unsigned char) frame->imageData[(row*frame->width)+ column], ++column);
				int stop = column-1;
				lineBlob lineBlobData = {start, stop, blobCounter, false};
				imgData[row].push_back(lineBlobData);
				blobCounter++;
			}
		}
	}

	/* Check lineBlobs for a touching lineblob on the next row */
	for(int row = 0; row < (int) imgData.size(); ++row) {
		for(int entryLine1 = 0; entryLine1 < (int) imgData[row].size(); ++entryLine1) {
			for(int entryLine2 = 0; entryLine2 < (int) imgData[row+1].size(); ++entryLine2) {
				if(!((imgData[row][entryLine1].max < imgData[row+1][entryLine2].min) || (imgData[row][entryLine1].min > imgData[row+1][entryLine2].max))) {
					if(imgData[row+1][entryLine2].attached == false) {
						imgData[row+1][entryLine2].blobId = imgData[row][entryLine1].blobId;
						imgData[row+1][entryLine2].attached = true;
					}
					else {
						imgData[row][entryLine1].blobId = imgData[row+1][entryLine2].blobId;
						imgData[row][entryLine1].attached = true;
					}
				}
			}
		}
	}

	// Sort and group blobs
	for(int row = 0; row < (int) imgData.size(); ++row) {
		for(int entry = 0; entry < (int) imgData[row].size(); ++entry) {
			if(blobs.find(imgData[row][entry].blobId) == blobs.end()) // Blob does not exist yet
			{
				blob blobData = {{imgData[row][entry].min, row}, {imgData[row][entry].max, row}, {0,0}};
				blobs[imgData[row][entry].blobId] = blobData;
			}
			else {
				if(imgData[row][entry].min < blobs[imgData[row][entry].blobId].min.x)
					blobs[imgData[row][entry].blobId].min.x = imgData[row][entry].min;
				else if(imgData[row][entry].max > blobs[imgData[row][entry].blobId].max.x)
					blobs[imgData[row][entry].blobId].max.x = imgData[row][entry].max;
				if(row < (int) blobs[imgData[row][entry].blobId].min.y)
					blobs[imgData[row][entry].blobId].min.y = row;
				else if(row > (int) blobs[imgData[row][entry].blobId].max.y)
					blobs[imgData[row][entry].blobId].max.y = row;
			}
		}
	}

	// Calculate center
	for(map<unsigned int, blob>::iterator i = blobs.begin(); i != blobs.end(); ++i) {
		(*i).second.center.x = (*i).second.min.x + ((*i).second.max.x - (*i).second.min.x) / 2;
		(*i).second.center.y = (*i).second.min.y + ((*i).second.max.y - (*i).second.min.y) / 2;

		int size = ((*i).second.max.x - (*i).second.min.x) * ((*i).second.max.y - (*i).second.min.y);

		// Print coordinates on image, if it is large enough
		if(size > 200) {
			CvFont font;
			cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0, 1, CV_AA);
			char textBuffer[128];

			// Draw crosshair and print coordinates (just for debugging, not necessary for later multi-touch use)
			cvLine(finalFrame, cvPoint((*i).second.center.x - 5, (*i).second.center.y), cvPoint((*i).second.center.x + 5, (*i).second.center.y), cvScalar(0, 0, 153), 1);
			cvLine(finalFrame, cvPoint((*i).second.center.x, (*i).second.center.y - 5), cvPoint((*i).second.center.x, (*i).second.center.y + 5), cvScalar(0, 0, 153), 1);
			sprintf(textBuffer, "(%d, %d)", (*i).second.center.x, (*i).second.center.y);
			cvPutText(finalFrame, textBuffer, cvPoint((*i).second.center.x + 5, (*i).second.center.y - 5), &font, cvScalar(0, 0, 153));
			cvRectangle(finalFrame, cvPoint((*i).second.min.x, (*i).second.min.y), cvPoint((*i).second.max.x, (*i).second.max.y), cvScalar(0, 0, 153), 1);

			// Show center point
			//cout << "(" << (*i).second.center.x << ", " << (*i).second.center.y << ")" << endl;
		}
	}
}
// print out some properties of the image
void printImageInfo( IplImage* image ) {
	// print some properties
	printf("--------printing image info-------\n");
	//printf( "Filename:    %s\n",        argv[1] );
	printf( "# of channels:  %d\n",     image->nChannels );
	printf( "Pixel depth: %d bits\n",   image->depth );
	printf( "width:       %d pixels\n", image->width );
	printf( "height:      %d pixels\n", image->height );
	printf( "Image size:  %d bytes\n",  image->imageSize );
	printf( "Width step:  %d bytes\n",  image->widthStep );
	printf( "Depth:  %d \n",  			image->depth );
	printf("----------------------------------\n");
}

// array that hold lego position data
int* legoPos = (int *) malloc(sizeof(int) * 2);

// implementation of the pinhole camera model
void toGlobal( int xpixel, int ypixel) {
	// do math for finding the actual position
	int f = 600;			// focal length
	int yhorz = 220;		// y-horizon
	double hheight = 3.5;
	//int* answer;

	xpixel -= 320;
	ypixel -= yhorz;

	int y = f*hheight/ypixel; //adjust as needed
	int x = y*xpixel/f;
	//printf("DEBUG: x = %d, y = %d\n", x, y);
	legoPos[0] = x;
	legoPos[1] = y;
	
	//something wrong with the pointer
	//answer = &legoPos[0];
	
	//return answer;
}

void getLegoPosition(void) {
	int showwindows = 0;
	//hardcode the path to the file to be processed 
	//need the typecast to avoid compiler warning
	char* imagefile =  (char *) "/home/jn/svn4/tmp.jpg";
	//char* imagefile =  (char *) "/home/crazyjoe/Desktop/rovio/tmp.jpg";
	//char* imagefile = (char *) "CamImg8129.jpg";

	//prints out the first argument
	//printf("File to be input is: %s\n", argv[1]);
	printf("File to be input is: %s\n", imagefile);

	//takes in an image file from a hardcoded location
	//IplImage* img = cvLoadImage( "lena.jpg" );
	img = cvLoadImage( imagefile );
	
	//takes in a file from the command line
	//img = cvLoadImage( imagefile );
	
	//create two windows
	if (showwindows == 1) {
		cvNamedWindow( "image-in" );
		cvNamedWindow( "image after segmentation" );
		cvNamedWindow( "image-out" );
	}

	//Show the original image
	if (showwindows == 1) {
		cvShowImage("image-in", img);
	}

	//set the rectangle for cropping
	//int new_width = 	200;
	//int new_height = 	200;
	//CvRect rect = cvRect(2, 2, new_width, new_height);
	//crop the image
	//cvSetImageROI(img, rect);
	//IplImage* cropped = cvCreateImage( cvSize(new_width, new_height), IPL_DEPTH_8U, 3 );
	//copy cropped image into cropped
	//cvCopy(img, cropped, NULL);

	//print image info
	//printImageInfo( img );
	
	// Create an image for the output
	IplImage* img2 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
	IplImage* dst  = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
	IplImage* temp = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );

	//copy from img to img2
	//cvCopy(img, img2, NULL);
	
	//access image data
	//there are other ways to do this, but this seemed to be the simplest
	//it is also said to be computationally fastest
	//this double for loop changes all the pixel values to red
	//notice that there are 3 channels
	//also, keep in mind that HSV and RGB are two different ways of
	//representing color
	int x, y;
	for (y=0; y<img2->height; y++) {
		//compute the pointer directly as the head of the relavant row y
		uchar* ptr = (uchar*) (img2->imageData + y * img2->widthStep);
		for (x=0; x<img2->width; x++) {
			ptr[3*x+1] = 40;		//setting the "H"-hue, or yellow
			ptr[3*x+2] = 196;		//setting the "S"-saturation, or red
			ptr[3*x+3] = 27;		//setting the "V"-value, or blue
		}
	}
	//Perform a Gaussian blur
	//cvSmooth( img, out, CV_GAUSSIAN, 11, 11 );
	
	//subtract the original image from the
	cvAbsDiff(img, img2, img2);

	//threshold the dst image
	cvThreshold(img2, dst, 50.0, 255, CV_THRESH_BINARY);
	
	if (showwindows == 1) {
		cvShowImage("image after segmentation", dst);
	}

	// after segmenting, allow only the "all 3 channels at 255" to remain
	for (y=0; y<dst->height; y++) {
		// compute the pointer directly as the head of the relavant row y
		uchar* data = (uchar*) (dst->imageData + y * dst->widthStep);
		for (x=0; x<dst->width; x++) {
			// the "black" parts should be converted to 255 and the rest should be set to 0
			if (data[3*x+1]!=0 || data[3*x+2]!=0 || data[3*x+3]!=0) {
				data[3*x+1] = 0;	
				data[3*x+2] = 0;
				data[3*x+3] = 0;
			} else {
				data[3*x+1] = 255;	
				data[3*x+2] = 255;
				data[3*x+3] = 255;
			}

			if(y<dst->height/2) {
				data[3*x+1] = 0;	
				data[3*x+2] = 0;
				data[3*x+3] = 0;
			}
		}
	}

	// make NULL kernel
	//IplConvKernel* nullkernel = NULL;
	int iterations = 1;

	// make a convolution kernel
	IplConvKernel* kernopen = cvCreateStructuringElementEx(4,4,2,2,CV_SHAPE_ELLIPSE);
	IplConvKernel* kerndilate = cvCreateStructuringElementEx(2,2,1,1,CV_SHAPE_ELLIPSE);

	// perform dilation which takes the max
	cvDilate(dst, dst, kerndilate, iterations);

	// apply morphological opening
	cvMorphologyEx(dst, dst, temp, kernopen, CV_MOP_CLOSE);

	// convert to greyscale (1channel) destination must be 1 channel
	cvCvtColor(dst, temp, CV_RGB2GRAY);

	// find the mean location of all the pixels
	int sumx = 0;
	int sumy = 0;
	int counter = 0;
	// count up the number of pixels and their x-y pixel coordinates
	for (y=0; y<dst->height; y++) {
		// compute the pointer directly as the head of the relavant row y
		uchar* temp = (uchar*) (dst->imageData + y * dst->widthStep);
		for (x=0; x<dst->width; x++) {
			if (temp[3*x+1] == 255) {
				//printf("DEBUG: x=%d, y=%d\n", x,y);
				sumx += x;
				sumy += y;
				counter++;
			}
		}
	}
	// debug
	printf("DEBUG: sumx = %d, sumy = %d, counter = %d\n", sumx, sumy, counter);
	// to prevent division by zero
	if (counter == 0) {
		counter = 1;
	}
	double averagex = (double) sumx / (double) counter;
	double averagey = (double) sumy / (double) counter;
	printf("DEBUG: averagex= %f averagey= %f\n", averagex, averagey);

	// use function to return pointer to array of positions
	//int* foo = toGlobal( (int) averagex, (int) averagey);
	//printf("DEBUG: function return is: %d and %d \n", foo[0], foo[1]);
	

	// save the output image to a file
	cvSaveImage("outputcv.jpg", temp);

	// print final image stats
	//printImageInfo( temp );

	// Show the processed image
	if (showwindows == 1) {
		cvShowImage("image-out", temp);
	}
	//#############################################################
	// for capturing for a webcam
	//CvCapture * capture = cvCaptureFromCAM(CV_CAP_ANY);
	//if(!capture) {
		//fprintf( stderr, "ERROR: capture is NULL \n" );
		//getchar();
	//}

	// Create a window in which the captured images will be presented
	//cvNamedWindow( "Capture", CV_WINDOW_AUTOSIZE );
	if (showwindows == 1) {
		cvNamedWindow("Capture");
		cvNamedWindow("Result");
	}


	// Get one frame from the web cam
	//IplImage* frame = cvQueryFrame(capture);
	IplImage* frame = cvLoadImage( imagefile );
	if(!frame) {
		fprintf( stderr, "ERROR: frame is null...\n" );
		getchar();
	}

	IplImage* gsFrame;
	IplImage* finalFrame;
	gsFrame = cvCreateImage(cvSize(frame->width,frame->height), IPL_DEPTH_8U, 1);
	finalFrame = cvCloneImage(frame);

	// Convert image to grayscale
	cvCvtColor(frame, gsFrame, CV_BGR2GRAY);

	// Blur the images to reduce the false positives
	cvSmooth(gsFrame, gsFrame, CV_BLUR);
	cvSmooth(finalFrame, finalFrame, CV_BLUR);

	// Detection (with timer for debugging purposes)
	clock_t start = clock();
	detectBlobs(gsFrame, finalFrame);
	clock_t end = clock();
	cout << end-start << endl;

	// Show images in a nice window
	if (showwindows == 1) {
		cvShowImage( "Capture", frame );
		cvShowImage( "Result", finalFrame );
	}

	//wait for a key to be pressed
	if (showwindows == 1) {
		cvWaitKey(0);
	}

	//if (showwindows == 1) {
		//cvDestroyWindow( "Capture" );
		//cvDestroyWindow( "Result" );
	//}

		//#######################end blobs

	//release the memory
	cvReleaseImage( &img );
	cvReleaseImage( &dst );
	cvReleaseImage( &temp );
	cvReleaseImage( &gsFrame);
	cvReleaseImage( &finalFrame);
	//cvReleaseCapture( &capture );
	//close windows
	//cvDestroyWindow( "image-in" );
	//cvDestroyWindow( "image-out" );
}



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
			cmd.linear.x=0;
		}else{
			cmd.linear.x=0;
		}
		//get image from ros
		//IplImage* imgMsgToCv(sensor_msgs::Image::ConstPtr image_message, string cv_encoding="passthrough");
		getLegoPosition();
		printf("DEBUG: legoPos[0] = %d, legoPos[1] = %d\n", legoPos[0], legoPos[1]);

		//send command
		pub.publish(cmd);
		dist+=cmd.linear.x/2.0;

		//allow other threads a chance and wait for next loop iteration
		ros::spinOnce();
		loop_rate.sleep();

	}
	//#####################################################################################################
}
