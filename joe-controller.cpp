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

//#include "/home/crazyjoe/Desktop/rovio/legodetect.h"
//int *legoPos;

#define PI 3.14159265

// global cv variables
IplImage* dst =0;
IplImage* img =0;
IplImage* temp =0;

//int* legoPos;

//robot's angular orientation
double w=0;
//robot's x position
double Vx=0;
//robot's y position
double Vy=0;

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
double resultRow[3][1],tmpRow[3][1],tmpRow1[3][1],tmpRow2[3][1],xP[3][1],wA,legoX,legoY,legoA,legoXL,legoYL;
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
	//char* imagefile =  (char *) "/home/jn/svn4/tmp.jpg";
	char* imagefile =  (char *) "/home/crazyjoe/Desktop/rovio/tmp.jpg";
	//char* imagefile = (char *) "CamImg8129.jpg";

	//prints out the first argument
	//printf("File to be input is: %s\n", argv[1]);
	//printf("File to be input is: %s\n", imagefile);

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
	//printf("DEBUG: sumx = %d, sumy = %d, counter = %d\n", sumx, sumy, counter);
	// to prevent division by zero
	if (counter == 0) {
		counter = 1;
	}
	double averagex = (double) sumx / (double) counter;
	double averagey = (double) sumy / (double) counter;
	//printf("DEBUG: averagex= %f averagey= %f\n", averagex, averagey);

	// use function to return pointer to array of positions
	//int* foo = toGlobal( (int) averagex, (int) averagey);
	toGlobal( (int) averagex, (int) averagey);
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
    xP[0][0] = x[0][0];
    xP[1][0] = x[1][0];
    xP[2][0] = x[2][0];
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
  cmdTypes[numCmds-2] = 0;
  cmds[numCmds-2]     = sqrt(x[0][0]*x[0][0]+x[1][0]*x[1][0])/3.0;
  cmdTypes[numCmds-1] = 3;
  cmds[numCmds-1]     = 3.0*PI/2.0 + atan2(x[1][0],x[0][0]);
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
    dist=  sqrt((x[0][0]-xP[0][0])*(x[0][0]-xP[0][0])+(x[1][0]-xP[1][0])*(x[1][0]-xP[1][0]));
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
      backup();
      moveToLego();
    } 

    //If there are no commands to take, do image processing and call high level planning
    if (numCmds <= 0 || cmdTypes[numCmds-1] == 20000){
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
        //double legoXG = x[0][0]+legoX;
        //double legoYG = x[1][0]+legoY;
        //double toZero = sqrt(legoXG*legoXG+legoYG*legoYG);

        if(cmds[numCmds-1] > PI){
            cmds[numCmds-1] -= 2.0*PI;
        }else if(cmds[numCmds-1] < - PI){
            cmds[numCmds-1] += 2.0*PI;
        }
        if(legoA>0){
            if(legoA<thresh/2.0){

            }else{
                cmd.angular.z = 5;
            }
        }else{
            if(legoA > -thresh/2.0){

            }else{
                cmd.linear.y =  3;
            } 
        }
        if (legoY > 10 && cmd.angular.z == 0){
          cmd.linear.x = 5;
        }else if(legoA<thresh&&legoA>-thresh){
          numCmds--;
          resetLastPos();
          goToZero();
        }  
         numCmds++;
         cmdTypes[numCmds-1] = 12784;
    }else if(cmdTypes[numCmds-1] == 3){//rotate around to a certain angle
        if(cmds[numCmds-1] > PI){
            cmds[numCmds-1] -= 2.0*PI;
        }else if(cmds[numCmds-1] < - PI){
            cmds[numCmds-1] += 2.0*PI;
        }
        if(angle-cmds[numCmds-1]>0){
            if(angle-cmds[numCmds-1]<thresh){
                numCmds--;
                resetLastPos();
            }else{
                cmd.linear.y = 3;
            }
        }else{
            if(angle-cmds[numCmds-1] > -thresh){
                numCmds--;
                resetLastPos();
            }else{
                cmd.linear.y =  -3;
            } 
        }
        numCmds++;
        cmdTypes[numCmds-1] = 2340; 
    }else if(cmdTypes[numCmds-1] == 4){//rotate just a little
       cmd.angular.z =  -3;
       cmdTypes[numCmds-1] = 2340;
       numCmds++;
       cmdTypes[numCmds-1] = 2340; 
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
