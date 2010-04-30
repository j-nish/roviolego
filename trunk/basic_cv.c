// basic_cv <image>
//this program takes in an image img, then does background subtraction
//with a red background, then saves and outputs the image
#ifdef _CH_
#pragma package <opencv>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>

using namespace cv;

//using namespace cv;
//using namespace std;
void trackbarHandler(int pos);

int threshVal= 50.0;
int maxVal= 255;
int pos = cvGetTrackbarPos("bar1", "image-out");

IplImage* dst =0;
IplImage* img =0;
IplImage* temp =0;

int main(int argc, char *argv[]) {
	//prints out the first argument
	printf("File to be input is: %s\n", argv[1]);

	//takes in an image file from a hardcoded location
	//IplImage* img = cvLoadImage( "lena.jpg" );
	
	//takes in a file from the command line
	img = cvLoadImage( argv[1] );
	
	//create two windows
	cvNamedWindow( "image-in" );
	cvNamedWindow( "image" );
	cvNamedWindow( "image-out" );

	//Show the original image
	cvShowImage("image-in", img);
	
	CvRect rect = cvRect(2, 2, 200, 200);

	//crop the image
	//cvSetImageROI(img, rect);

	//retrieve properties about the image that was just loaded
	int width 		= img->width;
	int height		= img->height;
	int nchannels	= img->nChannels;
	int step		= img->widthStep;

	//print some properties
	//see struct information for IplImage* for more info
	printf( "Filename:    %s\n",        argv[1] );
	printf( "# channels:  %d\n",        img->nChannels );
	printf( "Pixel depth: %d bits\n",   img->depth );
	printf( "width:       %d pixels\n", img->width );
	printf( "height:      %d pixels\n", img->height );
	printf( "Image size:  %d bytes\n",  img->imageSize );
	printf( "Width step:  %d bytes\n",  img->widthStep );
	printf( "Depth:  %d \n",  			img->depth );

	// Create an image for the output
	IplImage* img2 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
	IplImage* dst = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
	IplImage* dst2 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );

	//copy from img to img2
	cvCopy(img, img2, NULL);
	
	//access image data
	//there are other ways to do this, but this seemed to be the simplest
	//it is also said to be computationally fastest
	//this double for loop changes all the pixel values to red
	//notice that there are 3 channels
	//also, keep in mind that HSV and RGB are two different ways of
	//representing color
	int x, y;
	for (y=0; y<height; y++) {
		//compute the pointer directly as the head of the relavant row y
		uchar* ptr = (uchar*) (img2->imageData + y * img2->widthStep);
		for (x=0; x<width; x++) {
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
	
	cvShowImage("image", dst);

	//after segmenting, allow only the "all 3 channels at 255" to remain
	for (y=0; y<height; y++) {
		//compute the pointer directly as the head of the relavant row y
		uchar* data = (uchar*) (dst->imageData + y * dst->widthStep);
		for (x=0; x<width; x++) {
			if (data[3*x+1]!=0 || data[3*x+2]!=0 || data[3*x+3]!=0) {
				data[3*x+1] = 255;	
				data[3*x+2] = 255;
				data[3*x+3] = 255;
			}
		}
	}

	//convert to greyscale (1channel)
	//cvCvtColor(dst, dst2, CV_RGB2GRAY);

	//save the output image to a file
	cvSaveImage("tmp.jpeg", dst);

	//Show the processed image
	cvShowImage("image-out", dst);

	//wait for a key to be pressed
	cvWaitKey(0);
	//release the memory
	cvReleaseImage( &img );
	cvReleaseImage( &dst );
	//close windows
	cvDestroyWindow( "image-in" );
	cvDestroyWindow( "image-out" );
	return 0;
}

//define the trackbar handler
void trackbarHandler(int pos) {
	//create the image
	IplImage* img2 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
	threshVal = cvGetTrackbarPos("bar1", "image-out");
	//threshVal = pos;
	//printf("Trackbar position: %d\n", threshVal);
	//threshold the dst image
	cvThreshold(temp, img2, threshVal, 255, CV_THRESH_BINARY);
	//Show the processed image
	cvShowImage("image-out", img2);
	cvReleaseImage(&img2);
}
