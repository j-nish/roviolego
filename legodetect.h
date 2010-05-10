// header file for lego detection opencv program.
int divide(int a, int b) {
	return a/b;
}

#ifdef _CH_
#pragma package <opencv>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>

//using namespace cv;

IplImage* dst =0;
IplImage* img =0;
IplImage* temp =0;

//need to prototype this in a header file
void printImageInfo( IplImage* image);
int* toGlobal( int xpixel, int ypixel);

void getLegoPosition(void) {
	//hardcode the path to the file to be processed 
	//need the typecast to avoid warning
	char* imagefile = (char *) "CamImg8129.jpg";

	//prints out the first argument
	//printf("File to be input is: %s\n", argv[1]);
	printf("File to be input is: %s\n", imagefile);

	//takes in an image file from a hardcoded location
	//IplImage* img = cvLoadImage( "lena.jpg" );
	
	//takes in a file from the command line
	img = cvLoadImage( imagefile );
	
	//create two windows
	//cvNamedWindow( "image-in" );
	//cvNamedWindow( "image" );
	//cvNamedWindow( "image-out" );

	//Show the original image
	//cvShowImage("image-in", img);

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
	printImageInfo( img );
	
	// Create an image for the output
	IplImage* img2 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
	IplImage* dst  = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
	IplImage* temp = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );

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
	

	//after segmenting, allow only the "all 3 channels at 255" to remain
	for (y=0; y<dst->height; y++) {
		//compute the pointer directly as the head of the relavant row y
		uchar* data = (uchar*) (dst->imageData + y * dst->widthStep);
		for (x=0; x<dst->width; x++) {
			if (data[3*x+1]!=0 || data[3*x+2]!=0 || data[3*x+3]!=0) {
				data[3*x+1] = 255;	
				data[3*x+2] = 255;
				data[3*x+3] = 255;
			}

			if(y<dst->height/2) {
				data[3*x+1] = 255;	
				data[3*x+2] = 255;
				data[3*x+3] = 255;
			}
		}
	}

	//make a convolution kernel
	IplConvKernel* kern = cvCreateStructuringElementEx(6,6,3,3,CV_SHAPE_ELLIPSE);
	//cvShowImage("image", dst);

	//apply morphological opening
	cvMorphologyEx(dst, dst, temp, kern, CV_MOP_CLOSE);


	//convert to greyscale (1channel) destination must be 1 channel
	cvCvtColor(dst, temp, CV_RGB2GRAY);

	//find the mean location of all the pixels
	int sumx = 0;
	int sumy = 0;
	int counter = 0;
	for (y=0; y<dst->height; y++) {
		//compute the pointer directly as the head of the relavant row y
		uchar* temp = (uchar*) (dst->imageData + y * dst->widthStep);
		for (x=0; x<dst->width; x++) {
			if (temp[3*x+1]==0) {
				temp[3*x+1] = 255;	
				//printf("DEBUG x=%d, y=%d\n", x,y);
				sumx += x;
			sumy += y;
				counter++;
			}
		}
	}
	//debug
	printf("DEBUG sumx = %d, sumy = %d\n", sumx, sumy);
	printf("DEBUG counter = %d\n", counter);
	int averagex = divide(sumx, counter);
	int averagey = divide(sumy, counter);
	printf("DEBUG averagex= %d averagey= %d\n", averagex, averagey);

	//use function to return array of positions
	int* foo = toGlobal( averagex, averagey);
	printf("DEBUG function return is: %d and %d \n", foo[0], foo[1]);
	
	//save the output image to a file
	cvSaveImage("tmp.jpeg", temp);

	printImageInfo( temp );

	//Show the processed image
	//cvShowImage("image-out", temp);

	//wait for a key to be pressed
	//cvWaitKey(0);

	//release the memory
	cvReleaseImage( &img );
	cvReleaseImage( &dst );
	//close windows
	//cvDestroyWindow( "image-in" );
	//cvDestroyWindow( "image-out" );
}

void printImageInfo( IplImage* image ) {
	//retrieve properties about the image that was just loaded
	int width 		= image->width;
	int height		= image->height;
	int nchannels	= image->nChannels;
	int step		= image->widthStep;

	//print some properties
	//see struct information for IplImage* for more info
	
	printf("--------printing image info-------\n");
	//printf( "Filename:    %s\n",        argv[1] );
	printf( "# of channels:  %d\n",     image->nChannels );
	printf( "Pixel depth: %d bits\n",   image->depth );
	printf( "width:       %d pixels\n", image->width );
	printf( "height:      %d pixels\n", image->height );
	printf( "Image size:  %d bytes\n",  image->imageSize );
	printf( "Width step:  %d bytes\n",  image->widthStep );
	printf( "Depth:  %d \n",  			image->depth );
}


int* toGlobal( int xpixel, int ypixel) {
	//do math for finding the actual position
	int answerarray[2];
	int f = 700;
	int yhorz = 225;
	double hheight = 3.5;
	int* answer;

	xpixel -= 320;
	ypixel -= yhorz;

	int y = f*hheight/ypixel;
	int x = y*xpixel/f;
	answerarray[0] = x;
	answerarray[1] = y;

	answer = &answerarray[0];
	
	return answer;
}

