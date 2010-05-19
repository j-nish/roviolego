// header file for lego detection opencv program.
#ifdef _CH_
#pragma package <opencv>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cv.h>			// for cv
#include <highgui.h>
#include <iostream>		// for blob code
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <ctime>

using namespace std;

// hardcode the path to the file to be processed 
// need the typecast to avoid compiler warning
//char *imagefile =  (char *) "/home/crazyjoe/Desktop/rovio/tmp.jpg";
char *imagefile =  (char *) "/home/jn/svn4/tmp.jpg";
//char* imagefile = (char *) "CamImg8129.jpg";

// the global lego position array
double *legoPos = (double *) malloc(sizeof(double) * 2);
int pixelx;
int pixely;
double noise=0;

// arrays for colors (BRG)
int red[3] 			= {40,196,27};		// works
int green[3] 		= {70,40,127};		// works
int yellow[3] 		= {205,245,47}; 	// works
int blue[3] 		= {180,20,50};		// kinda works
int orange[3] 		= {155,226,63};		// works
int white[3] 		= {255,255,255};		// works
int black[3] 		= {0,0,0};		// works
int lightblue[3] 	= {245,51,126};		// works
int brown[3] 		= {3,62,36};		// ne

// some debugging flags
int showwindows = 1;
int debug = 1;
int debug2 = 1;
int debugtime = 0;
int debugnoise = 0;
int debugimage = 0;

// global cv variables
IplImage* dst = 0;
IplImage* img = 0;
IplImage* img2 = 0;
IplImage* temp = 0;
IplImage* frame = 0;
IplImage* gsFrame = 0;
IplImage* finalFrame = 0;

// global structs for the blob code
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

	// status flag
	int blobexists = 0;

    unsigned char threshold = 235;

    vector< vector<lineBlob> > imgData(frame->width);
	for(int row = 0; row < frame->height; ++row) {
		for(int column = 0; column < frame->width; ++column) {
			//unsigned char byte = (unsigned char) imgStream.get();
			unsigned char byte = (unsigned char) frame->imageData[(row*frame->width)+ column];
			// this is the condition for being a blob pixel
			if(byte > threshold) {
			//printf("DEBUG detectBlobs: unsigned char byte is: %d\n", byte);
				int start = column;
				for(;byte >= threshold; byte = (unsigned char) frame->imageData[(row*frame->width)+ column], ++column);
				int stop = column-1;
				lineBlob lineBlobData = {start, stop, blobCounter, false};
				imgData[row].push_back(lineBlobData);
				blobCounter++;
			}
		}
	}

	// Check lineBlobs for a touching lineblob on the next row
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
				blob blobData = {{imgData[row][entry].min, row, 0}, {imgData[row][entry].max, row, 0}, {0,0,0}};
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
		if (debug) printf("DEBUG detectBlobs: blob found. size is: %d\n", size);

		// Print coordinates on image, if it is large enough
		if(size > 30 && size < 2500) {
			// set flag, since if this statement is entered, then we have a blob
			blobexists = 1;
			CvFont font;
			cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0, 1, CV_AA);
			char textBuffer[128];

			// Draw crosshair and print coordinates (just for debugging, not necessary for later multi-touch use)
			cvLine(finalFrame, cvPoint((*i).second.center.x - 5, (*i).second.center.y), cvPoint((*i).second.center.x + 5, (*i).second.center.y), cvScalar(0, 0, 153), 1);
			cvLine(finalFrame, cvPoint((*i).second.center.x, (*i).second.center.y - 5), cvPoint((*i).second.center.x, (*i).second.center.y + 5), cvScalar(0, 0, 153), 1);
			sprintf(textBuffer, "(%d, %d)", (*i).second.center.x, (*i).second.center.y);
			// the cvScalar values denote BGR values
			cvPutText(finalFrame, textBuffer, cvPoint((*i).second.center.x + 5, (*i).second.center.y - 5), &font, cvScalar(0, 0, 153));
			cvRectangle(finalFrame, cvPoint((*i).second.min.x, (*i).second.min.y), cvPoint((*i).second.max.x, (*i).second.max.y), cvScalar(0, 0, 153), 1);

			// pixelx and pixely are globals
			pixelx = (*i).second.center.x;
			pixely = (*i).second.center.y;
			// Show center point
			if (debug) printf("DEBUG detectBlobs: Lego blob found. pixel location is: (%d, %d)\n", pixelx, pixely);

			//printf("DEBUG BLOB: legoPos[0] = %5.2f, legoPos[1] = %5.2f\n", legoPos[0], legoPos[1]);

		}
		//if (debug) cout << "DEBUG detectBlob: (" << (*i).second.center.x << ", " << (*i).second.center.y << ")" << endl;
	}
	// if blob doesn't exist
	if (blobexists == 0) {
		pixelx = -77;
		pixely = -77;
	}
}

// returns void, but uses legoPos array
// do math for finding the actual position
void toGlobal( int xpixel, int ypixel) {
	if (debug) printf("DEBUG toGlobal: input x is %d, input y is = %d\n", xpixel, ypixel);

	int f = 600;
	int yhorz = 220;
	double hheight = 3.5;
	double x, y;

	xpixel -= 320;
	ypixel -= yhorz;

	y = (double) f*hheight/ (double) ypixel + noise; //adjust as needed
	x = (double) y*xpixel/ (double) f;

	if (debug) printf("DEBUG toGlobal: x = %f, y = %f\n", x, y);
	
	legoPos[0] = x;
	legoPos[1] = y;
}


// simple print info function
void printImageInfo( IplImage* image ) {
	printf("--------printing image info-------\n");
	printf( "# of channels:  %d\n",     image->nChannels );
	printf( "Pixel depth: %d bits\n",   image->depth );
	printf( "width:       %d pixels\n", image->width );
	printf( "height:      %d pixels\n", image->height );
	printf( "Image size:  %d bytes\n",  image->imageSize );
	printf( "Width step:  %d bytes\n",  image->widthStep );
	printf( "Depth:  %d \n",  			image->depth );
	printf("----------------------------------\n");
} 

// start "main" program
void getLegoPosition(void) {
	// prints out the first argument
	if (debug) printf("File to be input is: %s\n", imagefile);

	while (1) {
		img = cvLoadImage( imagefile );
		if (!img) {
			printf("Uh oh, bad image!\n");
			sleep(1);
		} else {
			break;
		}
	}

	// set the noise value
	uchar* ptr = (uchar*) (img->imageData + 100 * img->widthStep);
	noise = (double) ptr[3*100+1] / 255.0;
	
	// create three windows
	if (showwindows) {
		cvNamedWindow( "Step 1 Original image" );
		cvNamedWindow( "Step 2 Background subtraction and thresholding" );
		cvNamedWindow( "Step 3 binary image" );
	}

	// Show the original image
	if (showwindows) cvShowImage("Step 1 Original image", img);

	// set the rectangle for cropping
	//int new_width = 	200;
	//int new_height = 	200;
	//CvRect rect = cvRect(2, 2, new_width, new_height);
	//crop the image
	//cvSetImageROI(img, rect);
	//IplImage* cropped = cvCreateImage( cvSize(new_width, new_height), IPL_DEPTH_8U, 3 );
	//copy cropped image into cropped
	//cvCopy(img, cropped, NULL);

	// print image info
	if (debugimage) printImageInfo( img );

	//copy from img to img2
	//cvCopy(img, img2, NULL);
	
	// access image data
	// there are other ways to do this, but this seemed to be the simplest
	// it is also said to be computationally fastest
	// this double for loop changes all the pixel values to red
	// notice that there are 3 channels
	// also, keep in mind that HSV and RGB are two different ways of
	// representing color
	// Create an image for the output
	img2 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
	dst  = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
	temp = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );

	int x, y;
	int thresh = 50;
	// PHASE: background subtraction and threshold input (img) output (dst)
	for (y=0; y<img2->height; y++) {
		// compute the pointer directly as the head of the relavant row y
		uchar* ptrimg = (uchar*) (img->imageData + y * img->widthStep);
		uchar* ptrdst = (uchar*) (dst->imageData + y * dst->widthStep);
		for (x=0; x<img2->width; x++) {
			// if the absolute diff between img and im2 is greater than 50 set dst to 255
			// else, set dst to 0
			if ( fabs( ptrimg[3*x+1]-red[0]) < thresh) ptrdst[3*x+1] = 0;
			else { ptrdst[3*x+1] = 255; }
			if ( fabs( ptrimg[3*x+2]-red[1]) < thresh) ptrdst[3*x+2] = 0;
			else { ptrdst[3*x+2] = 255; }
			if ( fabs( ptrimg[3*x+3]-red[2]) < thresh) ptrdst[3*x+3] = 0;
			else { ptrdst[3*x+3] = 255; }
			
			// now yellow
			if ( fabs( ptrimg[3*x+1]-yellow[0]) < thresh) ptrdst[3*x+1] = 0;
			if ( fabs( ptrimg[3*x+2]-yellow[1]) < thresh) ptrdst[3*x+2] = 0;
			if ( fabs( ptrimg[3*x+3]-yellow[2]) < thresh) ptrdst[3*x+3] = 0;

			// now orange
			if ( fabs( ptrimg[3*x+1]-orange[0]) < 30) ptrdst[3*x+1] = 0;
			if ( fabs( ptrimg[3*x+2]-orange[1]) < 30) ptrdst[3*x+2] = 0;
			if ( fabs( ptrimg[3*x+3]-orange[2]) < 30) ptrdst[3*x+3] = 0;

			// now blue
			if ( fabs( ptrimg[3*x+1]-blue[0]) < 20) ptrdst[3*x+1] = 0;
			if ( fabs( ptrimg[3*x+2]-blue[1]) < 20) ptrdst[3*x+2] = 0;
			if ( fabs( ptrimg[3*x+3]-blue[2]) < 20) ptrdst[3*x+3] = 0;

			// now green
			//if ( fabs( ptrimg[3*x+1]-green[0]) < 10) ptrdst[3*x+1] = 0;
			//if ( fabs( ptrimg[3*x+2]-green[1]) < 10) ptrdst[3*x+2] = 0;
			//if ( fabs( ptrimg[3*x+3]-green[2]) < 10) ptrdst[3*x+3] = 0;
			
			// now white
			if ( fabs( ptrimg[3*x+1]-white[0]) < 20) ptrdst[3*x+1] = 0;
			if ( fabs( ptrimg[3*x+2]-white[1]) < 20) ptrdst[3*x+2] = 0;
			if ( fabs( ptrimg[3*x+3]-white[2]) < 20) ptrdst[3*x+3] = 0;

			// now lightblue
			//if ( fabs( ptrimg[3*x+1]-lightblue[0]) < 20) ptrdst[3*x+1] = 0;
			//if ( fabs( ptrimg[3*x+2]-lightblue[1]) < 20) ptrdst[3*x+2] = 0;
			//if ( fabs( ptrimg[3*x+3]-lightblue[2]) < 20) ptrdst[3*x+3] = 0;
			
			// now brown
			//if ( fabs( ptrimg[3*x+1]-brown[0]) < 20) ptrdst[3*x+1] = 0;
			//if ( fabs( ptrimg[3*x+2]-brown[1]) < 20) ptrdst[3*x+2] = 0;
			//if ( fabs( ptrimg[3*x+3]-brown[2]) < 20) ptrdst[3*x+3] = 0;

			//setting the "H"-hue, or yellow
			//setting the "S"-saturation, or red
			//setting the "V"-value, or blue
		}
	}
	// Perform a Gaussian blur
	//cvSmooth( img, out, CV_GAUSSIAN, 11, 11 );
	
	// subtract the original image from the
	//cvAbsDiff(img, img2, img2);

	// threshold img2 and put in dst
	//cvThreshold(img2, dst, 50.0, 255, CV_THRESH_BINARY);
	
	if (showwindows == 1) cvShowImage("Step 2 Background subtraction and thresholding", dst);

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
			// set top half of the image to zero
			if(y<dst->height/2) {
				data[3*x+1] = 0;	
				data[3*x+2] = 0;
				data[3*x+3] = 0;
			}
		}
	}

	// make a convolution kernel
	IplConvKernel* kernopen = cvCreateStructuringElementEx(10,10,4,4,CV_SHAPE_ELLIPSE);

	// perform dilation which takes the max
	//cvDilate(dst, dst, kerndilate, 1);

	// apply morphological opening
	cvMorphologyEx(dst, dst, temp, kernopen, CV_MOP_CLOSE);
	//cvMorphologyEx(dst, dst, temp, kernopen, CV_MOP_CLOSE);

	// convert to greyscale (1channel) destination must be 1 channel
	cvCvtColor(dst, temp, CV_RGB2GRAY);

	// find the mean location of all the pixels
	//int sumx = 0;
	//int sumy = 0;
	//int counter = 0;
	// count up the number of pixels and their x-y pixel coordinates
	//for (y=0; y<dst->height; y++) {
		// compute the pointer directly as the head of the relevant row y
		//uchar* temp = (uchar*) (dst->imageData + y * dst->widthStep);
		//for (x=0; x<dst->width; x++) {
			//if (temp[3*x+1] == 255) {
				//temp[3*x+1] = 255;	
				//printf("DEBUG: x=%d, y=%d\n", x,y);
				//sumx += x;
				//sumy += y;
				//counter++;
			//}
		//}
	//}
	//if (debug) printf("DEBUG: sumx = %d, sumy = %d, counter = %d\n", sumx, sumy, counter);
	//to prevent division by zero
	//if (counter == 0) {
		//counter = 1;
	//}
	//double averagex = (double) sumx / (double) counter;
	//double averagey = (double) sumy / (double) counter;
	//if (debug) printf("DEBUG: averagex= %f averagey= %f\n", averagex, averagey);

	//use function to return pointer to array of positions
	//int *foo = toGlobal( (int) averagex, (int) averagey);
	
	// save the output image to a file
	cvSaveImage("outputcv.jpg", temp);

	// print final image stats
	if (debugimage) printImageInfo( temp );

	//Show the processed image
	if (showwindows) cvShowImage("Step 3 binary image", temp);
	
	//################################begin blob detection#######################
	if (showwindows) cvNamedWindow("Step 4 Blob detection result");


	// gsFrame and finalFrame are globals defined at the top
	// gsFrame is a greyscale image that will be the input for detectBlobs()
	gsFrame = cvCreateImage(cvSize(temp->width,temp->height), IPL_DEPTH_8U, 1);
	cvCopy(temp, gsFrame, NULL);

	//printf("DEBUG about to print!!!\n");
	//printImageInfo(frame);
	//printImageInfo(temp);

	// if you want the finalFrame to the input for the blob detection phase
	finalFrame = cvCloneImage(img);

	// Detection (with timer for debugging purposes)
	clock_t start = clock();
	detectBlobs(gsFrame, finalFrame);
	clock_t end = clock();
	if (debugtime) cout << "DEBUG BLOB: Time taken: " << end-start << endl;
	
	// this is where legoPos is set
	toGlobal( pixelx, pixely);
	// add noise to pixely noise is a global btw
	if (debugnoise) printf("DEBUG main: noise is: %f\n", noise);
	legoPos[1] += noise;

	if (debug) printf("DEBUG: function return from toGlobal is: %f and %f \n", legoPos[0], legoPos[1]);

	// Show images in a nice window
	if (showwindows) cvShowImage( "Step 4 Blob detection result", finalFrame );
	//#######################end blobs##########################################
	
	// wait for a key to be pressed
	if (showwindows) cvWaitKey(0);

	// release the memory
	cvReleaseImage( &img );
	cvReleaseImage( &img2 );
	cvReleaseImage( &dst );
	cvReleaseImage( &temp );
	cvReleaseImage( &frame );
	cvReleaseImage( &gsFrame );
	cvReleaseImage( &finalFrame );
	
	// destroy windows here
	//if (showwindows == 1) {
		//cvDestroyWindow( "Capture" );
		//cvDestroyWindow( "Result" );
		//cvDestroyWindow( "image-in" );
		//cvDestroyWindow( "image-out" );
	//}
	
	//printf("DEBUG: WHERE IS THE SEGFAULT?!\n");
}
