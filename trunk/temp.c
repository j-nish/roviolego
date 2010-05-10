//main program
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "legodetect.h"


using namespace cv;

int main(int argc, char *argv[]) {
	getLegoPosition();
	//printf("DEBUG answer is: %d\n", divide (6,3));

	return 0;
}
	/*function[]x,y] = toglobal(xpix,ypix)
		f=700
		yhorz = 225
		hieght 3,5

		xpixel = xpixel -320
		ypixel = yppixel - yhorz

		y= f*height/ypixel
		x = y* xpixel/f

		end
		*/
