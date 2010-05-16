# makefile for the opencv programs.
#$(CC) $(FLAGS) basic_cv.c -o basic_cv.o $(ARCH)
CC = g++
FLAGS = -I /usr/local/include/opencv -lm -lcv -lhighgui -lcvaux -lcxcore -Wall

UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
	ARCH = -arch i386
endif

build all:
	$(CC) $(FLAGS) temp.c -o temp.o $(ARCH)

original-blobs:
	$(CC) $(FLAGS) original-blobs.cpp -o original-blobs.o $(ARCH)

blobs:
	$(CC) $(FLAGS) blobs.cpp -o blobs.o $(ARCH)

clean:
	rm *.o
