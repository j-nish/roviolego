# makefile for the opencv programs.
CC = g++
FLAGS = -I /usr/local/include/opencv -lm -lcv -lhighgui -lcvaux -lcxcore

UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
	ARCH = -arch i386
endif

build all:
	$(CC) $(FLAGS) basic_cv.c -o basic_cv.o $(ARCH)

clean:
	rm *.o
