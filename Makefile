#/bin/bash

CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if 
LIB_LINK=-lhighgui -lcv -lcxcore -lm -lcxcore

all: data_collector simulator RobotPose

simualtor: simulator.c
	gcc ${CFLAGS} -c simulator.c
	gcc ${CFLAGS} -o simulator simulator.o ${LIB_FLAGS} ${LIB_LINK}

data_collector: data_collector.c
	gcc ${CFLAGS} -c data_collector.c
	gcc ${CFLAGS} -o data_collector data_collector.o ${LIB_FLAGS} ${LIB_LINK}

RobotPose: RobotPose.cpp
	gcc ${CFLAGS} -c RobotPose.cpp
	gcc ${CFLAGS} -o RobotPose RobotPose.o ${LIB_FLAGS} ${LIB_LINK}

clean:
	rm -rf *.o
	rm -rf data_collector
	rm -rf simulator
	rm -rf RobotPose
