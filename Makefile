#/bin/bash

CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if 
CPP_LIB_FLAGS=${LIB_FLAGS} -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore -lm 
LIB_KALMAN = ${LIB_LINK} -lgslcblas -L/usr/lib64/atlas/ -lclapack

all: data_collector simulator PIDController RobotPose base_navigator 

data_collector: data_collector.c
	gcc ${CFLAGS} -c data_collector.c
	gcc ${CFLAGS} -o data_collector data_collector.o ${LIB_FLAGS} ${LIB_LINK}

rovioKalmanFilter.o: Kalman/rovioKalmanFilter.c
	gcc ${CFLAGS} -c Kalman/rovioKalmanFilter.c

PIDController: PIDController.cpp
	g++ ${CFLAGS} -c PIDController.cpp

RobotPose: RobotPose.cpp
	g++ ${CFLAGS} -c RobotPose.cpp

simulator: simulator.c
	gcc ${CFLAGS} -c simulator.c
	gcc ${CFLAGS} -o simulator simulator.o ${LIB_FLAGS} ${LIB_LINK}

base_navigator: base_navigator.cpp RobotPose.o rovioKalmanFilter.o PIDController.o
	g++ ${CFLAGS} -c base_navigator.cpp
	g++ ${CFLAGS} -o base_navigator base_navigator.o PIDController.o rovioKalmanFilter.o ${CPP_LIB_FLAGS} ${LIB_KALMAN}

clean:
	rm -rf *.o
	rm -rf data_collector
	rm -rf simulator
	rm -rf RobotPose
	rm -rf PIDController
	rm -rf base_navigator
