#/bin/bash

CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if 
CPP_LIB_FLAGS=${LIB_FLAGS} -lrobot_if++
LIB_LINK=-lhighgui -lcv -lcxcore -lm 
LIB_KALMAN = ${LIB_LINK} -lgslcblas -L/usr/lib64/atlas/ -lclapack

all:  maze_navigator CameraPose RobotPose PIDController
#all: data_collector simulator PIDController RobotPose base_navigator camera_tester maze_navigator

#camera_tester: camera_tester.cpp
#	g++ ${CFLAGS} -c camera_tester.cpp
#	g++ ${CFLAGS} -o camera_tester camera_tester.o PIDController.o rovioKalmanFilter.o ${CPP_LIB_FLAGS} ${LIB_KALMAN}

maze_navigator: maze_navigator.cpp RobotPose.o rovioKalmanFilter.o PIDController.o CameraPose.o
	g++ ${CFLAGS} -c maze_navigator.cpp
	g++ ${CFLAGS} -o maze_navigator maze_navigator.o PIDController.o rovioKalmanFilter.o ${CPP_LIB_FLAGS} ${LIB_KALMAN}

#data_collector: data_collector.c
#	gcc ${CFLAGS} -c data_collector.c
#	gcc ${CFLAGS} -o data_collector data_collector.o ${LIB_FLAGS} ${LIB_LINK}

rovioKalmanFilter.o: Kalman/rovioKalmanFilter.c
	gcc ${CFLAGS} -c Kalman/rovioKalmanFilter.c

PIDController: PIDController.cpp
	g++ ${CFLAGS} -c PIDController.cpp

RobotPose: RobotPose.cpp
	g++ ${CFLAGS} -c RobotPose.cpp

CameraPose: CameraPose.cpp
	g++ ${CFLAGS} -c CameraPose.cpp

maze_strategy: maze_strategy.cpp
	g++ ${CFLAGS} -c maze_strategy.cpp
	g++ ${CFLAGS} -o maze_strategy maze_strategy.o ${CPP_LIB_FLAGS}

#simulator: simulator.c
#	gcc ${CFLAGS} -c simulator.c
#	gcc ${CFLAGS} -o simulator simulator.o ${LIB_FLAGS} ${LIB_LINK}

#hall_navigator: hall_navigator.cpp RobotPose.o rovioKalmanFilter.o PIDController.o CameraPose.o
#	g++ ${CFLAGS} -c hall_navigator.cpp
#	g++ ${CLFAGS} -o hall_navigator hall_navigator.o PIDController.o rovioKalmanFilter.o CameraPose.o ${CPP_LIB_FLAGS} ${LIB_KALMAN}

#base_navigator: base_navigator.cpp RobotPose.o rovioKalmanFilter.o PIDController.o
#	g++ ${CFLAGS} -c base_navigator.cpp
#	g++ ${CFLAGS} -o base_navigator base_navigator.o PIDController.o rovioKalmanFilter.o ${CPP_LIB_FLAGS} ${LIB_KALMAN}

clean:
	rm -rf *.o
	rm -rf data_collector
	rm -rf simulator
	rm -rf RobotPose
	rm -rf PIDController
	rm -rf base_navigator
	rm -rf camera_teser
	rm -rf maze_navigator
	rm -rf hall_navigator
