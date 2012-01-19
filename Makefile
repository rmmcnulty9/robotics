#/bin/bash

CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
LIB_LINK=-lhighgui -lcv -lcxcore -lm

all: robot_test_example lab_0 data_collector data_collector_fir

robot_test_example: robot_test_example.c
	gcc ${CFLAGS} -c robot_test_example.c
	gcc ${CFLAGS} -o robot_test_example robot_test_example.o ${LIB_FLAGS} ${LIB_LINK}
	

lab_0: lab_0.c
	gcc ${CFLAGS} -c lab_0.c
	gcc ${CFLAGS} -o lab_0 lab_0.o ${LIB_FLAGS} ${LIB_LINK}

data_collector: data_collector.c
	gcc ${CFLAGS} -c data_collector.c
	gcc ${CFLAGS} -o data_collector data_collector.o ${LIB_FLAGS} ${LIB_LINK}

data_collector: data_collector_fir.c
	gcc ${CFLAGS} -c data_collector_fir.c
	gcc ${CFLAGS} -o data_collector_fir data_collector_fir.o ${LIB_FLAGS} ${LIB_LINK}
clean:
	rm -rf *.o
	rm -rf robot_test_example
	rm -rf lab_0
	rm -rf data_collector
	rm -rf data_collector_fir
