all: rovioKalmanFilter_test

CFLAGS=-Wall -ggdb
LIBS=-lgslcblas -L/usr/lib64/atlas -lclapack

rovioKalmanFilter_test: rovioKalmanFilter.o roviokalmanFilter_test.c
	gcc ${CFLAGS}  -c roviokalmanFilter_test.c
	gcc ${CFLAGS} -o rovioKalmanFilter_test roviokalmanFilter_test.o rovioKalmanFilter.o ${LIBS}

rovioKalmanFIlter.o: rovioKalmanFilter.c
	gcc ${CFLAGS} -c rovioKalmanFilter.c

clean:
	rm -f *.o rovioKalmanFilter_test TR.csv
