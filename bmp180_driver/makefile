CC = gcc
CFLAGS = -g -std=gnu99

all: test

test: test.o driver.o timer.o calibration.o
	$(CC) $(CFLAGS) $^ -o test

test.o: test.c bmp180.h
	$(CC) $(CFLAGS) -c $<

driver.o: driver.c bmp180.h registers.h
	$(CC) $(CFLAGS) -c $<

timer.o: timer.c bmp180.h
	$(CC) $(CFLAGS) -c $<

calibration.o: calibration.c bmp180.h
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *.o test

