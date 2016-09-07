CC = gcc
CFLAGS = -g -std=gnu99 -shared -fPIC -lgcc

all: libbmp180.so

libbmp180.so: driver.c calibration.c timer.c
	$(CC) $(CFLAGS) $^ -o libbmp180.so

clean:
	rm -f libbmp180.so *.o

