CC = gcc
CFLAGS = -g -std=gnu99 -Llibbmp180 -lbmp180
LDFLAGS = -Wl,-rpath libbmp180/

.PHONY: all
all: test

test: test.c
	$(CC) $(CFLAGS) -o test $^ $(LDFLAGS)

clean:
	rm -f test
