
CC=gcc
LIBS=-lnmea
IDIR=../../../lib/common
CFLAGS=-I$(IDIR)
DEPS=$(IDIR)

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

nodegps: node_read_gps.o
	$(CC) -o node_read_gps node_read_gps.o $(LIBS) $(CFLAGS)

.PHONY: clean

clean:
	rm -f *.o node_read_gps
