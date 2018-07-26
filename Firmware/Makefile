GNUPREFIX=

CC=$(GNUPREFIX)gcc
AR=$(GNUPREFIX)ar
AS=$(GNUPREFIX)as
CXX=$(GNUPREFIX)g++
LD=$(GNUPREFIX)ld
STRIP=$(GNUPREFIX)strip

CFLAGS=-g -I. -O3 -pthread -lm -Wno-unused-result -Wno-unused-variable -Wno-unused-but-set-variable

APPLICATION=DexRun

INTDEMO=intdemo

OBJECTS=# core.o

all: $(APPLICATION) $(INTDEMO)

$(APPLICATION): $(OBJECTS) $(APPLICATION).o
	$(CC)  $(CFLAGS) $(OBJECTS) $(APPLICATION).o -o $(APPLICATION) -lrt -lm 

$(INTDEMO): $(OBJECTS) $(INTDEMO).o
	$(CC)  $(CFLAGS) $(OBJECTS) $(INTDEMO).o -o $(INTDEMO)

clean:
	rm -f *~ $(APPLICATION)  $(INTDEMO) *.o

.c.o:
	$(CC) $(CFLAGS) -c -o $@ $<

