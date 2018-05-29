DexRun install:

**putty into Dexter.**

To SSH into Dexter, you will need an [SSH client](https://en.wikipedia.org/wiki/Comparison_of_SSH_clients) (e.g. [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/)). Configure the IP address in the client software to 192.168.1.142 (this is [Dexters default IP address](../../../wiki/Dexter-Networking)), set the port to 22, and select SSH. 

Username: root
<br>password: klg

**Change Makefile:**
````
cd /usr/src/xillinux/xillybus-lite/demo
nano Makefile
````
change Makefile to:

````
GNUPREFIX=

CC=$(GNUPREFIX)gcc
AR=$(GNUPREFIX)ar
AS=$(GNUPREFIX)as
CXX=$(GNUPREFIX)g++
LD=$(GNUPREFIX)ld
STRIP=$(GNUPREFIX)strip

CFLAGS=-g -I. -O3 -pthread -lm -Wno-unused-result

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
````
ctr+x and hit y then enter to save a file from nano.

**Change pg:** pg is just a simple script that makes it easy to make DexRun from the share folder. 
````
cd /srv/samba/share
nano pg
````
change "uiotest" to "DexRun". It should look like this:
````
#!/bin/sh
ech0 "changing directory"
cd /usr/src/xillinux/xillybus-lite/demo
cp -f /srv/samba/share/DexRun.c .
make
cp -f DexRun /srv/samba/share/.
````
You might want to rename pg to makeDexRun. This would also be a good place to backup the prior DexRun file so that if there are problems, they can be easily reverted.  

**Setup to auto-run DexRun on bootup:**
````
cd /etc/
nano rc.local
````
change "uiotest" to "DexRun"



**Put DexRun.c into /srv/samba/share/**

If you have access to the share from your PC, just save the file there. E.g. to
\\192.168.1.142\share

Otherwise, copy the file from you PC to a USB flash drive, and then eject it and plug it into Dexter, then:
```
mount /dev/sda1 /mnt/usbstick		(if mount doesn't work lsblk and look for usb name)
cd /srv/samba/share
cp /mnt/usbstick/DexRun.c .		(hit y to overwrite if it already exists)
````

In either case, you need to <br>
**Update the file date:**
````
date -s "5mar18 21:30"			(change to current date)
````

**Compile** And now you can compile DexRun.c
````
./pg 2>&1 | grep "error"		(if ./pg has an error I screwed up and gave you uncomplible code)
````

**Kill the currently running program** <sup><a href="https://stackoverflow.com/questions/160924/how-can-i-kill-a-process-by-name-instead-of-pid">1</a></sup>
````
pkill DexRun
````


**Run the new program**
<br>You can run it from the command line, or just restart Dexter to run it via the rc.local file.
```
./DexRun 1 3 0
```

**Debugging notes**
<br>1. printf's used for debugging don't show up until a `\n` is sent. E.g. `printf("hello");` shows nothing at all. `printf(" world\n");` then shows "hello world"
<br>2. printfs slow down network communications when DexRun is not running from the shell. e.g. When it is run on startup from rc.local, any printf's will cause a delay to replies while the system times out waiting for the message to print to nothing. Use sparingly and avoid in areas where speed is critical.


**Back in DDE:**

Format of MOVETO:
<br>`make_ins("M", x(microns), y(microns), z(microns), dir_x, dir_y, dir_z, config_right_left, elbow_up_down, wrist_in_out)`
<br>Example:
<br>`make_ins("M", 0, 0.5/_um, 0.075/_um, 0, 0, -1, 1, 1, 1)`

Format of MOVETOSTRAIGHT:
<br>`make_ins("T", cartesian_speed(micron/sec), x1(microns), y1(microns), z1(microns), dir_x1, dir_y1, dir_z1, config_right_left1, elbow_up_down1, wrist_in_out1, x2, y2, z2, dir_x2, dir_y2, dir_z2, config_right_left2, elbow_up_down2, wrist_in_out2)`
<br>Example:
<br>`make_ins("T", 50000, .1 /_um, 0.5 /_um, 0.075 /_um, 0, 0, -1, 1, 1, 1, -.1/_um, 0.5/_um, 0.075/_um, 0, 0, -1, 1, 1, 1)`

