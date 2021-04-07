These Firmware files go in the /srv/samba/share folder on the robot. For more information on the Firmware system, please see the [Firmware page on the Wiki](https://github.com/HaddingtonDynamics/Dexter/wiki/Firmware).

Note that updated versions of DexRun.c may not be able to support new features in the Gateware unless the drive image is updated. For example, the 6 and 7th access (twist and grip of end effector on new tool interface) requires an interface to the servos which is implemented in the FPGAs and so needs an updated image. 

## DexRun.c Update via DDE
**Write New DexRun.c into /srv/samba/share/ and Restart Robot.**

On current Dexter robots, to update DexRun.c:
1. Use write_to_robot from DDE to put DexRun.c into the default folder (/srv/samba/share) on the robot. Using DDE automatically sets the clock on the robot.
2. Reboot. Just power cycle the robot. The new program should be automatically seen, compiled, and run on startup by the new RunDexRun script.

## DexRun.c Manual update?

If your SD Card Image is old, you may need to take a series of steps to get the system updated to the point where you can update the firmware. For most users, you shouldn't need to do that. If the DDE method fails to work, you can revert to [doing it the old, manual, way](https://github.com/HaddingtonDynamics/Dexter/blob/5874064c494af0c98758fe08ea924fbc6244261e/Firmware/README.md#dexrunc-manual-update) or better yet, just request an updated SD Card image. 

## Notes

### DexRun.c compile via SSH
Assuming your SD card is up to date, you can re-compile DexRun quickly by [establishing a network connection](https://github.com/HaddingtonDynamics/Dexter/wiki/Dexter-Networking) and [SSHing into Dexter](https://github.com/HaddingtonDynamics/Dexter/wiki/Dexter-Networking#shell-access-via-ssh), then, at the command prompt

#### Update the file date:
It is absolutely critical that the system date on the robot be correct before you re-compile DexRun. To check the date, simply type:<br>
`date`
<br>and press enter. If the date is incorrect, set it by typing something like the following:<br>
`date -s "5mar18 21:30"`			(change to current date)<BR>
or by connecting to the robot via DDE, which should set the date automatically.

#### Run compile script 'pg'
And now you can compile DexRun.c<br>
````
cd /srv/samba/share
pkill DexRun
./pg
./DexRun 1 3 1
````
Warning messages may be normal during the `./pg` step, but errors are not.

## Debugging notes

You can debug DexRun.c by [establishing a network connection](https://github.com/HaddingtonDynamics/Dexter/wiki/Dexter-Networking) and [SSHing into Dexter](https://github.com/HaddingtonDynamics/Dexter/wiki/Dexter-Networking#shell-access-via-ssh), then, at the command prompt:

````
cd /srv/samba/share
pkill DexRun
./DexRun 1 3 1
````
This will show you the printfs with debugging data from the DexRun side. 

<br>1. printf's used for debugging don't show up until a `\n` is sent. E.g. `printf("hello");` shows nothing at all. `printf(" world\n");` then shows "hello world"
<br>2. printfs slow down network communications when DexRun is not running from the shell. e.g. When it is run on startup from rc.local, any printf's will cause a delay to replies while the system times out waiting for the message to print to nothing. Use sparingly and avoid in areas where speed is critical.

### DexRun Options
When you run DexRun, there are three options expressed as 3 number separated by spaces. e.g. `DexRun 1 3 1`

- DefaultMode: The first digit controls default settings. A value of 1 loads the default speeds, PID_Ps, AdcCenters.txt, caltables form HiMem.dta, etc... 0 leaves those settings in an unknown state.

- ServerMode: The second digit controls where DexRun looks for commands. 1 = a socket connection on part 50000 expecting raw joint position data, 2 = the command line expecting oplets, 3 = a socket connection on port 50000 expecting commands from DDE with job, seq, start and ends times, an oplet, and a terminating ';'.

- RunMode: The third digit enables a real time monitor. 1 or 2 will start the monitor. The monitor gets position data from the Joint 6 and 7 servos and enables force calculations. Without it, the actual positions of Joint 6 and 7 positions will not be sensed, although they can still be moved.

### Operational Notes

The updated RunDexRun script checks the modified date of the DexRun.c file against the modified date of the compiled DexRun file and if it's newer, it copies the new DexRun.c to the /usr/src/xillinux/xillybus-lite/demo folder, cd's there, makes, and copies the resulting DexRun executable back to /srv/samba/share, then updates the modified date on the executable to match the source, so that won't happen again.

