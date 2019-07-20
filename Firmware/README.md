These Firmware files go in the /srv/samba/share folder on the robot. For more information on the Firmware system, please see the [Firmware page on the Wiki](https://github.com/HaddingtonDynamics/Dexter/wiki/Firmware).

Note that updated versions of DexRun.c may not be able to support new features in the Gateware unless the drive image is updated. For example, the 6 and 7th access (twist and grip of end effector on new tool interface) requires an interface to the servos which is implemented in the FPGAs and so needs an updated image. 

## DexRun.c Manual update?

If your SD Card Image is old, you may need to take a series of steps to get the system updated to the point where you can update the firmware. For most users, you shouldn't need to do that. If the following fails to work, you can revert to [doing it the old, manual, way](https://github.com/HaddingtonDynamics/Dexter/blob/5874064c494af0c98758fe08ea924fbc6244261e/Firmware/README.md#dexrunc-manual-update)

## DexRun.c Update via DDE
**Write New DexRun.c into /srv/samba/share/**

To update DexRun.c:
1. Use write_to_robot from DDE to put DexRun.c into the default folder (/srv/samba/share) on the robot. 
2. Reboot. You can do `shutdown -r` or just power cycle the robot. The new program should be automatically seen, compiled, and run on startup by the new RunDexRun script.

## Notes
**Debugging notes**

You can edit, compile, and debug DexRun.c by [establishing a network connection](https://github.com/HaddingtonDynamics/Dexter/wiki/Dexter-Networking) and [SSHing into Dexter](https://github.com/HaddingtonDynamics/Dexter/wiki/Dexter-Networking#shell-access-via-ssh), then, at the command prompt:

````
cd /srv/samba/share
pkill DexRun
./DexRun 1 3 1
````
This will show you the printfs with debugging data from the DexRun side. 

<br>1. printf's used for debugging don't show up until a `\n` is sent. E.g. `printf("hello");` shows nothing at all. `printf(" world\n");` then shows "hello world"
<br>2. printfs slow down network communications when DexRun is not running from the shell. e.g. When it is run on startup from rc.local, any printf's will cause a delay to replies while the system times out waiting for the message to print to nothing. Use sparingly and avoid in areas where speed is critical.

**Operational Notes**

The updated RunDexRun script checks the modified date of the DexRun.c file against the modified date of the compiled DexRun file and if it's newer, it copies the new DexRun.c to the /usr/src/xillinux/xillybus-lite/demo folder, cd's there, makes, and copies the resulting DexRun executable back to /srv/samba/share, then updates the modified date on the executable to match the source, so that won't happen again.

**Back in DDE:**

Format of MOVETO:
<br>`make_ins("M", x(microns), y(microns), z(microns), dir_x, dir_y, dir_z, config_right_left, elbow_up_down, wrist_in_out)`
<br>Example:
<br>`make_ins("M", 0, 0.5/_um, 0.075/_um, 0, 0, -1, 1, 1, 1)`

Format of MOVETOSTRAIGHT:
<br>`make_ins("T", cartesian_speed(micron/sec), x1(microns), y1(microns), z1(microns), dir_x1, dir_y1, dir_z1, config_right_left1, elbow_up_down1, wrist_in_out1, x2, y2, z2, dir_x2, dir_y2, dir_z2, config_right_left2, elbow_up_down2, wrist_in_out2)`
<br>Example:
<br>`make_ins("T", 50000, .1 /_um, 0.5 /_um, 0.075 /_um, 0, 0, -1, 1, 1, 1, -.1/_um, 0.5/_um, 0.075/_um, 0, 0, -1, 1, 1, 1)`

