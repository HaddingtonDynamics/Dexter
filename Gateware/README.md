Future home of FPGA logic files.

The FPGA Gateware is loaded from the [SDCard Image](../../wiki/SD-Card-Image)
Currently, the FPGA Gateware can be found at:<br>
https://github.com/kgallspark/Dexter (The [Firmware](../Firmware) folder here replaces the uiotest.c / DexRun.c files in that repo)

**Update .bit files**

First, copy the new .bit file to the share folder.

[Establish a network connection to Dexter and SSH in](https://github.com/HaddingtonDynamics/Dexter/wiki/Dexter-Networking)

At the command prompt

`lsblk`

find the block with a SIZE that matches the size of your sd card. e.g.

````
NAME        MAJ:MIN RM   SIZE RO TYPE MOUNTPOINT
mmcblk0     179:0    0    15G  0 disk
|-mmcblk0p1 179:1    0  15.7M  0 part
`-mmcblk0p2 179:2    0   7.4G  0 part /
mtdblock0    31:0    0     4M  0 disk
mtdblock1    31:1    0     1M  0 disk
mtdblock2    31:2    0     5M  0 disk
mtdblock3    31:3    0   128K  0 disk
mtdblock4    31:4    0   5.9M  0 disk
mtdblock5    31:5    0     4M  0 disk
````
then mount that. in this case, because the name is mmcblk0p1 do:

`mount /dev/mmcblk0p1 /mnt/usbstick/`

So now you should be able to see the SD card FAT share at /mnt/usbstick:

````
ls /mnt/usbstick/
DexRun.c                   boot.bin        uImage
System Volume Information  devicetree.dtb  xillydemo.bit
````

and now you can copy over the new .bit or other files you previously copied to the share. e.g. 

`cp /srv/samba/share/xillydemo.bit /mnt/usbstick/`

If you also updated [DexRun.c in the share folder, you should recompile](https://github.com/HaddingtonDynamics/Dexter/blob/master/Firmware/README.md) it at this point, but don't kill and relauch DexRun

Now restart. 

`shutdown -r now`



