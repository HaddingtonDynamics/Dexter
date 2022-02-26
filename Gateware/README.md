FPGA logic files. The .ldl file is the source code for the FPGA, which can be opened by Viva, a graphical FPGA design program which we will make available very soon. The .BIT file is the compiled image for the FPGA.

The FPGA Gateware is loaded from the [SDCard Image](../../../wiki/SD-Card-Image)
Prior versions of the FPGA Gateware can be found at:<br>
https://github.com/kgallspark/Dexter (The [Firmware](../Firmware) folder here replaces the uiotest.c / DexRun.c files in that repo)

**Update .bit files**

First, copy the new .bit file to the share folder. 

[Establish a network connection and SSH into Dexter](https://github.com/HaddingtonDynamics/Dexter/wiki/Dexter-Networking) and use WinSCP or whatever utility you have to tansfer the file.

At the command prompt in SSH

`md5sum /srv/samba/share/xillydemo.bit`

and verify the output matches the source file.

Next, to find the FAT partition.

`lsblk`

find the block with a SIZE that matches the FAT partition size of 15 or so Megs. e.g.

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

So now you should be able to see the SD card FAT share at /mnt/usbstick, e.g:

````
ls /mnt/usbstick/
DexRun.c                   boot.bin        uImage
System Volume Information  devicetree.dtb  xillydemo.bit
````

and now you can copy over the new .bit or other files you previously copied to the share. e.g. 

`cp /srv/samba/share/xillydemo.bit /mnt/usbstick/`

Confirm the copy by pressing "Y". You can use the linux cmp command to verify the file has copied:

`cmp -b /srv/samba/share/xillydemo.bit /mnt/usbstick/xillydemo.bit`

if it has no response, the files are equal. If there is a difference, it will tell you. You can also test with md5sum:

`md5sum /mnt/usbstick/xillydemo.bit`

If you also updated [DexRun.c in the share folder, you should recompile](https://github.com/HaddingtonDynamics/Dexter/blob/master/Firmware/README.md) it at this point, but don't kill and relauch DexRun

To ensure the data is written to the SD card, vs being cashed in RAM, use the 

`sync` 

command to write caches to the drive.

Now restart using `reboot`, because shutdown -r now does not re-load the bit file.

`reboot`

Power cycling should not be required, but it's good to be safe and just do that anyway.

