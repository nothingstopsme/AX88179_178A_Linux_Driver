============================================================================
ASIX AX88179_178A USB 3.0/2.0 Gigabit Ethernet Network Adapter
Driver Compilation & Configuration on Linux
============================================================================

================
Prerequisites
================

Prepare to build the driver, you need the Linux kernel sources installed on the
build machine, and make sure that the version of the running kernel must match
the installed kernel sources. If you don't have the kernel sources, you can get
it from www.kernel.org or contact to your Linux distributor. If you don't know
how to do, please refer to KERNEL-HOWTO.

Note: Please make sure the kernel is built with one of the "Support for
       Host-side, EHCI, OHCI, or UHCI" option support.

================
File Description
================
README		This file
ax88179_178a.c	AX88179_178A Linux driver main file
ax88179_178a.h	AX88179_178A Linux driver header file
Makefile	AX88179_178A driver make file
COPYING	GNU GERNERAL LICENSE

===========================
Conditional Compilation Flag
===========================

================
Getting Start
================

1. Extract the compressed driver source file to your temporary directory by the
   following command:

	[root@localhost template]# tar -xf DRIVER_SOURCE_PACKAGE.tar.bz2

2. Now, the driver source files should be extracted under the current directory.
   Executing the following command to compile the driver:
 
	[root@localhost template]# make
			
3. If the compilation is done, the ax88179_178a.ko will be created under the current
   directory.
 
4. If you want to use modprobe command to mount the driver, executing the
   following command to install the driver into your Linux:

	[root@localhost template]# make install


================
Usage
================

1. If you want to load the driver manually, go to the driver directory and
   execute the following commands:

	[root@localhost template]# modprobe usbnet
	[root@localhost template]# insmod ax88179_178a.ko

If you want to unload the driver, just executing the following command:

	[root@localhost anywhere]# rmmod axax88179_178a

===============
DRIVER PARAMETERS
===============
The following parameters can be set when using insmod.

msg_enable=0xNNNNNNN
	specifies the msg_enable of usbnet.

example: insmod ax88179_178a.ko msg_enable=0x00000000


bsize=xx (0~24)
	specifies the the Rx Bulk In Queue size(KB).
	The maximum value for this parameters is 24. 
	The default value is -1 that will use the driver default setting (18K for USB3.0).

example: insmod ax88179_178a.ko bsize=12

ifg=xxx (0~255)
	specifies the the Rx Bulk In Queue Inter-Frame-Gap timer. (The timer's unit is 0.25us)
	The maximum value for this parameters is 255.
	The default value is -1 that will use the driver default setting (255 for USB3.0).

bEEE=x	(0 or 1)
	Enable/Disable the Ethernet EEE function.
	0: Disable the EEE
	1: Enalbe the EEE
	The default value is 0 that will disable the EEE function.

bGETH=x (0 or 1)
	Enable/Disable the Green Ethernet function.
	0: Disable the Green Ethernet
	1: Enalbe the Green Ethernet
	The default value is 0 that will disable the Green Ethernet function.
