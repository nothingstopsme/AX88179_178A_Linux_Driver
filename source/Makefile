CURRENT	= $(shell uname -r)
#TARGET	= usbnet
#OBJS	= usbnet.o
TARGET	= ax88179_178a
OBJS	= ax88179_178a.o
MDIR	= drivers/net/usb
KDIR	= /lib/modules/$(CURRENT)/build
#KDIR	= /root/work/2440/2.6.17/linux-2.6.17.11
SUBLEVEL= $(shell uname -r | cut -d '.' -f 3 | cut -d '.' -f 1 | cut -d '-' -f 1 | cut -d '_' -f 1)
USBNET	= $(shell find $(KDIR)/include/linux/usb/* -name usbnet.h)

ifneq (,$(filter $(SUBLEVEL),14 15 16 17 18 19 20 21))
MDIR = drivers/usb/net
#USBNET	= $(shell find $(KDIR)/$(MDIR)/* -name usbnet.h)
endif

#ifneq (,$(filter $(SUBLEVEL),21 22 23 24))
#USBNET	= $(shell find $(KDIR)/$(MDIR)/* -name usbnet.h)
#endif

#$(if $(USBNET),,$(error $(KDIR)/$(MDIR)/usbnet.h not found. please refer to readme file for the detailed description))

EXTRA_CFLAGS = -DEXPORT_SYMTAB
PWD = $(shell pwd)
DEST = /lib/modules/$(CURRENT)/kernel/$(MDIR)

obj-m      := $(TARGET).o

default:
	make -C $(KDIR) M=$(PWD) modules

$(TARGET).o: $(OBJS)
	$(LD) $(LD_RFLAG) -r -o $@ $(OBJS)

install:
	su -c "cp -v $(TARGET).ko $(DEST) && /sbin/depmod -a"

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm *.mod

.PHONY: modules clean

-include $(KDIR)/Rules.make
