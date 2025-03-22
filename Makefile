ifneq ($(KERNELRELEASE),)
obj-m  := casio4l.o
else
KDIR ?= /lib/modules/`uname -r`/build

all:
	$(MAKE) -C $(KDIR) M=$$PWD
clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
endif

