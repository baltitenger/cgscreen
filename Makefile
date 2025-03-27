ifneq ($(KERNELRELEASE),)
obj-m := casio4l.o
else
# since linux 6.13
KDIR ?= /lib/modules/$(shell uname -r)/build
export KBUILD_EXTMOD := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
include $(KDIR)/Makefile
endif
