obj-m += cgscreen.o
all:
	make -C /usr/src/linux M=$(PWD) modules
clean:
	make -C /usr/src/linux M=$(PWD) clean
re: all
	sudo rmmod cgscreen
	sudo insmod cgscreen.ko
