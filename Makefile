# Makefile

export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-

ifeq (${KERNELRELEASE},)
    KERNEL_SOURCE := ../linux
    PWD := $(shell pwd)
default:
	${MAKE} -C ${KERNEL_SOURCE} SUBDIRS=${PWD} modules

clean:
	${MAKE} -C ${KERNEL_SOURCE} SUBDIRS=${PWD} clean

else
	obj-m := dw1000.o
#	CFLAGS_dw1000.o := -DDEBUG
endif

remote_install:
	scp dw1000.ko pi@raspberrypi.local:
	scp dw1000.ko pi@raspberrypi2.local:
