obj-m += nectar_temperature.o
nectar_temperature-objs := temp.o

KDIR := /home/tom/onedrive_normco/projects/01_nectar/code/toolchain/kernel/linux
SDIR := $(shell pwd)

all:
	make -C $(KDIR) M=$(SDIR) modules
clean:
	make -C $(KDIR) M=$(SDIR) clean