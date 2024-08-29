obj-m += mc3479.o

KERNEL_SRC = $(KERNEL_SRC_HERE)

all:
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean