obj-m += mc3479.o

KERNEL_SRC = $(KERNEL_SRC_HERE)

all:
	make -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean