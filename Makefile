KDIR = /home/nikishin/GyroscopeDev/linux-3.0.35
PWD = $(shell pwd)

TARGET1 = lsm9ds0_gyr_spi

obj-m	:= $(TARGET1).o

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	@rm -f *.o .*.cmd .*.flags *.mod.c *.order
	@rm -f .*.*.cmd *~ *.*~ TODO.*
	@rm -fR .tmp*
	@rm -rf .tmp_versions

disclean: clean
	@rm *.ko *.symvers
