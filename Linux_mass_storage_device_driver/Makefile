obj-m := usbdd.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules
clean:
	find . -type f | xargs -n 5 touch
	rm -rf $(OBJS)
