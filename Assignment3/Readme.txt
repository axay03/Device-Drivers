STEP 1:	Remove Builtin drivers
	Remove the inbuilt drivers for USB devices, by blacklisting them or by removing them manually:
          Blacklisting process:
            Place the following text in etc/modprobe.d/blacklist.conf:
            # Blacklisting inbuilt USB drivers
            blacklist uas
            blacklist usb_storage

				Manually removing them:
					$ sudo rmmod uas
					$ sudo rmmod usb_storage

STEP 2: Build the driver, using the makefile uploaded with it.
          Use the following commands:
          $ sudo make all
          $ sudo insmod usbdriver.ko


STEP 3:	Make sure flash drive is formatted in FAT32 format and has some files for testing purpose, then insert and mount the device.

          $ sudo mount -t vfat /dev/pd1 <mountpoint> -o uid=1000,gid=1000,utf8,dmask=000,fmask=111

STEP 4: Access the Pendrive and do the required operations for testing

STEP 5: Unmount the drive when you are done
          $ umount <mount point>
          or right click on the drive and select unmount

READING KERNEL LOGS:
	For each successfull read and write commands a line would be printed on kernel logs which would have:
				Starting sector
				no of sectors to read/write
				Offset in the page where to read/write
	For failure appropriate faliure cause will be printed

NOTE: Please insert your flash drive only after the driver is inserted

Submitted by:
	Akshay A Soni 				    (2019H1400117P)
	Virendra Singh Chauhan		(2019H1400580P)
