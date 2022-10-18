#include<linux/kernel.h>
#include<linux/module.h>
#include<linux/usb.h>
#include<linux/slab.h>

# define SANDISK_32GB_VID 0x0781
# define SANDISK_32GB_PID 0x558a

# define HP_8GB_VID 0x03f0
# define HP_8GB_PID 0x5607

# define DIRECTION_IN 0x80
# define DIRECTION_OUT 0x00

# define BOMS_RESET 0xFF
# define BOMS_RESET_TYPE 0x21

# define BOMS_GET_MAX_LUN 0xFE
# define BOMS_GET_MAX_LUN_TYPE 0xA1

# define be_to_int32(buf)(((buf)[0] << 24) | ((buf)[1] << 16) | ((buf)[2] << 8) | (buf)[3])

//definations of functions used
static int get_mass_storage_status(struct usb_device * , uint8_t, uint32_t);
static int send_mass_storage_command(struct usb_device * , uint8_t, uint8_t, uint8_t * , uint8_t, int, uint32_t * );
static int test_usbstorage(struct usb_device * , struct usb_interface * , uint8_t, uint8_t);

static uint8_t cdb_length[256] = {
//	 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  0
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  1
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  2
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  3
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  4
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  5
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  6
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  7
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  8
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  9
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  A
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  B
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  C
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  D
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  E
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  F
};

typedef struct {
  uint8_t dCSWSignature[4];
  uint32_t dCSWTag;
  uint32_t dCSWDataResidue;
  uint8_t bCSWStatus;
}
command_status_wrapper;

typedef struct {
  uint8_t dCBWSignature[4];
  uint32_t dCBWTag;
  uint32_t dCBWDataTransferLength;
  uint8_t bmCBWFlags;
  uint8_t bCBWLUN;
  uint8_t bCBWCBLength;
  uint8_t CBWCB[16];
}
command_block_wrapper;

uint8_t endpoint_in;
uint8_t endpoint_out;
uint32_t expected_tag;

static int probe_func(struct usb_interface * interface,
    const struct usb_device_id * id) {
    int i, returned;
    unsigned char epAddr, epAttr;
    struct usb_endpoint_descriptor * ep_desc;
    struct usb_device * udev;
    udev = container_of(interface -> dev.parent, struct usb_device, dev); // getting the usb_device pointer

    printk(KERN_INFO "**********************************My Log Begins ************************************************");
    printk(KERN_INFO "KNOWN USB DRIVE DETECTED\n");
    printk(KERN_INFO "Vendor ID = %#06x \n", udev -> descriptor.idVendor); //using udev to get to the device descriptor for VID
    printk(KERN_INFO "Product ID = %#06x \n", udev -> descriptor.idProduct); //using udev to get to the device descriptor for PID

    printk(KERN_INFO "USB DEVICE CLASS : %x", interface -> cur_altsetting -> desc.bInterfaceClass);
    printk(KERN_INFO "USB DEVICE SUB CLASS : %x", interface -> cur_altsetting -> desc.bInterfaceSubClass);
    printk(KERN_INFO "USB DEVICE Protocol : %x", interface -> cur_altsetting -> desc.bInterfaceProtocol);

    printk(KERN_INFO "No. of Endpoints = %d\n", interface -> cur_altsetting -> desc.bNumEndpoints);
    for (i = 0; i < interface -> cur_altsetting -> desc.bNumEndpoints; i++) // IMP :: desc is the interface descriptor
    {
      ep_desc = & interface -> cur_altsetting -> endpoint[i].desc; // IMP :: desc here is the endpoint descriptor
      epAddr = ep_desc -> bEndpointAddress;
      epAttr = ep_desc -> bmAttributes;
      if ((epAttr & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK) {
        if (epAddr & 0x80) {
          printk(KERN_INFO "EP %d is Bulk IN\n", i);
          endpoint_in = ep_desc -> bEndpointAddress;
        } else {
          printk(KERN_INFO "EP %d is Bulk OUT\n", i);
          endpoint_out = ep_desc -> bEndpointAddress;
        }
      }
    }
    //check if valid scsi device
    if ((interface -> cur_altsetting -> desc.bInterfaceClass == 8) && ((interface -> cur_altsetting -> desc.bInterfaceSubClass == 0x01) || (interface -> cur_altsetting -> desc.bInterfaceSubClass == 0x06)) && (interface -> cur_altsetting -> desc.bInterfaceProtocol == 0x50)) {
			printk(KERN_INFO "Detected device is a valid SCSI mass storage device.\n");
			printk(KERN_INFO "Initiating SCSI commands\n");
			returned = test_usbstorage(udev, interface, endpoint_in, endpoint_out);
      if (returned < 0) {
        printk(KERN_INFO "test_usbstorage function failed.....Exiting\n");
        return -1;
      }
		}
		else{
			printk(KERN_INFO "Detected device is not a valid SCSI mass storage device.\n");
		}
      return 0;
    }

    static int send_mass_storage_command(struct usb_device * udev, uint8_t endpoint, uint8_t lun,
      uint8_t * cdb, uint8_t direction, int data_length, uint32_t * ret_tag) {
      static uint32_t tag = 1;
      uint8_t cdb_len;
      int i, size, returned;
      command_block_wrapper * cbw = (command_block_wrapper * ) kmalloc(sizeof(command_block_wrapper), GFP_KERNEL);
      if (cdb == NULL) {
        return -1;
      }
      cdb_len = cdb_length[cdb[0]];
      if ((cdb_len == 0) || (cdb_len > sizeof(cbw -> CBWCB))) {
        printk(KERN_INFO "send_mass_storage_command: don't know how to udev this command (%02X, length %d)\n",
          cdb[0], cdb_len);
        return -1;
      }
      memset(cbw, 0, sizeof(cbw));
      cbw -> dCBWSignature[0] = 'U';
      cbw -> dCBWSignature[1] = 'S';
      cbw -> dCBWSignature[2] = 'B';
      cbw -> dCBWSignature[3] = 'C';
      * ret_tag = tag;
      cbw -> dCBWTag = tag++;
      cbw -> dCBWDataTransferLength = data_length;
      cbw -> bmCBWFlags = direction;
      cbw -> bCBWLUN = lun;
      cbw -> bCBWCBLength = cdb_len;
      memcpy(cbw -> CBWCB, cdb, cdb_len);
      i = 0;
      do {
        returned = usb_bulk_msg(udev, usb_sndbulkpipe(udev, endpoint), (void * ) cbw, 31, & size, 1000);

        i++;
      } while ((returned != 0) && (i < 5));
      if (returned != 0) {
        printk(KERN_INFO "usb_bulk_msg function failed with code%d and no of bytes transfered is %d number of times tried is %d\n", returned, size, i);
        return -1;
      }
      printk(KERN_INFO " Successfuly sent %d CDB bytes\n", size);
      return 0;
    }

    static int test_usbstorage(struct usb_device * udev, struct usb_interface * interface, uint8_t endpoint_in, uint8_t endpoint_out) {
      int size, returned;
      long max_lba, block_size;
      long device_size;
      uint8_t cdb[16]; // SCSI Command Descriptor Block
      uint8_t * buffer = (uint8_t * ) kmalloc(64 * sizeof(uint8_t), GFP_KERNEL);
      uint8_t * lun = (uint8_t * ) kmalloc(sizeof(uint8_t), GFP_KERNEL);

			printk(KERN_INFO "\n**********************************************************************************************************\n");

			printk(KERN_INFO "Sending Bulk-only mass storage reset command:\n");
      returned = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), BOMS_RESET, BOMS_RESET_TYPE, 0, interface -> cur_altsetting -> desc.bInterfaceNumber, 0, 0, 0);
      if (returned < 0) {
        printk(KERN_INFO "Bulk-only mass storage reset command failed error returned is  %d \n", returned);
        return -1;
      }
      printk(KERN_INFO "USB Device Reset successful \n");


  		printk(KERN_INFO "\n**********************************************************************************************************\n");

			printk(KERN_INFO "Sending Get Max LUN command:\n");
			returned = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), BOMS_GET_MAX_LUN, BOMS_GET_MAX_LUN_TYPE, 0, interface -> cur_altsetting -> desc.bInterfaceNumber, (void * ) lun, 1, 0);
			if (returned < 0) {
				printk(KERN_INFO "Get max lun command failed error returned is  %d \n", returned);
				return -1;
			}
			printk(KERN_INFO "max lun is  %u \n", * lun);


			printk(KERN_INFO "\n**********************************************************************************************************\n");

      printk(KERN_INFO "Sending Read Capacity command:\n");
      memset(buffer, 0, sizeof(buffer));
      memset(cdb, 0, sizeof(cdb));
      cdb[0] = 0x25; // Read capacity command
      send_mass_storage_command(udev, endpoint_out, * lun, cdb, DIRECTION_IN, 0x08, & expected_tag);
      returned = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, endpoint_in), buffer, 64, & size, 0);
      if (returned < 0) {
        printk(KERN_INFO "Data stage failed for read_capacity (error code %d) \n", returned);
        return -1;
      }
      max_lba = (long) be_to_int32( & buffer[0]);
      block_size = (long) be_to_int32( & buffer[4]);
      device_size = (((max_lba + 1) / (1024 * 1024)) * block_size) / 1024;
      printk(KERN_INFO "   Max LBA: %08li, Block Size: %08li (%li GB) \n", max_lba, block_size, device_size);
      if (get_mass_storage_status(udev, endpoint_in, expected_tag) == -1) {
        printk(KERN_INFO " Function get_mass_storage_status failed");
      }
      return 0;
    }

    static int get_mass_storage_status(struct usb_device * udev, uint8_t endpoint, uint32_t expected_tag) {
      int returned, size;
      command_status_wrapper * csw = (command_status_wrapper * ) kmalloc(sizeof(command_status_wrapper), GFP_KERNEL);;

      returned = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, endpoint), (void * ) csw, 31, & size, 0);
      if (returned != 0) {
        printk(KERN_INFO "Response CSW transaction failed");
        return -1;
      }

      if (size != 13) {
        printk(KERN_INFO "   get_mass_storage_status: received %d bytes (expected 13)\n", size);
        return -1;
      }
      if (csw -> dCSWTag != expected_tag) {
        printk(KERN_INFO "   get_mass_storage_status: mismatched tags (expected %08X, received %08X)\n",
          expected_tag, csw -> dCSWTag);
        return -1;
      }
      printk(KERN_INFO "   Mass Storage Status: %02X (%s)\n", csw -> bCSWStatus, csw -> bCSWStatus ? "FAILED" : "Success");
      return 0;
    }

    static void disconnect_func(struct usb_interface * interface) {
      printk(KERN_INFO "USB Device Removed\n");
      return;
    }

    static struct usb_device_id usbdev_table[] = {
      {USB_DEVICE(SANDISK_32GB_VID, SANDISK_32GB_PID)},
      {USB_DEVICE(HP_8GB_VID, HP_8GB_PID)},
      {}
    };

    static struct usb_driver usbdev_driver = {
      name: "usbdev", //name of the device
      probe: probe_func, // Whenever Device is plugged in
      disconnect: disconnect_func, // When we remove a device
      id_table: usbdev_table, //  List of devices served by this driver
    };

    static int __init init_func(void) {
      int result;
      result = usb_register( & usbdev_driver);
      if (result < 0) {
        printk(KERN_NOTICE "FAILED TO REGISTER UAS_READ_CAPACITY DRIVER (Error code %d)\n", result);
        return -1;
      }
      printk(KERN_NOTICE "UAS_READ_CAPACITY DRIVER INSERTED\n");
      return 0;
    }

    static void __exit exit_func(void) {
      usb_deregister( & usbdev_driver);
      printk(KERN_NOTICE "UAS_READ_CAPACITY DRIVER REMOVED\n");
    }

    module_init(init_func);
    module_exit(exit_func);
    MODULE_LICENSE("GPL");
