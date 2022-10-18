#include<linux/kernel.h>
#include<linux/init.h>
#include<linux/module.h>
#include<linux/slab.h>
#include<linux/blkdev.h>
#include<linux/genhd.h>
#include<linux/spinlock.h>
#include <linux/bio.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include<linux/usb.h>

#define DEVICE_NAME 									"pd"
#define SECTOR_SIZE 									512

# define SANDISK_32GB_VID 								0x0781
# define SANDISK_32GB_PID 								0x558a

# define HP_8GB_VID 									0x03f0
# define HP_8GB_PID 									0x5607

# define DIRECTION_IN 									0x80
# define DIRECTION_OUT 									0x00

# define BOMS_RESET 									0xFF
# define BOMS_RESET_TYPE 								0x21

# define BOMS_GET_MAX_LUN 								0xFE
# define BOMS_GET_MAX_LUN_TYPE		 						0xA1

#define REQUEST_SENSE_LENGTH          							0x12
#define INQUIRY_LENGTH                							0x24

# define be_to_int32(buf)(((buf)[0] << 24) | ((buf)[1] << 16) | ((buf)[2] << 8) | (buf)[3])
# define __bio_kunmap_atomic(addr, kmtype) kunmap_atomic(addr)


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


struct private_dev {
        int size;                       /* Device size in sectors */
        u8 *data;                       /* The data array */
        short users;                    /* How many users */
        short media_change;             /* Flag a media change? */
        spinlock_t lock;                /* For mutual exclusion */
        struct request_queue *queue;    /* The device request queue */
        struct gendisk *gd;             /* The gendisk structure */
        struct timer_list timer;        /* For simulated media changes */
};

struct scsi_work{
  struct request *req;
	struct work_struct work;
}scsi_work;


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


//global param declaration
struct usb_device * udev;
struct usb_interface * globalinterface;
uint8_t endpoint_in;
uint8_t endpoint_out;
uint32_t expected_tag;
int err =0;


//definations of functions used
static struct private_dev *p_blkdev = NULL;
static struct workqueue_struct *scsiqueue = NULL;
static int get_mass_storage_status(uint32_t);
static int send_mass_storage_command(uint8_t * , uint8_t, int, uint32_t * );
static int get_max_lun(void);
static int reset_recovery(void);
static void scsi_router(struct work_struct *work);
long scsi_read_capacity(void);
int scsi_read(sector_t,char*,sector_t);
int scsi_write(sector_t ,char*,sector_t);
int scsi_request_sense(void);
void scsi_inquiry(void);

static int block_open(struct block_device *bdev, fmode_t mode)
{
	struct private_dev *dev = bdev->bd_disk->private_data;
	spin_lock(&dev->lock);
	if (! dev->users)
		check_disk_change(bdev);
	dev->users++;
	spin_unlock(&dev->lock);
	return 0;
}

static void block_release(struct gendisk *gd, fmode_t mode)
{
	struct private_dev *dev = gd->private_data;
	spin_lock(&dev->lock);
	dev->users--;
	spin_unlock(&dev->lock);
}

int block_media_changed(struct gendisk *gd)
{
    struct private_dev *dev = gd->private_data;
    printk(KERN_INFO "INSIDE block_media_changed");
    return dev->media_change;
}

int block_revalidate(struct gendisk *gd)
{
    struct private_dev *dev = gd->private_data;
		printk(KERN_INFO "INSIDE block_revalidate");
    if (dev->media_change) {
        dev->media_change = 0;
        memset (dev->data, 0, dev->size);
    }
    return 0;
}

void block_request(struct request_queue *q)
{
		struct request *req;
    struct scsi_work *usb_work = NULL;
		while((req = blk_fetch_request(q)) != NULL)
		{
      if (blk_rq_is_passthrough(req)) {
        __blk_end_request_all(req, -EIO);
      }
			usb_work = (struct scsi_work*)kmalloc(sizeof(struct scsi_work),GFP_ATOMIC);
      if(usb_work == NULL){
        printk("Memory Allocation for defferd work failed");
          __blk_end_request_cur(req, 0);
          continue;
      }
      usb_work->req = req;
      INIT_WORK(&usb_work->work, scsi_router);
      queue_work(scsiqueue,&usb_work->work);
		}
}

static void scsi_router(struct work_struct *work){
  struct scsi_work *usb_work =NULL;
  struct request *req = NULL;
  struct req_iterator iter;
  struct bio_vec bvec;
  int returned;
	sector_t start_sector,xfer_sector;
	char *page;
	unsigned int offset;
	unsigned long flags;
  usb_work = container_of(work,struct scsi_work,work);
  req=usb_work->req;
  rq_for_each_segment(bvec,req,iter){	//here bvec is current biovector
		start_sector = iter.iter.bi_sector;
    page = page_address(bvec.bv_page);
	  xfer_sector = bvec.bv_len/512;//number of sectors
	  offset = bvec.bv_offset;
		page = page + offset;
	  //write
	   if(rq_data_dir(req)){
	    returned = scsi_write(start_sector,page,xfer_sector);
	    /*if(returned<0){*/printk(KERN_INFO"****WRITE*****Start sector = %llu ::: total no of sectors = %llu ::: offset = %u",start_sector,xfer_sector,offset);//}
	   }
	   //read
	   else{
	     returned = scsi_read(start_sector,page,xfer_sector);
	     /*if(returned<0){*/printk(KERN_INFO"****READ*****Start sector = %llu ::: total no of sectors = %llu ::: offset = %u",start_sector,xfer_sector,offset);//}
	   }
	   kunmap_atomic(page);
  }
	spin_lock_irqsave(&p_blkdev->lock,flags);
 	__blk_end_request_cur(req,0);
	spin_unlock_irqrestore(&p_blkdev->lock,flags);
	kfree(usb_work);
	return;
}

static struct block_device_operations blkdev_ops = {
	.owner           	= THIS_MODULE,
	.open 	         	= block_open,
	.release 	 			 	= block_release,
	.media_changed		= block_media_changed,
	.revalidate_disk	= block_revalidate
};


static int probe_func(struct usb_interface * interface,const struct usb_device_id * id) {
    int i, returned;
		struct gendisk *usb_drive = NULL;
    unsigned char epAddr, epAttr;
    struct usb_endpoint_descriptor * ep_desc;
		long max_lba = 0;
    globalinterface = interface;	//assigning interface to global variable
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
			returned = get_max_lun();
      if (returned < 0) {
        printk(KERN_INFO "get_max_lun function failed\n");
      }
			max_lba = scsi_read_capacity();
			if (max_lba < 0) {
        printk(KERN_INFO "scsi_read_capacity function failed\n");
      }
			scsi_inquiry();		//fetch and display info about your device
		}
		else{
			printk(KERN_INFO "Detected device is not a valid SCSI mass storage device.\n");
		}

		err = register_blkdev(0, "Pendrive");
		if (err < 0)
			printk(KERN_WARNING "DiskonRAM: unable to get major number\n");
		p_blkdev = kmalloc(sizeof(struct private_dev),GFP_KERNEL);
		if(!p_blkdev)
		{
			printk("ENOMEM  at %d\n",__LINE__);
			return 0;
		}
		memset(p_blkdev, 0, sizeof(struct private_dev));
		spin_lock_init(&p_blkdev->lock);
		p_blkdev->queue = blk_init_queue(block_request, &p_blkdev->lock);
		usb_drive = p_blkdev->gd = alloc_disk(2);
		if(!usb_drive)
		{
			kfree(p_blkdev);
			printk(KERN_INFO "alloc_disk failed\n");
			return 0;
		}

		scsiqueue = create_workqueue("scsiqueue");
		INIT_WORK(&scsi_work.work, scsi_router);

		usb_drive->major = err;
		usb_drive->first_minor = 0;
		usb_drive->fops = &blkdev_ops;
		usb_drive->queue = p_blkdev->queue;
		usb_drive->private_data = p_blkdev;
		strcpy(usb_drive->disk_name, DEVICE_NAME);
		set_capacity(usb_drive, max_lba+1);
		printk(KERN_INFO "\n");
		add_disk(usb_drive);
		printk(KERN_INFO "Registered block driver\n");
		return 0;
}

static int send_mass_storage_command(uint8_t * cdb, uint8_t direction, int data_length, uint32_t * ret_tag) {
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
      cbw -> bCBWLUN = 0;
      cbw -> bCBWCBLength = cdb_len;
      memcpy(cbw -> CBWCB, cdb, cdb_len);
      i = 0;
      do {
        returned = usb_bulk_msg(udev, usb_sndbulkpipe(udev, endpoint_out), (void *)cbw, 31, & size, 0);
        i++;
      } while ((returned != 0) && (i < 5));
      if (returned != 0) {
        printk(KERN_INFO "usb_bulk_msg function failed with code%d and no of bytes transfered is %d number of times tried is %d\n", returned, size, i);
				return -1;
      }
      return 0;
}

static int get_max_lun()
{
      int returned;
			uint8_t * lun = (uint8_t * ) kmalloc(sizeof(uint8_t), GFP_KERNEL);
			printk(KERN_INFO "\n**********************************************************************************************************\n");
			printk(KERN_INFO "Sending Get Max LUN command:\n");
			returned = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), BOMS_GET_MAX_LUN, BOMS_GET_MAX_LUN_TYPE, 0, globalinterface -> cur_altsetting -> desc.bInterfaceNumber, (void * ) lun, 1, 0);
			if (returned < 0) {
				printk(KERN_INFO "Get max lun command failed error returned is  %d \n", returned);
				return -1;
			}
			printk(KERN_INFO "max lun is  %u \n", * lun);
			printk(KERN_INFO "\n**********************************************************************************************************\n");
			return 0;
}

static int reset_recovery()
{
	int returned;
	printk(KERN_INFO "Reset recovery called  \n");
	returned = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), BOMS_RESET, BOMS_RESET_TYPE, 0, globalinterface -> cur_altsetting -> desc.bInterfaceNumber, 0, 0, 0);
	usb_clear_halt(udev, usb_sndctrlpipe(udev, endpoint_in));
	usb_clear_halt(udev, usb_sndctrlpipe(udev, endpoint_out));
	return 0;
}

long scsi_read_capacity(){
			int size,returned;
			long max_lba = 0, block_size, device_size;
			uint8_t * buffer = (uint8_t * ) kmalloc(64 * sizeof(uint8_t), GFP_KERNEL);
			uint8_t cdb[16]; // SCSI Command Descriptor Block
      memset(buffer, 0, sizeof(buffer));
      memset(cdb, 0, sizeof(cdb));
      cdb[0] = 0x25; // Read capacity command
      send_mass_storage_command(cdb, DIRECTION_IN, 0x08, & expected_tag);
      returned = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, endpoint_in), (void *)buffer, 31, & size, 0);
			printk(KERN_INFO "returned value %d and size is %d", returned, size);
      if (returned < 0) {
				if(returned == -32){//stalled pipe
					usb_clear_halt(udev,usb_rcvbulkpipe(udev, endpoint_in));
				}
        printk(KERN_INFO "Data stage failed for read_capacity command(error code %d) \n", returned);
      }
			else{
      max_lba = (long) be_to_int32( & buffer[0]);
      block_size = (long) be_to_int32( & buffer[4]);
      device_size = (((max_lba + 1) / (1024 * 1024)) * block_size) / 1024;
      printk(KERN_INFO "   Max LBA: %08li, Block Size: %08li (%li GB) \n", max_lba, block_size, device_size);
			}
      if (get_mass_storage_status(expected_tag) == -2) {
        printk(KERN_INFO " Function get_mass_storage_status failed");
				scsi_request_sense();
      }
			return max_lba;
}

void scsi_inquiry(){
			char vid[9], pid[9], rev[5];
			int i,size,returned;
			uint8_t cdb[16];
			uint8_t * inquiry = (uint8_t *) kmalloc(64 * sizeof(uint8_t), GFP_KERNEL);
			printk("Sending Inquiry:\n");
			memset(inquiry, 0, sizeof(inquiry));
			memset(cdb, 0, sizeof(cdb));
			cdb[0] = 0x12;	// Inquiry
			cdb[4] = INQUIRY_LENGTH;
			send_mass_storage_command(cdb, DIRECTION_IN, INQUIRY_LENGTH, &expected_tag);
			returned = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, endpoint_in), (void *)inquiry, INQUIRY_LENGTH, &size, 0);
			if (returned < 0)
			{
				printk("Request sense failed: %d\n", returned);
			}
			printk("   received %d bytes\n", size);
			// The following strings are not zero terminated
			for (i=0; i<8; i++) {
				vid[i] = inquiry[8+i];
				pid[i] = inquiry[16+i];
				rev[i/2] = inquiry[32+i/2];	// instead of another loop
			}
			vid[8] = 0;
			pid[8] = 0;
			rev[4] = 0;
			printk("   VID:PID:REV \"%8s\":\"%8s\":\"%4s\"\n", vid, pid, rev);
			if (get_mass_storage_status(expected_tag) == -2) {
        printk(KERN_INFO " Function get_mass_storage_status failed");
				scsi_request_sense();
      }
			return;
}
int scsi_request_sense(){
			uint8_t cdb[16];	// SCSI Command Descriptor Block
			uint32_t expected_tag;
			int size;
			int returned;
			uint8_t * sense = (uint8_t *) kmalloc(18 * sizeof(uint8_t), GFP_KERNEL);
			printk("Request Sense:\n");
			memset(sense, 0, sizeof(sense));
			memset(cdb, 0, sizeof(cdb));
			cdb[0] = 0x03;	// Request Sense
			cdb[4] = REQUEST_SENSE_LENGTH;
			send_mass_storage_command(cdb, DIRECTION_IN, REQUEST_SENSE_LENGTH, &expected_tag);
			returned = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, endpoint_in), (void *)sense, REQUEST_SENSE_LENGTH, &size, 0);
			if (returned < 0)
			{
				printk("Request sense failed: %d\n", returned);
			}
			printk("   received %d bytes\n", size);
			if ((sense[0] != 0x70) && (sense[0] != 0x71)) {
				printk("   ERROR No sense data\n");
			} else {
				printk("   ERROR Sense: %02X %02X %02X\n", sense[2]&0x0F, sense[12], sense[13]);
			}
			get_mass_storage_status(expected_tag);
			return 0;
}

int scsi_read(sector_t start_sector, char* page,sector_t xfer_sector){
			int size,returned;
			int size_in_bytes;
			uint8_t cdb[16];
			size_in_bytes = xfer_sector*512;
      memset(cdb, 0, sizeof(cdb));
			cdb[0] = 0x28; // Read data command for READ (10)
			cdb[2] = (start_sector >> 24) & 0xFF ;
			cdb[3] = (start_sector >> 16) & 0xFF ;
			cdb[4] = (start_sector >> 8) & 0xFF ;
			cdb[5] = (start_sector >> 0) & 0xFF ;
			cdb[7] = (xfer_sector >> 8) & 0xFF ;
			cdb[8] = (xfer_sector >> 0) & 0xFF ;
			send_mass_storage_command(cdb, DIRECTION_IN, size_in_bytes, & expected_tag);
			returned = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, endpoint_in),(void *)page, size_in_bytes, & size, 0);
      if (returned < 0) {
				if(returned == -32){		//stalled pipe
					usb_clear_halt(udev,usb_rcvbulkpipe(udev, endpoint_in));
				}
        printk(KERN_INFO "Data stage failed for Read command (error code %d) \n", returned);
      }
			if (get_mass_storage_status(expected_tag) == -2) {
        printk(KERN_INFO " Function get_mass_storage_status failed");
				scsi_request_sense();
      }
	return 0;
}

int scsi_write(sector_t start_sector,char* page,sector_t xfer_sector){
			uint8_t cdb[16];
			int size_in_bytes;
			int size = 0,returned;
			size_in_bytes = xfer_sector*512;
			memset(cdb, 0, sizeof(cdb));
			cdb[0] = 0x2A; //data command for write (10)
			cdb[2] = (start_sector >> 24) & 0xFF ;
			cdb[3] = (start_sector >> 16) & 0xFF ;
			cdb[4] = (start_sector >> 8) & 0xFF ;
			cdb[5] = (start_sector >> 0) & 0xFF ;
			cdb[7] = (xfer_sector >> 8) & 0xFF ;
			cdb[8] = (xfer_sector >> 0) & 0xFF ;
			send_mass_storage_command(cdb, DIRECTION_OUT, size_in_bytes, & expected_tag);
			returned = usb_bulk_msg(udev, usb_sndbulkpipe(udev, endpoint_out),(void *)page, size_in_bytes, & size, 0);
			if (returned < 0) {
				if(returned == -32){		//stalled pipe
						usb_clear_halt(udev, usb_sndbulkpipe(udev, endpoint_out));
				}
				printk(KERN_INFO "Data stage failed for WRITE command (error code %d) \n", returned);
			}
			if(get_mass_storage_status(expected_tag) == -2) {
				printk(KERN_INFO " Function get_mass_storage_status failed");
				scsi_request_sense();
			}
			return 0;
}


static int get_mass_storage_status(uint32_t expected_tag) {
      int returned, size,i;
      command_status_wrapper * csw = (command_status_wrapper * ) kmalloc(sizeof(command_status_wrapper), GFP_KERNEL);;
			i = 0;
			do {
      returned = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, endpoint_in), (void *) csw, 13, & size, 0);
			usb_clear_halt(udev,usb_rcvbulkpipe(udev, endpoint_in));
			i++;
			} while ((returned == -32) && (i<2));
      if (returned != 0) {
        printk(KERN_INFO "Response CSW transaction failed (error code:%d)and size is%d no of tries%d",returned,size,i);
        return -1;
      }
      if (size != 13) {
        printk(KERN_INFO "Get_mass_storage_status: received %d bytes (expected 13)\n", size);
        return -1;
      }
      if (csw -> dCSWTag != expected_tag) {
        printk(KERN_INFO "Get_mass_storage_status: mismatched tags (expected %08X, received %08X)\n",expected_tag, csw -> dCSWTag);
        return -1;
      }
			if (csw->bCSWStatus == 0x01)
				return -2;	// request Get Sense

			if(csw -> bCSWStatus == 0x02)
				reset_recovery();

			printk(KERN_INFO "   Mass Storage Status: %02X (%s)\n", csw -> bCSWStatus, csw -> bCSWStatus ? "FAILED" : "Success");
      return 0;
}

static void disconnect_func(struct usb_interface * interface) {
			struct gendisk *usb_drive = p_blkdev->gd;
			printk(KERN_INFO "USB Device Removed\n");
			del_gendisk(usb_drive);
			blk_cleanup_queue(p_blkdev->queue);
			flush_workqueue(scsiqueue);
			destroy_workqueue(scsiqueue);
			kfree(p_blkdev);
      return;
}

static struct usb_device_id usbdev_table[] = {
      {USB_DEVICE(SANDISK_32GB_VID, SANDISK_32GB_PID)},
      {USB_DEVICE(HP_8GB_VID, HP_8GB_PID)},
      {}
};

static struct usb_driver usbdev_driver = {
      name: "usbdev", 								//name of the device
      probe: probe_func, 							// Whenever Device is plugged in
      disconnect: disconnect_func, 		// When we remove a device
      id_table: usbdev_table, 				// List of devices served by this driver
};

static int __init init_func(void) {
      int result;
      result = usb_register( & usbdev_driver);
      if (result < 0) {
        printk(KERN_NOTICE "FAILED TO REGISTER DRIVER (Error code %d)\n", result);
        return -1;
      }
      printk(KERN_NOTICE "DRIVER INSERTED\n");
      return 0;
}

static void __exit exit_func(void) {
      usb_deregister( & usbdev_driver);
      printk(KERN_NOTICE "DRIVER REMOVED\n");
}

    module_init(init_func);
    module_exit(exit_func);
    MODULE_LICENSE("GPL");
	MODULE_DESCRIPTION("USB mass storage device driver");
	MODULE_AUTHOR("AKSHAY A SONI <h20190117@pilani.bits-pilani.ac.in>");
