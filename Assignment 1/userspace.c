
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>              /* open */
#include <unistd.h>             /* exit */
#include <stdint.h>
#include <sys/ioctl.h>          /* ioctl */
#include "adcchardev.h"


uint16_t data;


int ioctl_sel_channel(int file_desc, int channel)
{
    int ret_val;
    ret_val = ioctl(file_desc, SEL_CHANNEL, channel);
    if (ret_val < 0) {
        printf("ioctl_sel_channel failed:%d\n", ret_val);
        exit(-1);
    }
    return 0;
}

int ioctl_sel_alignment(int file_desc, char align)
{
    int ret_val;
    ret_val = ioctl(file_desc, SEL_ALIGNMENT, align);
    if (ret_val < 0) {
        printf("ioctl_sel_alignment failed:%d\n", ret_val);
        exit(-1);
    }
    return 0;
}



/*
 * Main - Call the ioctl functions
 */
int main()
{
    int file_desc, ret_val;
    int channel;
    char align;
    file_desc = open(DEVICE_FILE_NAME, 0);
    if (file_desc < 0) {
        printf("Can't open device file: %s\n", DEVICE_FILE_NAME);
        exit(-1);
    }
    printf("Enter the Channel(0-7):");
    scanf("%d", &channel);
    printf("Enter the Alignment(r or l):");
    scanf(" %c", &align);
    if(channel <0 || channel>7 || (align != 'r' && align != 'l')){
      printf("Invalid Selection of Channel or Alignment\n");
      exit(-1);
    }
    ioctl_sel_channel(file_desc,channel);
    ioctl_sel_alignment(file_desc,align);
    if(read(file_desc,&data,sizeof(data))){
      if(align == 'l'){
        data = data/64;
      }
        printf("DATA read is:%u\n", data);
    }
    close(file_desc);
    return 0;
}
