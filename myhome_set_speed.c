#define __STDC_FORMAT_MACROS

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>


#include "src/dlt645.h"

#define INIT_SPEED 2400
#define TARGET_SPEED 9600

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: <%s> <device_path>\n", argv[0]);
        exit(-1);
    }

    
    /**
     * 电表地址
     */
    unsigned char addr[] = {0x88, 0x92, 0x35, 0x24, 0x01, 0x26};
//    unsigned char addr[] = {0x04, 0x99, 0x00, 0x11, 0x01, 0x19};
//      unsigned char addr[] = {0x22, 0x42, 0x02, 0x14, 0x05, 0x20};
  
	printf("使用 %d 波特率打开串口\n", INIT_SPEED);

    int fd = dlt645_open(argv[1], INIT_SPEED, 8, 'N', 2);
//    int fd = dlt645_open(argv[1], 2400, 8, 'N', 2);
    if (fd <= 0) {
        exit(-2);
    }
    
    
    int ret;
    ret = dlt645_set_speed(fd, addr, TARGET_SPEED);
    if (ret == 0) {
	    printf("设置应该成功了，现在的通讯速率应该已经改为 %d\n", TARGET_SPEED);
    } else {
	    printf("设置可能没有成功: %s", strerror(errno));
    }


    
    
    
    dlt645_close(fd);
    
    return 0;
}

