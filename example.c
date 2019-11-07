#include <stdio.h>
#include <stdlib.h>

#include "src/ttyusb.h"


int main() {
    /**
     * 电表地址
     */
    unsigned char addr[] = {0x04, 0x99, 0x00, 0x11, 0x01, 0x19};
    
    /**
     * 将这个路径修改为你的串口设备路径
     */
    int fd = dlt645_open("/dev/ttyUSB0", 2400, 8, 'N', 2);
    
    
    unsigned char* buf;
    
    unsigned char cmd1[] = {0x00, 0x01, 0x01, 0x02};
    buf = dlt645_read(fd, addr, cmd1);
    printf("电压: %lf\n", atol(dlt645_bcd2str(buf, 3)) / 10.0);
    
    unsigned char cmd2[] = {0x00, 0x01, 0x02, 0x02};
    buf = dlt645_read(fd, addr, cmd2);
    printf("电流: %lf\n", atol(dlt645_bcd2str(buf, 2)) / 1000.0);
    
    unsigned char cmd3[] = {0x00, 0x00, 0x03, 0x02};
    buf = dlt645_read(fd, addr, cmd3);
    printf("功率: %lf\n", atol(dlt645_bcd2str(buf, 3)) * 1000 / 10000.0);
    
    
    unsigned char cmd4[] = {0x00, 0x00, 0x06, 0x02};
    buf = dlt645_read(fd, addr, cmd4);
    printf("功率因数: %lf\n", atol(dlt645_bcd2str(buf, 2)) / 1000.0);
    
    unsigned char cmd5[] = {0x02, 0x00, 0x80, 0x02};
    buf = dlt645_read(fd, addr, cmd5);
    printf("频率: %lf\n", atol(dlt645_bcd2str(buf, 2)) / 100.0);
    
    
    unsigned char cmd6[] = {0x00, 0x00, 0x00, 0x00};
    buf = dlt645_read(fd, addr, cmd6);
    printf("电量: %lf\n", atol(dlt645_bcd2str(buf, 4)) / 100.0);
    
    dlt645_close(fd);
    
    
    return 0;
}
