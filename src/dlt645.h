#ifndef __GREENSEA_DLT645_H__
#define __GREENSEA_DLT645_H__



/// 注释此行，可在运行时打印调试信息
//#define DEBUG 1
#ifdef DEBUG
    #define debug printf
#else
    #define debug(...) fflush(stderr)
#endif


/**
 * 获取毫秒级的当前 UNIX 时间戳，单位：毫秒
 */
long microtime();



int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);


/**
 * 打开一个串口设备，并设置通讯参数
 * 
 * @param const char*       串口设备地址
 * @param int               通讯波特率，如 1200， 2400， 4800 等
 * @param char              奇偶校验位：N = 无校验；O = 奇校验；E = 偶校验；
 * @param int               停止位个数，可传 1 或 2
 */
int dlt645_open(const char* dev, int baud, int dbit, char parity, int sbit);

/**
 * 读取一个数据
 * 
 * @param int                   已经打开的串口设备的文件描述符（使用 dlt645_open 方法打开）
 * @param unsigned char[6]      设备地址
 * @param unsigned char[4]      指令（即数据域 D0, D1, D2, D3）。指令不需要进行 +0x33 的操作，本函数会自动进行 +0x33 的操作
 * @return unsigned char*       读取到的数据内容。注意，该指针指向堆上的一个静态空间，故此方法不是线程安全的
 */
unsigned char* dlt645_read(int fd, const unsigned char addr[6], const unsigned char cmd[4]);

/**
 * 设置通讯速率
 *
 * @param int                   已经打开的串口设备的文件描述符（使用 dlt645_open 方法打开）
 * @param unsigned char[6]      设备地址
 * @param int                   波特率
 */
int dlt645_set_speed(int fd, const unsigned char addr[6], int speed);


int dlt645_close(int fd);


/**
 * 将站点返回的 BCD 码的数据转换成字符串
 * 
 * @param const unsigned char*  站点返回的数据
 * @param int                   站点返回的数据的长度
 */
char* dlt645_bcd2str(const unsigned char *bcd, int len);


#endif
