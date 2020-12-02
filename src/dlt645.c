#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<sys/ioctl.h>
#include<sys/time.h>
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>
#include<string.h>
#include<errno.h>
#include<assert.h>

#include<time.h>

#include "dlt645.h"

#define DLT645_ADDR_LEN 6



#define DLT645_ADDR_LEN 6

/**
 * 用于设定串口通讯参数的魔法般的代码，建议勿动
 * 这段代码是从网上某个地方复制来的，已经忘了出处了
 */
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    fprintf(stderr, "设定串口通讯参数为: speed = %d, nBits = %d, parity = %c, stopBits = %d\n", nSpeed, nBits, nEvent, nStop);
    
    struct termios newtio,oldtio;
    if  ( tcgetattr( fd,&oldtio)  !=  0) { 
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent )
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E': 
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':  
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if( nStop == 1 )
        newtio.c_cflag &=  ~CSTOPB;
    else if ( nStop == 2 )
    newtio.c_cflag |=  CSTOPB;
    newtio.c_cc[VTIME]  = 0;//重要
    newtio.c_cc[VMIN] = 100;//返回的最小值  重要
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }

    return 0;
}


/**
 * 获取当前时间戳，单位：秒
 */
int64_t microtime() {
    struct timeval tv;
    gettimeofday(&tv, 0);
    
    int64_t tv_sec = tv.tv_sec;
    int64_t tv_usec = tv.tv_usec;

    return tv_sec * 1000 + tv_usec / 1000;
}




/**
 * 打开一个串口设备，并设置通讯参数
 * 
 * @param const char*       串口设备地址
 * @param int               通讯波特率，如 1200， 2400， 4800 等
 * @param char              奇偶校验位：N = 无校验；O = 奇校验；E = 偶校验；
 * @param int               停止位个数，可传 1 或 2
 */
int dlt645_open(const char* dev, int baud, int dbit, char parity, int sbit) {
    int fd = open(dev, O_RDWR );
    if (fd <= 0){
        fprintf(stderr, "无法打开串口设备 `%s': %s\n", dev, strerror(errno));
        return 0;
    }
    
    
    int ret = set_opt(fd, baud, dbit, parity, sbit);//设置串口属性
    if (ret == -1) {
        fprintf(stderr, "设置串口通讯参数失败: %s", strerror(errno));
        return 0;
    }
    
    return fd;
}

/**
 * 设置波特率
 * 
 * 成功返回 0，失败返回其他值
 */
int dlt645_set_speed(int fd, const unsigned char addr[6], int speed)
{
    unsigned char Z = 0;
    
    if (speed == 2400) {
        Z = 0x08;
    } else if (speed == 4800) {
        Z = 0x10;
    } else if (speed == 9600) {
        Z = 0x20;
    } else {
        fprintf(stderr, "不支持此波特率: %d", speed);
        return EINVAL;
    }
    
    
/*
[发送]68 01 00 12 01 19 20 68 17 01 08 3D 16
正确协议
[发送]68 01 00 12 01 19 20 68 17 01 3B 70 16
*/
    
        /// 1. 发送指令
    /// 1.1 构建要写入的数据帧
    unsigned char wbuf[] = {
        0xFE, 0xFE, 0xFE, 0xFE,             /// 开头
        0x68,                               /// 帧起始符
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /// 地址
        0x68,                               /// 帧起始符
        0x17, 0x01,                         /// 设置波特率指令 数据域长度 1 字节
        Z,                                 /// 数据域 (D0)
        0x00,                               /// 校验码
        0x16                                /// 结束符
    };
    
    /// 1.2 复制地址和数据域到要发送的数据帧中
    memcpy(wbuf + 4 + 1, addr, DLT645_ADDR_LEN);
    
    /**
     * 根据 DL/T 645 标准，对数据域进行 +0x33 操作
     */
    for (int i = 4 + 1 + 6 + 1 + 2; i < 4 + 1 + 6 + 1 + 2 + 1; i++) {
        wbuf[i] += 0x33;
    }
    
    /// 1.3 计算帧校验，并将校验值写入要发送的数据帧中
    int checksum = 0;
    for (int i = 4; i < sizeof(wbuf) - 2; i++) {
        checksum += wbuf[i];
    }
    checksum = checksum % 256;
    wbuf[sizeof(wbuf) - 2] = checksum;
    
    /// 2. 发送数据
    int nwrite = write(fd, wbuf, sizeof(wbuf));
    
    if (nwrite <= 0) {
        fprintf(stderr, "发送数据帧失败: %s\n", strerror(errno));
        return errno;
    } else if (nwrite != sizeof(wbuf)) {
        fprintf(stderr, "未能发送完整的数据帧，本次只发送了 %d/%ld 字节，请重试\n", nwrite, sizeof(wbuf));
        return errno;
    }
    
    unsigned char buf[1024];
    while (1) {
        int readb = read(fd, buf, 1);
        printf("%08x ", buf[0]);
        buf[0] = 0x00;
    }
    
    
    /// TODO: 检查返回值是否成功
    return 0;
}

/**
 * 读取一个数据
 * 
 * @param int                   已经打开的串口设备的文件描述符（使用 dlt645_open 方法打开）
 * @param unsigned char[6]      设备地址
 * @param unsigned char[4]      指令（即数据域 D0, D1, D2, D3）。指令不需要进行 +0x33 的操作，本函数会自动进行 +0x33 的操作
 * @return unsigned char*       读取到的数据内容。注意，该指针指向堆上的一个静态空间，故此方法不是线程安全的
 */
unsigned char* dlt645_read(int fd, const unsigned char addr[6], const unsigned char cmd[4])
{
    int CMD_LEN = 4;
        
    static unsigned char ret_buf[128];     /// 返回的数据指针
    int ret_len;                            /// 要返回的数据的长度
    
    
    
    /// 1. 发送指令
    /// 1.1 构建要写入的数据帧
    unsigned char wbuf[] = {
        0xFE, 0xFE, 0xFE, 0xFE,             /// 开头
        0x68,                               /// 帧起始符
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /// 地址
        0x68,                               /// 帧起始符
        0x11, 0x04,                         /// 读取指令 数据域长度 4 字节
        0x0, 0x0, 0x0, 0x0,             /// 数据域 (D0, D1, D2, D3)
        0x00,                               /// 校验码
        0x16                                /// 结束符
    };
    
    /// 1.2 复制地址和数据域到要发送的数据帧中
    memcpy(wbuf + 4 + 1, addr, DLT645_ADDR_LEN);
    memcpy(wbuf + 4 + 1 + 6 + 1 + 2, cmd, CMD_LEN);
    
    /**
     * 根据 DL/T 645 标准，对数据域进行 +0x33 操作
     */
    for (int i = 4 + 1 + 6 + 1 + 2; i < 4 + 1 + 6 + 1 + 2 + CMD_LEN; i++) {
        wbuf[i] += 0x33;
    }
    
    
    /// 1.3 计算帧校验，并将校验值写入要发送的数据帧中
    int checksum = 0;
    for (int i = 4; i < sizeof(wbuf) - 2; i++) {
        checksum += wbuf[i];
    }
    checksum = checksum % 256;
    wbuf[sizeof(wbuf) - 2] = checksum;
    
    /// 2. 发送数据
    int nwrite = write(fd, wbuf, sizeof(wbuf));
    
    if (nwrite <= 0) {
        fprintf(stderr, "发送数据帧失败: %s\n", strerror(errno));
        return NULL;
    } else if (nwrite != sizeof(wbuf)) {
        fprintf(stderr, "未能发送完整的数据帧，本次只发送了 %d/%ld 字节，请重试\n", nwrite, sizeof(wbuf));
        return NULL;
    }
    
    
    /// 3. 开始读取数据
    
    /// 3.1 设为非阻塞模式
    int opt = 1;
    ioctl(fd, FIONBIO, &opt);
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK | O_NDELAY);
    
    
    /// 3.2 开始正式读取数据
    /**
     * 这里使用一个小型的状态机来读取数据
     */
    enum {
        S_FE,
        S_BEGIN,
        S_ADDR,
        S_BEGIN2,
        S_COMMAND,
        S_DLEN,
        S_DATA,
        S_CS,
        S_END,
    };
    
    unsigned char d[128];
    int state = S_FE;
    int expect = 1;
    int64_t last_read_time = microtime();       /// 站点最后一次向我们发送数据的时间
    
    while (1) {
        
        /**
         * 读取 expect 字节个数据，同时顺便判断站点响应是否超时
         */
        int nread = 0;
        
        while (1) {
            int ret;
            fd_set rfds;
            struct timeval tv;
            
            FD_ZERO(&rfds);
            FD_SET(fd, &rfds);
            
            tv.tv_sec = 0;
            tv.tv_usec = 1000 * 1000 / 2400 * 10;
            
            ret = select(fd + 1, &rfds, NULL, NULL, &tv);
            
            ret = read(fd, d + nread, expect - nread);
            assert(nread <= sizeof(d));
            
            if (ret <= 0) {
                if (errno == EAGAIN) {
                    /**
                     * 根据 DL/T 645 的规定，站点响应 1 个字节的时间必须在 500ms 以内。
                     * 如果距离上次读到数据，已经超过了 500ms + 10ms（10ms 用于容错），则认为站点异常
                     */
                    if (microtime() - last_read_time > 500 + 10) {
                        fprintf(stderr, "站点超过 500ms 没有继续发送数据，站点异常\n");
                        return NULL;
                    } else {
                        continue;
                    }
                } else {
                    fprintf(stderr, "读取数据时发生错误: %s\n", strerror(errno));
                    return NULL;
                }
            } else {
                nread += ret;
                last_read_time = microtime();
                
                
                /// 调试用：取消下面几行代码的注释可以查看原始的站点返回
                debug("<< ");
                for (int i = 0; i < nread; i++) {
                    debug(" %02x ", d[i]);
                    fflush(stdout);
                }
                
                // debug("nread = %d, ret = %d\n", nread, ret);
                
                
            }
            
            if (nread >= expect) {
                break;
            } else {
                continue;
            }
        }
        
        debug(" 进入状态机处理\n");
        
        
        /**
         * S_FE 状态，我们应该能够读取到 4 个或更少的 0xfe
         */
        if (state == S_FE) {
            debug("开始进行状态机处理 S_FE\n");
            
            static int fe_read_count = 0;
            
            /// 读取一个数据，如果这个数据是 fe，则丢弃，并继续读取下一个数据；否则，进入下一个状态
            if (d[0] == 0xfe) {
                fe_read_count++;
                
                if (fe_read_count > 8) {
                    fprintf(stderr, "站点发送了超过 8 个 0xfe，认为站点异常\n");
                    return NULL;
                }
                continue;
            } else {
                fe_read_count = 0;
                
                state = S_BEGIN;
                expect = 1;
            }
        }
            
        /**
         * S_BEGIN 状态，我们应该读取到一个 0x68 （帧起始符）
         */
        if (state == S_BEGIN) {
            debug("开始进行状态机处理 S_BEGIN\n");
            
            if (d[0] == 0x68) {
                state = S_ADDR;
                expect = 6;         /// 接下来将要读取地址，地址应该有 6 个字符
                continue;
            } else {
                fprintf(stderr, "站点没有发送 0x68 帧起始符，站点发送的数据是 %02x\n", d[0]);
                return NULL;
            }
        }
        
        /**
         * S_ADDR 状态，直接跳过
         * FIXME: 应该对比返回的地址与我们发送的地址是否相同，如果地址不同，说明消息可能不是发给我们的，应该丢弃
         */
        if (state == S_ADDR) {
            debug("开始进行状态机处理 S_ADDR\n");
            
            /// 直接跳过这个状态
            state = S_BEGIN2;
            expect = 1;
            continue;
        }
        
        if (state == S_BEGIN2) {
            debug("开始进行状态机处理 S_BEGIN2\n");
            
            if (d[0] != 0x68) {
                fprintf(stderr, "站点没有发送控制指令开始的 0x68 起始符，站点发送的数据是 %02x\n", d[0]);
                return NULL;
            } else {
                state = S_COMMAND;
                expect = 1;
                continue;
            }
        }
        
        if (state == S_COMMAND) {
            debug("开始进行状态机处理 S_COMMAND\n");
            
            /// 直接跳过这个状态
            /// FIXE: 应该检查指令是否符合 DL/T 645 标准
            state = S_DLEN;
            expect = 1;
            continue;
        }
        
        
        if (state == S_DLEN) {
            debug("开始进行状态机处理 S_DLEN, d[0] = 0x%02x\n", d[0]);
            
            /// 保存数据长度
            ret_len = d[0];
            ret_len -= 4;           /// 相应帧的长度总是 4 + 数据长度（其中前 4 个字节是我们发送的数据域，后面几个字节才是站点的回复）
            
            if (ret_len < 0) {
                fprintf(stderr, "站点返回的数据长度小于 4，这应该是不合法的\n");
                return NULL;
            } else if (ret_len == 0) {
                state = S_CS;
                expect = 1;
            } else {
                /// 初始化返回值缓冲
                memset(ret_buf, 0x00, sizeof(ret_buf));
                
                state = S_DATA;
                expect = d[0];
            }
            
            
            
            continue;
        }
        
        
        if (state == S_DATA) {
            debug("开始进行状态机处理 S_DATA\n");
            
            
            memcpy(ret_buf, d + 4, ret_len);    ///  将站点发来的数据保存到返回缓存区
            
            /**
             * 根据 DL/T 645 标准，对数据域进行 -0x33 操作
             */
            for (int i = 0; i < ret_len; i++) {
                ret_buf[i] -= 0x33;
            }
            
            /// 进入下一个状态
            state = S_CS;
            expect = 1;
            continue;
        }
        
        
        if (state == S_CS) {
            debug("开始进行状态机处理 S_CS\n");
            
            /// 直接跳过这个状态
            /// FIXME: 应该检查帧校验值是否正确
            state = S_END;
            expect = 1;
            continue;
        }
        
        
       if (state == S_END) {
           debug("开始进行状态机处理 S_END\n");
            
           /// 结束咯，收工
           break;
       }
    }
        
        
    return ret_buf;
}

int dlt645_close(int fd) {
    return close(fd);
}

char* dlt645_bcd2str(const unsigned char *bcd, int len) {
    static char buf[128];
    
    assert(len * 2 < sizeof(buf) - 1);
    
    memset(buf, 0x00, sizeof(buf));
    
    for (int i = 0; i < len; i++) {
        snprintf(buf + i * 2, 3, "%02x", bcd[len - i - 1]);
    }

    //printf("输入: ");
    for (int i = 0; i < len; i++) {
	    printf("%02x ", bcd[i]);
    }
    //printf("   , 输出: %s\n", buf);
        
    return buf;
}


