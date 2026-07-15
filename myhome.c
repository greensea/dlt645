#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>

#include "src/dlt645.h"

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: <%s> <device_path> <data_path>\n", argv[0]);
        exit(-1);
    }
    
    /**
     * 电表地址
     */
    unsigned char addr[] = {0xff, 0xff, 0xff, 0x6b, 0x66, 0x32}; 	// fzj2-3P 
//    unsigned char addr[] = {0x88, 0x92, 0x35, 0x24, 0x01, 0x26}; 	// fzj2
//    unsigned char addr[] = {0x04, 0x99, 0x00, 0x11, 0x01, 0x19};	// fzj
//    unsigned char addr[] = {0x20, 0x05, 0x14, 0x02, 0x42, 0x22};
//    unsigned char addr[] = {0x22, 0x42, 0x02, 0x14, 0x05, 0x20};	/// lj
    
    int fd = dlt645_open(argv[1],9600, 8, 'E', 1);
    if (fd <= 0) {
        exit(-2);
    }
    
    
    /**
     * 处理数据文件地址
     */
    char tmp_path[1024];
    char path[1024];
    
    strncpy(path, argv[2], sizeof(path) - 1);
    strncpy(tmp_path, argv[2], sizeof(tmp_path) - 1 - strlen(".tmp"));
    strcat(tmp_path, ".tmp");

    fprintf(stderr, "数据文件是 `%s'\n", path);
    fprintf(stderr, "临时数据文件是 `%s'\n", tmp_path);
    

    int64_t last_timestamp = 0;
    int64_t timestamp = 0;

    
    while (1) {
        last_timestamp = microtime();
        
        double V, I, P, Pc, F, W;
        
        /// 1. 开始读取各项数据
        unsigned char* buf;
        
        unsigned char cmd1[] = {0x00, 0x01, 0x01, 0x02};
        buf = dlt645_read(fd, addr, cmd1);
        V = atol(dlt645_bcd2str(buf, 2)) / 10.0;
        printf("电压: %lf  ", V);
        
        unsigned char cmd2[] = {0x00, 0x01, 0x02, 0x02};
        buf = dlt645_read(fd, addr, cmd2);
        I = atol(dlt645_bcd2str(buf, 3)) / 1000.0;
        printf("电流: %lf  ", I);
        
        unsigned char cmd3[] = {0x00, 0x00, 0x03, 0x02};
        buf = dlt645_read(fd, addr, cmd3);
        P = atol(dlt645_bcd2str(buf, 3)) * 1000 / 10000.0;
        printf("功率: %lf  ", P);
        
        
        unsigned char cmd4[] = {0x00, 0x00, 0x06, 0x02};
        buf = dlt645_read(fd, addr, cmd4);
        Pc = atol(dlt645_bcd2str(buf, 2)) / 1000.0;
        printf("功率因数: %lf  ", Pc);
        
        unsigned char cmd5[] = {0x02, 0x00, 0x80, 0x02};
        buf = dlt645_read(fd, addr, cmd5);
        F = atol(dlt645_bcd2str(buf, 2)) / 100.0;
        printf("频率: %lf  ", F);
        
        
        unsigned char cmd6[] = {0x00, 0x00, 0x00, 0x00};
        buf = dlt645_read(fd, addr, cmd6);
        W = atol(dlt645_bcd2str(buf, 4)) / 100.0;
        printf("电量: %lf\n", W);
        
        timestamp = microtime();
        
        
        /// 2. 将数据写入文件
        char file_buf[10240];
        const char* fmt = "{    \n\
    \"V\": \"%lf\",  \n\
    \"I\": \"%lf\",  \n\
    \"P\": \"%lf\",  \n\
    \"Pc\": \"%lf\",  \n\
    \"F\": \"%lf\",  \n\
    \"W\": \"%lf\",  \n\
    \"timestamp\": \"%" PRId64 "\",  \n\
    \"timestamp_ms\": \"%" PRId64 "\"  \n\
}";

        FILE* fp = fopen(tmp_path, "w");
        if (!fp) {
            fprintf(stderr, "无法打开临时数据文件: `%s'\n", strerror(errno));
            exit(-3);
        }
        fprintf(fp, fmt, V, I, P, Pc, F, W, timestamp / 1000, timestamp);
        fclose(fp);
        
        int ret = rename(tmp_path, path);
        if (ret < 0) {
            fprintf(stderr, "将临时数据文件复制为数据文件失败: %s\n", strerror(errno));
        }
        
        
        
        /// 3. 休眠到下一个整数秒
        
        long sleep_time = 1000 - (microtime() % 1000) + 1;
        if (sleep_time <= 0) {
            /// 什么都不用做
            fprintf(stderr, "上次操作耗时 %ldms，准备进行下一次读取\n", timestamp - last_timestamp);
        } else {
            fprintf(stderr, "休眠 %dms 后进行下一次读取\n", sleep_time);
            usleep((sleep_time - 1) * 1000);
        }
    }
    
    
    
    
    dlt645_close(fd);
    
    return 0;
}
