#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <inttypes.h>

// 计算 Modbus RTU 的 CRC16 校验码
unsigned short calculate_crc(unsigned char *buf, int len) {
    unsigned short crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (unsigned short)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 获取微秒时间戳
int64_t microtime() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

// 原生 Linux 配置并打开 9600 8N1 串口
int open_modbus_serial(const char *dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) return -1;
    
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;        // 无校验 (None)
    options.c_cflag &= ~CSTOPB;        // 1位停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;            // 8位数据位
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY | INPCK | ISTRIP);
    
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

// 用于将两个 16 位寄存器组合成一个 32 位无符号整数
uint32_t combine_32bit(unsigned char high_msb, unsigned char high_lsb, unsigned char low_msb, unsigned char low_lsb) {
    uint32_t high = (high_msb << 8) | high_lsb;
    uint32_t low = (low_msb << 8) | low_lsb;
    return (high << 16) | low;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: <%s> <device_path> <data_file_path>\n", argv[0]);
        exit(-1);
    }
    
    int fd = open_modbus_serial(argv[1]);
    if (fd < 0) {
        fprintf(stderr, "无法打开串口: %s\n", argv[1]);
        exit(-2);
    }
    
    char tmp_path[1024];
    char path[1024];
    strncpy(path, argv[2], sizeof(path) - 1);
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);

    fprintf(stderr, "数据输出文件: `%s'\n", path);
    fprintf(stderr, "临时中转文件: `%s'\n", tmp_path);
    
    // 批量读取命令：从地址 0x0000 开始，连续读取 37 (0x0025) 个寄存器 (涵盖 0x00 到 0x24)
    unsigned char cmd[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x25, 0x00, 0x00};
    unsigned short crc = calculate_crc(cmd, 6);
    cmd[6] = crc & 0xFF;
    cmd[7] = (crc >> 8) & 0xFF;

    // 预期回包大小 = 1(ID) + 1(Func) + 1(Len) + 37*2(Data) + 2(CRC) = 79 字节
    unsigned char rx_buf[128];
    int expected_len = 79;

    while (1) {
        int64_t last_timestamp = microtime();
        
        // 声明全电参变量
        double v_a = 0.0, v_b = 0.0, v_c = 0.0;
        double i_a = 0.0, i_b = 0.0, i_c = 0.0, i_n = 0.0;
        double p_total = 0.0, p_a = 0.0, p_b = 0.0, p_c = 0.0;
        double pf_total = 0.0, pf_a = 0.0, pf_b = 0.0, pf_c = 0.0;
        double freq_a = 0.0, freq_b = 0.0, freq_c = 0.0;
        double w_active_fwd = 0.0, w_active_rev = 0.0;

        tcflush(fd, TCIOFLUSH);
        write(fd, cmd, 8);
        usleep(200000); // 留出 200ms 等待多寄存器长包回复
        
        int n = read(fd, rx_buf, sizeof(rx_buf));
        
        if (n >= expected_len && rx_buf[0] == 0x01 && rx_buf[1] == 0x03) {
            // Modbus 数据区起始偏移：rx_buf[3] 开始是第一个寄存器高字节
            // rx_buf[3 + 2*N] 即为寄存器 N 的高字节偏移
            #define REG_PTR(addr) (3 + (addr) * 2)
            #define GET_16BIT(addr) ((rx_buf[REG_PTR(addr)] << 8) | rx_buf[REG_PTR(addr) + 1])
            
            // 1. 电压 (0x00 ~ 0x02)
            v_a = GET_16BIT(0x00) / 10.0;
            v_b = GET_16BIT(0x01) / 10.0;
            v_c = GET_16BIT(0x02) / 10.0;

            // 2. 电流 (0x03 ~ 0x06)
            i_a = GET_16BIT(0x03) / 1000.0;
            i_b = GET_16BIT(0x04) / 1000.0;
            i_c = GET_16BIT(0x05) / 1000.0;
            i_n = GET_16BIT(0x06) / 1000.0;

            // 3. 有功功率 (0x07 ~ 0x0A) -> 说明书显示格式 XXXX，无需除以系数
            p_total = (short)GET_16BIT(0x07);
            p_a     = (short)GET_16BIT(0x08);
            p_b     = (short)GET_16BIT(0x09);
            p_c     = (short)GET_16BIT(0x0A);

            // 4. 功率因数 (0x13 ~ 0x16) -> 格式 XX.XX
            pf_total = GET_16BIT(0x13) / 100.0;
            pf_a     = GET_16BIT(0x14) / 100.0;
            pf_b     = GET_16BIT(0x15) / 100.0;
            pf_c     = GET_16BIT(0x16) / 100.0;

            // 5. 频率 (0x1A ~ 0x1C) -> 格式 XX.XX
            freq_a = GET_16BIT(0x1A) / 100.0;
            freq_b = GET_16BIT(0x1B) / 100.0;
            freq_c = GET_16BIT(0x1C) / 100.0;

            // 6. 电量 (0x1D ~ 0x20 为32位双寄存器) -> 格式 XX.XX
            uint32_t raw_w_fwd = combine_32bit(rx_buf[REG_PTR(0x1D)], rx_buf[REG_PTR(0x1D)+1], rx_buf[REG_PTR(0x1E)], rx_buf[REG_PTR(0x1E)+1]);
            uint32_t raw_w_rev = combine_32bit(rx_buf[REG_PTR(0x1F)], rx_buf[REG_PTR(0x1F)+1], rx_buf[REG_PTR(0x20)], rx_buf[REG_PTR(0x20)+1]);
            w_active_fwd = raw_w_fwd / 100.0;
            w_active_rev = raw_w_rev / 100.0;

            printf("📊 采集 -> 电压: %.1fV/%.1fV/%.1fV | 电流: %.3fA/%.3fA | 总功率: %.0fW | 正向电量: %.2fkWh\n", 
                   v_a, v_b, v_c, i_a, i_b, p_total, w_active_fwd);
        } else {
            fprintf(stderr, "⚠️ 读取失败，包大小不足或CRC校验不匹配 (收到字节数: %d)\n", n);
        }
        
        int64_t timestamp = microtime();
        
        // 构造完整的全电参结构体化 JSON 写入
        FILE* fp = fopen(tmp_path, "w");
        if (fp) {
            fprintf(fp, "{\n"
                        "    \"V_A\": \"%.2lf\", \"V_B\": \"%.2lf\", \"V_C\": \"%.2lf\",\n"
                        "    \"I_A\": \"%.3lf\", \"I_B\": \"%.3lf\", \"I_C\": \"%.3lf\", \"I_N\": \"%.3lf\",\n"
                        "    \"P_Total\": \"%.2lf\", \"P_A\": \"%.2lf\", \"P_B\": \"%.2lf\", \"P_C\": \"%.2lf\",\n"
                        "    \"PF_Total\": \"%.2lf\", \"PF_A\": \"%.2lf\", \"PF_B\": \"%.2lf\", \"PF_C\": \"%.2lf\",\n"
                        "    \"Freq_A\": \"%.2lf\", \"Freq_B\": \"%.2lf\", \"Freq_C\": \"%.2lf\",\n"
                        "    \"W_Active_Fwd\": \"%.2lf\", \"W_Active_Rev\": \"%.2lf\",\n"
                        "    \"timestamp\": \"%" PRId64 "\",\n"
                        "    \"timestamp_ms\": \"%" PRId64 "\"\n"
                        "}",
                    v_a, v_b, v_c, i_a, i_b, i_c, i_n, p_total, p_a, p_b, p_c,
                    pf_total, pf_a, pf_b, pf_c, freq_a, freq_b, freq_c, w_active_fwd, w_active_rev,
                    timestamp / 1000000, timestamp / 1000);
            fclose(fp);
            rename(tmp_path, path);
        }
        
        // 精准控制秒级采集
        long sleep_time = 1000 - ((microtime() / 1000) % 1000) + 1;
        if (sleep_time > 0) {
            usleep(sleep_time * 1000);
        }
    }
    
    close(fd);
    return 0;
}

