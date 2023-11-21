#include "SerialPort/SerialPort.h"
#include <iostream>

unsigned char rdata[255];
unsigned char Sdata[255];
static double serialtimer, timerlast;

/// NAME    :CRC8/MAXIM
/// POLY    :(0x31)->x8 + x5 + x4 + 1
/// WIDTH   :8
/// INIT    :00
/// REFIN   :true
/// REFOUT  :true
/// XOROUT  :00
const uint8_t CRC8_INIT     = 0xff;
const uint8_t CRC8_TAB[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
/**
 * @brief crc8_check 计算crc校验位
 * @param addr 首位地址
 * @param len 数据长度
 * @return
 */
uint8_t Uart::crc8_check(uint8_t* addr, int len)
{
    uint8_t data;
    uint8_t crc = CRC8_INIT;  //初始值

    for (; len > 0; len--)
    {
        data = *addr++;
        crc  = crc ^ data;     //与crc初始值异或
        crc  = CRC8_TAB[crc];  //替换下面for循环
    }

    return crc;  //返回最终校验值
}
/**
 * @brief 串口发送数据
 */
void Uart::TransformTarPos(int& fd, const HostComputerData& data)
{
    Sdata[0]  = 0x5A;
    Sdata[1]  = data.Pitch.c[0];
    Sdata[2]  = data.Pitch.c[1];
    Sdata[3]  = data.Pitch.c[2];
    Sdata[4]  = data.Pitch.c[3];
    Sdata[5]  = data.Yaw.c[0];
    Sdata[6]  = data.Yaw.c[1];
    Sdata[7]  = data.Yaw.c[2];
    Sdata[8]  = data.Yaw.c[3];
    Sdata[9]  = data.distance.c[0];
    Sdata[10] = data.distance.c[1];
    Sdata[11] = data.distance.c[2];
    Sdata[12] = data.distance.c[3];
    // Sdata[9]  = data.if_armor_exist;
    Sdata[13] = data.if_shoot;
    Sdata[14] = data.if_real_shoot;
    /// CRC check
    //    Sdata[10] = crc8_check(&Sdata[1], 9);
//    Sdata[14] = crc8_check(&Sdata[1], 13);
    Sdata[15]=0Xa5;
    write(fd, Sdata, 16);
}

/**
 * @brief 串口接受数据
 */
void Uart::GetMode(int& fd, GroundChassisData& data)
{
    int    i = 0;
    size_t bytes;
    ioctl(fd, FIONREAD, &bytes);
    if (bytes == 0)
        return;
    bytes = read(fd, rdata, 14);
    // crc8_check(&rdata[1], 9);
    for (int i = 0; i < 13; i++)
    {
        if (rdata[i] == 0XAA)
        {
            break;
        }
    }
    if (rdata[i] == 0XAA && rdata[i + 12] == 0xBB /*crc8_check(&rdata[i + 1], 11)*/)  //&& Verify_CRC8_Check_Sum(rdata,6)
    {

        ///< pitch
        data.gain_pitch.c[0] = rdata[i + 1];
        data.gain_pitch.c[1] = rdata[i + 2];
        data.gain_pitch.c[2] = rdata[i + 3];
        data.gain_pitch.c[3] = rdata[i + 4];

        ///< yaw
        data.gain_yaw.c[0] = rdata[i + 5];
        data.gain_yaw.c[1] = rdata[i + 6];
        data.gain_yaw.c[2] = rdata[i + 7];
        data.gain_yaw.c[3] = rdata[i + 8];

        data.mode  = rdata[i + 9];
        data.color = rdata[i + 10];
        data.speed = rdata[i + 11];

        // data.selfid = rdata[i + 12];
    }
}

Uart::Uart(int speed) {}
/**
 *@brief 设置串口通信速率
 *@param fd 类型 int 打开串口的文件句柄
 *@param speed 类型 int 串口速度
 *@return void
 */
// static int
speed_t speed_arr[] = {
    B921600, B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300, B38400, B19200, B9600, B4800, B2400, B1200, B300,
};
speed_t name_arr[] = {
    921600, 115200, 38400, 19200, 9600, 4800, 2400, 1200, 300, 38400, 19200, 9600, 4800, 2400, 1200, 300,
};
void Uart::set_speed(int fd, speed_t speed)
{
    int            i;
    int            status;
    struct termios Opt;
    tcgetattr(fd, &Opt);
    for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);  //
            if (status != 0)
            {
                perror("tcsetattr fd1");
                return;
            }
            tcflush(fd, TCIOFLUSH);
        }
    }
}

/**
 *@brief 设置串口数据位，停止位和效验位
 *@param fd 类型 int 打开的串口文件句柄
 *@param databits 类型 int 数据位 取值 为 7 或者8
 *@param stopbits 类型 int 停止位 取值为 1 或者2
 *@param parity 类型 int 效验类型 取值为N,E,O,S
 */
int Uart::set_Parity(int fd, int databits, int stopbits, int parity)
{
    struct termios options;

    if (tcgetattr(fd, &options) != 0)
    {
        perror("SetupSerial 1");
        return (FALSE);
    }

    options.c_cflag &= ~static_cast<unsigned int>(CSIZE);
    switch (databits) /*设置数据位数*/
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "Unsupported data sizen");
        return (FALSE);
    }

    switch (parity)
    {
    case 'N':
    {
        options.c_cflag &= ~static_cast<unsigned int>(PARENB); /* Clear parity enable */
        options.c_iflag &= ~static_cast<unsigned int>(INPCK);  /* Enable parity checking */
    }
    break;

    case 'O':
    {
        options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
        options.c_iflag |= INPCK;             /* Disnable parity checking */
    }
    break;

    case 'E':
    {
        options.c_cflag |= PARENB;                             /* Enable parity */
        options.c_cflag &= ~static_cast<unsigned int>(PARODD); /* 转换为偶效验*/
        options.c_iflag |= INPCK;                              /* Disnable parity checking */
    }
    break;

    case 'S': /*as no parity*/
    {
        options.c_cflag &= ~static_cast<unsigned int>(PARENB);
        options.c_cflag &= ~static_cast<unsigned int>(CSTOPB);
    }
    break;

    default:
    {
        fprintf(stderr, "Unsupported parityn");
        return (FALSE);
    }
    }
    /* 设置停止位*/
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~static_cast<unsigned int>(CSTOPB);
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported stop bitsn");
        return (FALSE);
    }
    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;

    options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
    options.c_cc[VMIN]  = 0;   /* Update the options and do it NOW */

    options.c_cflag &= ~static_cast<unsigned int>(HUPCL);
    options.c_iflag &= ~static_cast<unsigned int>(INPCK);
    options.c_iflag |= static_cast<unsigned int>(IGNBRK);
    options.c_iflag &= ~static_cast<unsigned int>(ICRNL);
    options.c_iflag &= ~static_cast<unsigned int>(IXON);
    options.c_lflag &= static_cast<unsigned int>(IEXTEN);
    options.c_lflag &= ~static_cast<unsigned int>(ECHOK);
    options.c_lflag &= ~static_cast<unsigned int>(ECHOCTL);
    options.c_lflag &= ~static_cast<unsigned int>(ECHOKE);
    options.c_lflag &= ~static_cast<unsigned int>(ONLCR);
    options.c_oflag = ~static_cast<unsigned int>(ICANON);

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("SetupSerial 3");
        return (FALSE);
    }
    return (TRUE);
}

void Uart::close_serial(int fd)
{
    close(fd);
}

/**
 *@brief 串口初始化、获取设备号
 */
int Uart::Init_serial(int& fdcom, int baud_speed)
{
    if (open("/dev/ttyACM0", O_RDWR | O_NONBLOCK) == -1)
    {
        fdcom = open("/dev/ttyACM1", O_RDWR | O_NONBLOCK);  // O_NOCTTY | O_NDELAY
    }
    else
    {
        fdcom = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK);  // O_NOCTTY | O_NDELAY
    }
    if (-1 == fdcom)
    {
        perror(" Can''t Open Serial Port\n");
        return -1;
    }
    else
        perror(" Open Serial Port success\n");
    set_speed(fdcom, baud_speed);
    if (set_Parity(fdcom, 8, 1, 'N') == FALSE)
    {
        printf("Set Parity Errorn\n");
        exit(0);
    }
    return 0;
}
