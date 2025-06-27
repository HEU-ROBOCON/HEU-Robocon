#include "include/serial_port.h"
#include <iostream>
//定义结构体---坐标结构体
//using namespace cv;
//using namespace std;
static union
{
        char Receive_Val[24];
        float Act_val[6];
} uart_posture;

int ret;
int fd;

const char name[] = "/dev/ttyUSB0"; //因为UART0_Init()的最后一个参数是const 所以定义一个const数组
char head[2] = {0x0a, 0x0d};
char tail[2] = {0x0d, 0x0a};

char head1[1];
char head2[1];
char tail1[2];
int Init_ = false;
Seral_Send_ReaD serial_port; //USB send data

void send_data(float Data, float depth)
{
        if (Init_ == false)
        {
                serial_port.UART0_Init(115200, 0, 8, 1, 0, name);
                Init_ = true;
        }
        data send;
        send.data_need[0] = Data;
        send.data_need[1] = depth;
        serial_port.uart_write(head, 2);
        serial_port.uart_write(send.data_send, 8);
        serial_port.uart_write(tail, 2);
}
double read_data()
{
        if (Init_ == false)
        {
                serial_port.UART0_Init(115200, 0, 8, 1, 0, name);
                Init_ = true;
        }
        data_read need;
        need.data_show[0] = 0.0;
        //返回读到的字节数
        int key = 0;
        char key_tail;
        for (int j = 0;; j++)
        {
                if (j == 0)
                {
                        while (true)
                        {
                                int key = serial_port.uart_read(head1, 1);
                                if (key == 1 && head1[0] == 10)
                                        break;
                        }
                        serial_port.uart_read(head2, 1);
                        key_tail = head2[0];
                }
                else
                {
                        head1[0] = key_tail;
                        serial_port.uart_read(head2, 1);
                        key_tail = head2[0];
                }
                if (head1[0] == 10 && key_tail == 14)
                {
                        break;
                }
        }
        serial_port.uart_read(need.data_read, 8);
        serial_port.uart_read(tail1, 2);
        if (tail1[0] == 14 && tail1[1] == 15)
        {
                return need.data_show[0];
        }
        return 0;
}
/*
 * 安全读写函数
 */

ssize_t Seral_Send_ReaD::safe_write(const char *vptr, size_t n)
{
        size_t nleft;
        ssize_t nwritten;
        const char *ptr;

        ptr = vptr;
        nleft = n;

        while (nleft > 0)
        {
                if ((nwritten = write(fd, ptr, nleft)) <= 0)
                {
                        if (nwritten < 0 && errno == EINTR)
                                nwritten = 0;
                        else
                                return -1;
                }
                nleft -= nwritten;
                ptr += nwritten;
        }
        return (n);
}

ssize_t Seral_Send_ReaD::safe_read(char *vptr, size_t n)
{
        size_t nleft;
        ssize_t nread;
        char *ptr;

        ptr = vptr;
        nleft = n;

        while (nleft > 0)
        {
                if ((nread = read(fd, ptr, nleft)) < 0)
                {
                        if (errno == EINTR) //被信号中断
                                nread = 0;
                        else
                                return -1;
                }
                else if (nread == 0)
                        break;
                nleft -= nread;
                ptr += nread;
        }
        return (n - nleft);
}
/*******************************************************************
* 名称：                  UART0_Open
* 功能：                打开串口并返回串口设备文件描述
* 入口参数：        fd    :文件描述符     pathname :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int Seral_Send_ReaD::uart_open(const char *pathname)
{
        assert(pathname);

        /*打开串口*/
        fd = open(pathname, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == FALSE)
        {
                perror("Open UART failed!");
                return FALSE;
        }

        /*清除串口非阻塞标志*/
        if (fcntl(fd, F_SETFL, 0) < 0)
        {
                fprintf(stderr, "fcntl failed!\n");
                return FALSE;
        }
        return fd;
}

/*******************************************************************
* 名称：                UART0_Set
* 功能：                设置串口数据位，停止位和效验位
* 入口参数：					   fd        串口文件描述符
*                              baude     波特率
*                              c_flow    数据流控制
*							   bits      数据位   取值为 7 或者8
*							   parity    效验类型 取值为N,E,O,,S
*                              stop      停止位   取值为 1 或者2
*                              
*出口参数：          正确返回为1，错误返回为0
*******************************************************************/
int Seral_Send_ReaD::uart_set(int baude, int c_flow, int bits, char parity, int stop)
{
        struct termios options;

        /*获取终端属性*/
        if (tcgetattr(fd, &options) < 0)
        {
                perror("tcgetattr error");
                return FALSE;
        }

        /*设置输入输出波特率，两者保持一致*/
        switch (baude)
        {
        case 0:
                cfsetispeed(&options, B0);
                cfsetospeed(&options, B0);
                break;
        case 2400:
                cfsetispeed(&options, B2400);
                cfsetospeed(&options, B2400);
                break;
        case 4800:
                cfsetispeed(&options, B4800);
                cfsetospeed(&options, B4800);
                break;
        case 9600:
                cfsetispeed(&options, B9600);
                cfsetospeed(&options, B9600);
                break;
        case 115200:
                cfsetispeed(&options, B115200);
                cfsetospeed(&options, B115200);
                break;
        case 19200:
                cfsetispeed(&options, B19200);
                cfsetospeed(&options, B19200);
                break;
        case 38400:
                cfsetispeed(&options, B38400);
                cfsetospeed(&options, B38400);
                break;
        default:
                fprintf(stderr, "Unkown baude!\n");
                return FALSE;
        }

        /*设置控制模式*/
        options.c_cflag |= CLOCAL; //保证程序不占用串口
        options.c_cflag |= CREAD;  //保证程序可以从串口中读取数据

        /*设置数据流控制*/
        switch (c_flow)
        {
        case 0: //不进行流控制
                options.c_cflag &= ~CRTSCTS;
                break;
        case 1: //进行硬件流控制
                options.c_cflag |= CRTSCTS;
                break;
        case 2: //进行软件流控制
                options.c_cflag |= IXON | IXOFF | IXANY;
                break;
        default:
                fprintf(stderr, "Unkown c_flow!\n");
                return -1;
        }

        /*设置数据位*/
        options.c_cflag &= ~CSIZE; //屏蔽其它标志位
        switch (bits)
        {
        case 5:
                options.c_cflag |= CS5;
                break;
        case 6:
                options.c_cflag |= CS6;
                break;
        case 7:
                options.c_cflag |= CS7;
                break;
        case 8:
                options.c_cflag |= CS8;
                break;
        default:
                fprintf(stderr, "Unkown bits!\n");
                return FALSE;
        }

        /*设置校验位*/
        switch (parity)
        {
        /*无奇偶校验位*/
        case 'n':
        case 'N':
                options.c_cflag &= ~PARENB; //PARENB：产生奇偶位，执行奇偶校验
                //options.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用//lzz
                break;
        /*设为空格,即停止位为2位*/
        case 's':
        case 'S':
                options.c_cflag &= ~PARENB; //PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag &= ~CSTOPB; //CSTOPB：使用两位停止位
                break;
        /*设置奇校验*/
        case 'o':
        case 'O':
                options.c_cflag |= PARENB; //PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag |= PARODD; //PARODD：若设置则为奇校验,否则为偶校验
                options.c_cflag |= INPCK;  //INPCK：使奇偶校验起作用
                options.c_cflag |= ISTRIP; //ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
                break;
        /*设置偶校验*/
        case 'e':
        case 'E':
                options.c_cflag |= PARENB;  //PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag &= ~PARODD; //PARODD：若设置则为奇校验,否则为偶校验
                options.c_cflag |= INPCK;   //INPCK：使奇偶校验起作用
                options.c_cflag |= ISTRIP;  //ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
                break;
        default:

                options.c_cflag &= ~PARENB; //PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag &= ~INPCK;  //INPCK：使奇偶校验起作用

                fprintf(stderr, "Unkown parity!\n");
                //return -1;
        }

        /*设置停止位*/
        switch (stop)
        {
        case 1:
                options.c_cflag &= ~CSTOPB; //CSTOPB：使用两位停止位
                break;
        case 2:
                options.c_cflag |= CSTOPB; //CSTOPB：使用两位停止位
                break;
        default:
                fprintf(stderr, "Unkown stop!\n");
                return -1;
        }

        /*设置输出模式为原始输出*/
        options.c_oflag &= ~OPOST; //OPOST：若设置则按定义的输出处理，否则所有c_oflag失效
        /*设置本地模式为原始模式*/
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        //options.c_lflag &= ~(ISIG | ICANON);
        /*
     *ICANON：允许规范模式进行输入处理
     *ECHO：允许输入字符的本地回显
     *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
     *ISIG：允许信号
     */

        /*设置等待时间和最小接受字符*/
        options.c_cc[VTIME] = 0; //可以在select中设置 // 读取一个字符等待0*(1/10)s
        options.c_cc[VMIN] = 1;  //最少读取一个字符

        /*如果发生数据溢出，只接受数据，但是不进行读操作*/
        tcflush(fd, TCIFLUSH);

        /*激活配置*/
        if (tcsetattr(fd, TCSANOW, &options) < 0)
        {
                perror("tcsetattr failed");
                return FALSE;
        }

        return TRUE;
}

int Seral_Send_ReaD::uart_read(char *r_buf, size_t len)
{
        ssize_t cnt = 0;
        fd_set rfds;
        struct timeval time;

        /*将文件描述符加入读描述符集合*/
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        /*设置超时为15s*/
        time.tv_sec = 5;
        time.tv_usec = 0;

        /*实现串口的多路I/O*/
        ret = select(fd + 1, &rfds, NULL, NULL, &time);
        switch (ret)
        {
        case -1:
                fprintf(stderr, "select error!\n");
                return -1;
        case 0:
                fprintf(stderr, "time over!\n");
                return -1;
        default:
                cnt = safe_read(r_buf, len);
                if (cnt == -1)
                {
                        fprintf(stderr, "read error!\n");
                        return FALSE;
                }
                return cnt;
        }
}

int Seral_Send_ReaD::uart_write(const char *w_buf, size_t len)
{
        ssize_t cnt = 0;

        cnt = safe_write(w_buf, len);
        if (cnt == -1)
        {
                fprintf(stderr, "write error!\n");
                return FALSE;
        }

        return cnt;
}
int Seral_Send_ReaD::UART0_Init(int speed, int flow_ctrl, int databits, int stopbits, int parity, const char *pathname)
{
        int err;
        //设置串口数据帧格式
        fd = uart_open(pathname);
        if (Seral_Send_ReaD::uart_set(speed, flow_ctrl, databits, 'N', stopbits) == FALSE)
        {
                return FALSE;
        }
        else
        {
                return TRUE;
        }
}

// int  Seral_Send_ReaD::uart_send_two(float data1, float data2)
// {
// 	uart_posture.Act_val[0] = data1;
// 	uart_posture.Act_val[1] = data2;

// 	char sendData[8];
// 	for (size_t i = 0; i < 8; i++) {
// 		sendData[i] = uart_posture.Receive_Val[i];
// 	}
// 	uart_write(sendData, 8);

// }
int Seral_Send_ReaD::uart_close()
{
        assert(fd);
        close(fd);

        /*可以在这里做些清理工作*/

        return TRUE;
}
