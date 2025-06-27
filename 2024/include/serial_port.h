#ifndef _Seral_H
#define _Seral_H
#pragma once //防止头文件重复包含
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>

#define FALSE -1
#define TRUE 0

class Seral_Send_ReaD;
union data
{
        char data_send[8];
        float data_need[2];
};
union data_read
{
        char data_read[8];
        double data_show[1];
};
class Seral_Send_ReaD
{
public:
        //Seral_Send_ReaD();
        //~Seral_Send_ReaD();

        ssize_t safe_write(const char *vptr, size_t n);
        ssize_t safe_read(char *vptr, size_t n);
        int uart_open(const char *pathname);
        int uart_set(int baude, int c_flow, int bits, char parity, int stop);
        int uart_read(char *r_buf, size_t len);
        int uart_write(const char *w_buf, size_t len);
        int uart_close();
        int UART0_Init(int speed, int flow_ctrl, int databits, int stopbits, int parity, const char *pathname);
        //int uart_send_two(float data1, float data2);

private:
        int fd;
};
void send_data(float Data, float depth);

double read_data();
#endif
