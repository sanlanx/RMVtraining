// SerialPort.h  这是串口类的头文件，定义了一个串口了
#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

/*linux下串口需要使用到的头文件*/
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <glog/logging.h>

class SerialPort
{
public:
    SerialPort(char devname[100]);
    SerialPort(){}
    ~SerialPort();

    bool InitSerialPort(int BaudRate,
                        int DataBits,
                        int StopBits,
                        int ParityBit); // 初始化串口
    bool CloseSerialPort();             // 关闭串口

    int Write(char *Buff, const int Len); // 向串口写入数据
    int Read(char *Buff, const int Len);  // 从串口中读取数据
    void StartRead();  //  开启一个线程来循环读取
    void StartWrite(); // 开启一个 线程来循环写入
    int ReceiveFd(); // 获得fd的值，fd是打开串口设备后返回的文件描述符
private:
    static int m_BaudRateArr[]; // 波特率数组
    static int m_SpeedArr[];    // 波特率数组
    static char *m_DevName;     // 串口设备名称
    struct termios m_Setting;   // 串口配置的结构体

    int fd; // 打开串口设备后返回的文件描述符
};
#endif