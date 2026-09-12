#include "MessageManager.hpp"
#include "globalParam.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <string>
inline void crc16_update(uint16_t *currectCrc,
                         char *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j = 0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t tmp = crc << 1;
            if (crc & 0x8000)
            {
                tmp ^= 0x1021;
            }
            crc = tmp;
        }
    }
    *currectCrc = crc;
}
MessageManager::MessageManager(GlobalParam &gp)
{

    this->last_message = Translator();
    this->message_hold_threshold = gp.message_hold_threshold;
    this->loss_cnt = this->message_hold_threshold;
    this->gp = &gp;
    this->now_time = 0;
    this->n_time = 0;
    this->start_time = 0;
    std::string video_address = "../video/v4.avi";
#ifdef VIRTUALGRAB
    this->capture.open(video_address);
    this->totalFrames = capture.get(cv::CAP_PROP_FRAME_COUNT);
    // std::cout<<totalFrames<<std::endl;
    this->currentFrames = 0;
#endif
#ifdef RECORDVIDEO
    // std::string output_address = "../video/";
    // int idx = 1;
    // int coder = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    // this->vw = new cv::VideoWriter(output_address + std::to_string(idx) + ".MP4", coder, 25.0, cv::Size(1080, 720), true);
#endif
}
MessageManager::~MessageManager()
{
    vw->release();
    capture.release();
    delete this->vw;
}
void MessageManager::HoldMessage(Translator &ts)
{
    if (ts.message.crc == 0)
    {
        if (loss_cnt < message_hold_threshold)
        {
            loss_cnt += 1;
            ts.message = last_message.message;
            ts.message.crc = 0;
        }
    }
    else
    {
        loss_cnt = 0;
        last_message.message = ts.message;
    }
    return;
}

void MessageManager::FakeMessage(Translator &ts)
{
    ts.message.pitch = gp->fake_pitch;
    ts.message.yaw = fmod(gp->fake_yaw, 2 * M_PI);
    ts.message.status = gp->fake_status;
    if (gp->attack_mode == ARMOR)
        this->now_time += 50;
    else if (gp->attack_mode == ENERGY)
        this->now_time += 50;
    MessageManager::UpdateCrc(ts, 61);
    return;
}
void MessageManager::ChangeBigArmor(Translator &ts)
{
    gp->armorStat = ts.message.armor_flag;
    int tmp = gp->armorStat;
    for (int i = 5; i > 2; i--)
    {
        gp->isBigArmor[i] = tmp % 2;
        tmp /= 2;
    }
}
void MessageManager::read(Translator &ts, SerialPort &serialPort)
{
    int len = 0;
#ifndef NOPORT
    int count=0;
    Translator temp_ts;  // 使用临时变量读取
    // temp_ts.status = 5;
    // 开始读取串口信息，存储在临时文件中，这样可以快速反复读取，避免串口中信息来不及读取导致堆积
    while (1)
    {
        if (count>120) std::exit(-1);
        // 注意：Translator.data 大小为64字节，不能读取超过64字节
        len = serialPort.Read(temp_ts.data, 64);
        
        if (len == 64)
        {
            usleep(100);
            break;
        }
#ifdef THREADANALYSIS
    printf("len1 is %d\n", len);
    printf("status1 is %d\n", temp_ts.message.status);
#endif
        if (len == -1 || len == 0) std::exit(-1); //csy 7_24 new added
        count++;
       // //std::cout << "wei: " << ts.messageWM.bullet_v << std::endl;
        usleep(1000);
    }
    // 只在成功读取后才加锁复制数据
    this->message_lock.lock();
    ts = temp_ts;
    this->message_lock.unlock();
    // 如果长度为-1，为error，也就是串口连接出现问题，退出程序，并依靠外部的脚本使程序重新启动

#endif // NOPORT
}
void MessageManager::write(Translator &ts, SerialPort &serialPort)
{
    ts.message.head = 0x71;
    ts.message.tail = 0x4C;
    // 如果使用串口，输出，假如失败，输出日志：写失败
    if (!serialPort.Write(ts.data, 64))
    {
        LOG_IF(INFO, gp->switch_INFO) << "write failed";
    }
}
int MessageManager::CheckCrc(Translator &ts, int len)
{

    // 设置crc校验相关内容
    uint16_t nowCrc = ts.message.crc;
    uint16_t currectCrc = 0;
    uint16_t *crcPoint = &currectCrc;
    crc16_update(crcPoint, ts.data, len);
    if (currectCrc == nowCrc)
        return 1;
    else
        return 0;
}
void MessageManager::UpdateCrc(Translator &ts, int len)
{
    // 设置crc校验
    ts.message.crc = 0;
    crc16_update(&ts.message.crc, ts.data, len);
}
void MessageManager::copy(Translator &src, Translator &dst)
{
    this->message_lock.lock();
    dst = src;
    this->message_lock.unlock();
}
void MessageManager::initParam(int color)
{

    // 如果颜色是红色，gp读取红色对应的参数
    if (color == RED)
        gp -> initGlobalParam(RED);
    // 如果颜色是蓝色，gp读取蓝色对应的参数
    else if (color == BLUE)
        gp ->initGlobalParam(BLUE);
}
void MessageManager::getFrame(cv::Mat &pic, Translator translator)
{
    capture >> pic;
    // 当前帧数自加，用于记录当前的帧数，用于判断是否视频被播放完毕
    currentFrames++;

    // 假如说视频被播放完毕，设置位置到上一次
    if (currentFrames == totalFrames - 1)
    {

        currentFrames = 0;
        capture.set(cv::CAP_PROP_POS_FRAMES, 0);
    }
}
void MessageManager::recordFrame(cv::Mat &pic)
{
    cv::Mat temp = pic.clone();
    cv::resize(temp, temp, cv::Size(1080, 720));
    vw->write(temp);
    // static int delete_bad=1;
    static int frame_accum = 0;
    if (frame_accum > 1000)
    {
        // start_time = n_time;
        frame_accum = 0;
        vw->release();
        delete vw;
        std::string output_address = "../video/";
        int idx = 1;
        int coder = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        this->vw = new cv::VideoWriter(output_address + std::to_string(idx) + ".MP4", coder, 25.0, cv::Size(1080, 720), true);
    }
    frame_accum++;
}
void MessageManager::ReadLogMessage(Translator &ts,GlobalParam &gp)
{
    LOG_IF(INFO, gp.switch_INFO) << "read successful";
    LOG_IF(INFO, gp.switch_INFO) << "当前pitch: " << ts.message.pitch;
    LOG_IF(INFO, gp.switch_INFO) << "当前yaw: " << ts.message.yaw;
    LOG_IF(INFO, gp.switch_INFO) << "当前状态: " << +ts.message.status;
}
void MessageManager::WriteLogMessage(Translator &ts,GlobalParam &gp)
{
    LOG_IF(INFO, gp.switch_INFO) << "write successful";
    LOG_IF(INFO, gp.switch_INFO) << "当前crc: " << ts.message.crc;
}