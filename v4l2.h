#ifndef V4L2_H_
#define V4L2_H_

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include <thread>
#include <string>
#include <atomic>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "safe_queue.h"

/*缓存帧内存映射信息*/
struct BufferMmap//单平面
{
    uchar * addr = NULL;//缓冲帧映射到内存中的起始地址
    uint length = 0;//缓冲帧映射到内存中的长度
};


struct BufferMmapMplane//多平面
{
    uchar * addr[VIDEO_MAX_PLANES] = {NULL};//缓冲帧(每个平面)映射到内存中的起始地址
    uint length[VIDEO_MAX_PLANES] = {0};//缓冲帧(每个平面)映射到内存中的长度
};


class V4l2{

public:
    V4l2(std::string dev_path,std::string pixel_format, \
            uint pixel_width,uint pixel_height,int buf_count);
    ~V4l2();
    void start_get_frame();
    int output_frame(cv::Mat& frame);
    int get_frame_num();
private:
    int open_dev();
    int query_dev_info();
    void set_stream_format();
    int request_buf();
    int mmap_buffers();
    void start_video_stream();
    void worker();
    
    int camera_fd;
    std::string dev_path;
    int v4l2_buffer_type;
    int planes_num;
    uint pixel_format;
    uint pixel_width;
    uint pixel_height;
    int buf_count;
    BufferMmap bufferMmapPtr[4];
    BufferMmapMplane bufferMmapMplanePtr[4];
    std::thread work_thread;
    SafeQueue<cv::Mat> safe_queue_;
    std::atomic<bool> running; // 控制线程的运行状态
};






#endif    // V4L2_H_