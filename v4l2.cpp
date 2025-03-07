#include "v4l2.h"

#include <fcntl.h>
#include <linux/videodev2.h>

#include <iostream>

V4l2::V4l2(std::string dev_path,std::string pixel_format, \
            uint pixel_width,uint pixel_height,int buf_count): \
            dev_path(dev_path),pixel_width(pixel_width), \
            pixel_height(pixel_height),camera_fd(0), \
            buf_count(buf_count),running(true)
{
    if(pixel_format == "NV12" || pixel_format == "nv12")
    {
        this->pixel_format = V4L2_PIX_FMT_NV12;
    }
    else if(pixel_format == "YUYV" || pixel_format == "yuyv") // YUYV 4:2:2
    {
        this->pixel_format = V4L2_PIX_FMT_YUYV;
    }
    else if(pixel_format == "MJPG" || pixel_format == "mjpg") // MJPG
    {
        this->pixel_format = V4L2_PIX_FMT_MJPEG; // MJPG
        std::cout << "MJPG\n"; 
        // MJPG是单平面
    }
    else if(pixel_format == "NV21" || pixel_format == "nv21") // nv21
    {
        this->pixel_format = V4L2_PIX_FMT_NV21; // NV21
    }
}


V4l2::~V4l2()
{

}

int V4l2::open_dev()
{
    this->camera_fd = open(this->dev_path.c_str(),O_RDWR);
    if (camera_fd < 0) {
        std::cout << "Failed to open device\n";
        return -1;
    }
    return 0;
}

int V4l2::query_dev_info()
{
    v4l2_capability capa;
    // 查询设备驱动信息
    if (ioctl(this->camera_fd, VIDIOC_QUERYCAP, &capa) == -1)
    {
        std::cout<<"VIDIOC_QUERYCAP failed\n";
        return -1;
    }
    //优先使用单平面帧格式采集
    if(capa.capabilities & V4L2_CAP_VIDEO_CAPTURE)
    {
        this->v4l2_buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    }
    //V4L2多平面API是为了满足一些设备的特殊要求(帧数据存储在不连续的缓冲区)
    else if(capa.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
    {
        this->v4l2_buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    }
    else
    {
        printf("The V4L2 device does not have the ability to capture videos!!!");
        return -1;
    }
    // printf("\nBasic Information:\n"
    //        "driver=%s\tcard=%s\t"
    //        "bus_info=%s\tversion=%d\tcapabilities=%x\n",
    //        capa.driver,capa.card,capa.bus_info,capa.version,capa.capabilities);
    return 0;
}

void V4l2::set_stream_format()
{
    v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = this->v4l2_buffer_type;
    if(this->v4l2_buffer_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)//1.多平面
    {
        //填充参数
        fmt.fmt.pix_mp.width = this->pixel_width;
        fmt.fmt.pix_mp.height = this->pixel_height;
        fmt.fmt.pix_mp.pixelformat = this->pixel_format;
        fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;//帧域
        /*planes相关参数应用层无需填写，由驱动层根据pixelformat自动设置，通过VIDIOC_G_FMT获取驱动设置得信息
        format.fmt.pix_mp.num_planes = 1;
        format.fmt.pix_mp.plane_fmt[0].bytesperline = width;
        format.fmt.pix_mp.plane_fmt[0].sizeimage = width*height*3/2;*/
        //设置格式
        ioctl(this->camera_fd,VIDIOC_S_FMT,&fmt);
        //获取格式(目的是为了获取该格式多平面的数量)
        ioctl(this->camera_fd,VIDIOC_G_FMT,&fmt);
        this->planes_num = fmt.fmt.pix_mp.num_planes;
        // printf("\nCurrent Mplane pixformat:\nnum_planes:%d\n",planes_num);
    }
    else//2.单平面
    {
        //填充参数
        fmt.fmt.pix.width = this->pixel_width;
        fmt.fmt.pix.height = this->pixel_height;
        fmt.fmt.pix.pixelformat = this->pixel_format;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;//帧域
        //设置格式
        ioctl(camera_fd,VIDIOC_S_FMT,&fmt);
    }
    usleep(100000);
    return ;
}

int V4l2::request_buf()
{
    v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = this->buf_count;                    // 缓冲帧数目 一般不低于3个，但太多的话按顺序刷新可能会造成视频延迟
    req.type = this->v4l2_buffer_type;           // 采集帧
    req.memory = V4L2_MEMORY_MMAP;    // 内存映射方式，省却数据拷贝的时间
    if (ioctl(camera_fd, VIDIOC_REQBUFS, &req) < 0) {     // 申请视频缓冲区
        std::cout << "Failed to request buffers\n";
        return -1;
    }
    return 0;
}

int V4l2::mmap_buffers()
{
    v4l2_buffer vbuffer;//视频缓冲帧
    for(int i = 0;i<this->buf_count;i++)
    {
        memset(&vbuffer,0,sizeof(vbuffer));
        vbuffer.index = i;//索引号
        vbuffer.type = this->v4l2_buffer_type;
        vbuffer.memory = V4L2_MEMORY_MMAP;
        //多平面
        if(this->v4l2_buffer_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
        {
            struct v4l2_plane m_planes[this->planes_num];
            memset(m_planes,0,sizeof(v4l2_plane)*this->planes_num);

            vbuffer.length = this->planes_num;
            vbuffer.m.planes = m_planes;
            //查询指定index的缓冲帧信息(缓冲帧在内核空间的长度和偏移量地址)
            ioctl(this->camera_fd,VIDIOC_QUERYBUF,&vbuffer);
            for (int j=0;j<this->planes_num;j++)
            {
                bufferMmapMplanePtr[i].length[j] = vbuffer.m.planes[j].length;
                bufferMmapMplanePtr[i].addr[j] = (unsigned char *)mmap(NULL,vbuffer.m.planes[j].length,
                                                                       PROT_READ,MAP_SHARED,this->camera_fd,
                                                                       vbuffer.m.planes[j].m.mem_offset);
                if (bufferMmapMplanePtr[i].addr[j] == MAP_FAILED)
                {
                    printf("mmap failed\n");
                    return false;
                }
            }
        }
        //单平面
        else
        {
            //查询指定index的缓冲帧信息(缓冲帧在内核空间的长度和偏移量地址)
            ioctl(this->camera_fd,VIDIOC_QUERYBUF,&vbuffer);
            /*记录缓存帧的长度及内存映射的地址*/
            bufferMmapPtr[i].length = vbuffer.length;
            bufferMmapPtr[i].addr = (unsigned char *)mmap(NULL,vbuffer.length,PROT_READ,MAP_SHARED,
                                                          this->camera_fd,vbuffer.m.offset);
            if(bufferMmapPtr[i].addr == MAP_FAILED)
            {
                printf("mmap failed.\n");
                return false;
            }
        }
    }
    return true;
}

void V4l2::start_video_stream()
{
    v4l2_buf_type type = (v4l2_buf_type)this->v4l2_buffer_type;

    // 放缓冲帧进输入队列
    v4l2_buffer vbuffer;
    for(int i = 0;i < buf_count;i++)
    {
        memset(&vbuffer,0,sizeof(vbuffer));
        vbuffer.index = i;
        vbuffer.type = this->v4l2_buffer_type;
        vbuffer.memory = V4L2_MEMORY_MMAP;
        if(this->v4l2_buffer_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
        {
            struct v4l2_plane m_planes[this->planes_num];
            memset(m_planes,0,sizeof(v4l2_plane)*this->planes_num);

            vbuffer.length = this->planes_num;
            vbuffer.m.planes = m_planes;
        }
        ioctl(this->camera_fd,VIDIOC_QBUF,&vbuffer);//缓冲帧放入视频输入队列 FIFO
    }
    ioctl(this->camera_fd,VIDIOC_STREAMON,&type);
}

void V4l2::worker()
{
    v4l2_buffer vbuffer;
    memset(&vbuffer,0,sizeof(vbuffer));
    vbuffer.type = this->v4l2_buffer_type;
    vbuffer.memory = V4L2_MEMORY_MMAP;
    if(this->v4l2_buffer_type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
    {
        struct v4l2_plane m_planes[this->planes_num];
        vbuffer.length = this->planes_num;
        vbuffer.m.planes = m_planes;
    }

    while(this->running)
    {

        //从视频输出队列取出一个缓冲帧
        if(ioctl(this->camera_fd,VIDIOC_DQBUF,&vbuffer) == -1)
        {
            printf("Dequeue buffers failed.\n");
            return ;
        }

        if(this->pixel_format == V4L2_PIX_FMT_NV12)
        {
            uchar *nv12_21FrameAddr = bufferMmapMplanePtr[vbuffer.index].addr[0];
            cv::Mat yuv(this->pixel_height / 2 * 3, this->pixel_width, CV_8UC1,
                        nv12_21FrameAddr);
            cv::Mat bgr;
            cv::cvtColor(yuv, bgr,
                      cv::COLOR_YUV2BGR_NV12);
            this->safe_queue_.enqueue(bgr);
        }
        else if(this->pixel_format == V4L2_PIX_FMT_YUYV)
        {
            uchar *nv12_21FrameAddr1 = bufferMmapPtr[vbuffer.index].addr;
            cv::Mat yuyv(this->pixel_height, this->pixel_width, CV_8UC2,
                nv12_21FrameAddr1);
            cv::Mat bgr;
            cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
            this->safe_queue_.enqueue(bgr);
        }
        else if(this->pixel_format == V4L2_PIX_FMT_MJPEG)
        {
            uchar *nv12_21FrameAddr1 = bufferMmapPtr[vbuffer.index].addr;
            int data_length = bufferMmapPtr[vbuffer.index].length;
            std::vector<uchar> encoded_data(nv12_21FrameAddr1, nv12_21FrameAddr1 + data_length);
            cv::Mat bgr = cv::imdecode(encoded_data, cv::IMREAD_COLOR);
            // cv::imwrite("frame_" + std::to_string(i++) + ".jpg",bgr);
            // cv::Mat yuv_data;
            // I420不是NV12
            // cv::cvtColor(decoded_image, yuv_data, cv::COLOR_BGR2YUV_I420);
            this->safe_queue_.enqueue(bgr);
        }
        else if(this->pixel_format == V4L2_PIX_FMT_NV21) // nv21
        {
            uchar *nv12_21FrameAddr = bufferMmapMplanePtr[vbuffer.index].addr[0];
            cv::Mat yuv(this->pixel_height / 2 * 3, this->pixel_width, CV_8UC1,
                        nv12_21FrameAddr);
            cv::Mat bgr;
            cv::cvtColor(yuv, bgr,
                      cv::COLOR_YUV2BGR_NV21);
            this->safe_queue_.enqueue(bgr);
        }
        ioctl(this->camera_fd,VIDIOC_QBUF,&vbuffer);
    }
}

int V4l2::output_frame(cv::Mat& frame)
{
    int ret = this->safe_queue_.dequeue(frame);
    if(ret != 1 )
    {
        std::cout << "队列为空，没有图像可取！\n";
        return -1;
    }
    return 0;
}

int V4l2::get_frame_num()
{
    return this->safe_queue_.size();
}

void V4l2::start_get_frame()
{
    this->open_dev();
    this->query_dev_info();
    this->set_stream_format();
    this->request_buf();
    this->mmap_buffers();
    this->start_video_stream();
    this->work_thread = std::thread(&V4l2::worker, this);;
    this->work_thread.detach();
    return ;
}