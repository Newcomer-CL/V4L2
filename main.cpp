#include "v4l2.h"
#include <cstdlib> 

int main(int argc,char* argv[])
{
    if(argc!=5)
    {
        std::cout << "Usage:"<<argv[0]<< "[dev_path] [width] [height] [pixel_format]\n";
        return -1;
    }

    std::string dev_path = std::string(argv[1]);
    int height = std::atoi(argv[3]);
    int width = std::atoi(argv[2]);
    std::string pixel_format = std::string(argv[4]);

    // std::string dev_path = std::string("/dev/video0");
    // int height = 1080;
    // int width = 1920;
    // std::string pixel_format = std::string("MJPG");


    V4l2 v4l2(dev_path,pixel_format,width,height,4);
    v4l2.start_get_frame();
    cv::Mat image;
    int i = 0;
    while(1)
    {
        v4l2.output_frame(image);
        cv::imwrite("frame_" + std::to_string(i++) + ".jpg",image);
        
    }
    return 0;
    

}