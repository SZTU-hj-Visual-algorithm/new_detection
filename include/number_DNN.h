#include <opencv2/opencv.hpp>
#include <iostream>


//namespace robot_detection {

class DNN_detect{
    std::string net_path;
    int input_width;
    int input_height;
    cv::dnn::Net net;
public:
    DNN_detect();
    cv::Mat img_processing(cv::Mat ori_img);
    void net_forward(const cv::Mat& blob, int& id, double & confidence);
};

//}