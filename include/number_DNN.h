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
    void img_processing(cv::Mat ori_img, std::vector<cv::Mat>& numROIs);
    cv::Mat net_forward(const std::vector<cv::Mat>& numROIs);
};

//}