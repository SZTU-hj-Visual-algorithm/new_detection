#include <opencv2/opencv.hpp>
#include <iostream>

#define INPUT_WIDTH 20
#define INPUT_HEIGHT 30
#define TO_GRAY 1
#define THRESH_CONFIDENCE 0.783

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