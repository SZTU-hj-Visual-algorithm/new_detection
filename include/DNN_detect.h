#ifndef CV_DNN_DETECT_H
#define CV_DNN_DETECT_H

#include <opencv2/opencv.hpp>
#include <iostream>

#define NET_PATH "../module/1_2.0_16_7.onnx"
#define IMG_SIZE 16
#define TO_GRAY 1



class DNN_detect{
public:
    static cv::dnn::Net read_net(const std::string& net_path);
    static cv::Mat img_processing(cv::Mat ori_img, bool is_gray);
    static void net_forward(const cv::Mat& blob, cv::dnn::Net net, int& id, double & confidence);
};
#endif //CV_DNN_DETECT_H
