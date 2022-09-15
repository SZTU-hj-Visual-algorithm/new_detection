#ifndef CV_DNN_DETECT_H
#define CV_DNN_DETECT_H

#include <opencv2/opencv.hpp>
#include "ArmorDetector.hpp"
#include <iostream>

using namespace cv;
#define NET_PATH "../my_module/best_80_1_16_954.onnx"
#define IMG_SIZE 16
#define TO_GRAY 1


class DNN_detect{
public:
    static void dnn_detect(Mat frame, Armor& armor);  // 调用该函数即可返回数字ID
private:
    static dnn::Net read_net(const String& net_path);
    static Mat img_processing(Mat ori_img, bool is_gray);
    static void net_forward(const Mat& blob, dnn::Net net, int& id, double & confidence);
};
#endif //CV_DNN_DETECT_H
