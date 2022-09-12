#ifndef CV_DNN_DETECT_H
#define CV_DNN_DETECT_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
#define NET_PATH "../my_module/best_80_1_16_954.onnx"
#define IMG_SIZE 16
#define TO_GRAY 1


class DNN_detect{
public:
    static int dnn_detect(Mat frame);  // 调用该函数即可返回数字ID
    static int dnn_detect(const String& img_path);
private:
    static dnn::Net read_net(const String& net_path);
    static Mat img_processing(Mat ori_img, bool is_gray);
    static int net_forward(const Mat& blob, dnn::Net net);
};
#endif //CV_DNN_DETECT_H
