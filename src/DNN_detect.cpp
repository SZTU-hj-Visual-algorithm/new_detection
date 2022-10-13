#include "DNN_detect.h"

using namespace cv;

dnn::Net DNN_detect::read_net(const std::string& net_path) {
    return dnn::readNetFromONNX(net_path);
}

Mat DNN_detect::img_processing(Mat ori_img, bool to_gray) {
    Mat out_blob;
//    if(to_gray) {
//        cvtColor(ori_img, ori_img, COLOR_BGR2GRAY);
//        ori_img.convertTo(ori_img, CV_32FC1, 1.0f / 255.0f);
//    } else{ori_img.convertTo(ori_img, CV_32FC3, 1.0f / 255.0f);}

    cvtColor(ori_img, ori_img, cv::COLOR_RGB2GRAY);
    threshold(ori_img, ori_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    ori_img.convertTo(ori_img,CV_32FC1,1.0f / 255.0f);
    dnn::blobFromImage(ori_img, out_blob, 1.0, Size(INPUT_WIDTH, INPUT_HEIGHT));
    return out_blob;
}

void DNN_detect::net_forward(const Mat& blob, dnn::Net net, int& id, double& confidence) {
    net.setInput(blob);
    Mat outputs = net.forward();
    cv::Mat softmax_prob;
    cv::exp(outputs, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;
    cv::Point class_id;
    minMaxLoc(softmax_prob, nullptr, &confidence, nullptr, &class_id);
    if(class_id.x)
    id = class_id.x;
}