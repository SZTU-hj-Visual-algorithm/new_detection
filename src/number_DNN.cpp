#include "number_dnn.h"

using namespace cv;
using namespace std;

//namespace robot_detection {
DNN_detect::DNN_detect()
{
    FileStorage fs("../other/dnn_data.yaml", FileStorage::READ);
    net_path = (string)fs["net_path"];
    input_width = (int)fs["input_width"];
    input_height = (int)fs["input_height"];
    net = dnn::readNetFromONNX(net_path);
//    net.setPreferableTarget(dnn::dnn4_v20211004::DNN_TARGET_CUDA_FP16);
//    net.setPreferableBackend(dnn::dnn4_v20211004::DNN_BACKEND_CUDA);
    fs.release();
}

Mat DNN_detect::img_processing(Mat ori_img) {
    Mat out_blob;
    cvtColor(ori_img, ori_img, cv::COLOR_RGB2GRAY);
    threshold(ori_img, ori_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    dnn::blobFromImage(ori_img, out_blob, 1.0f/255.0f, Size(input_width, input_height));
    return out_blob;
}

void DNN_detect::net_forward(const Mat& blob, int& id, double& confidence) {
    net.setInput(blob);
    Mat outputs = net.forward();
    cv::Point class_id;
    minMaxLoc(outputs, nullptr, &confidence, nullptr, &class_id);
    if(class_id.x)id = class_id.x;
}

//}