#include "DNN_detect.h"

int DNN_detect::dnn_detect(Mat frame) {
    return net_forward(img_processing(std::move(frame), TO_GRAY), read_net(NET_PATH));
}

int DNN_detect::dnn_detect(const String& imgpath) {
    Mat frame = imread(imgpath);
    return net_forward(img_processing(frame, TO_GRAY), read_net(NET_PATH));
}

dnn::Net DNN_detect::read_net(const String& net_path) {
    return dnn::readNetFromONNX(net_path);
}

Mat DNN_detect::img_processing(Mat ori_img, bool to_gray) {
    Mat out_blob;
    if(to_gray) {cvtColor(ori_img, ori_img, COLOR_BGR2GRAY);}
    ori_img.convertTo(ori_img, CV_32FC1, 1.0f / 255.0f);
    dnn::blobFromImage(ori_img, out_blob, 1.0, Size(IMG_SIZE, IMG_SIZE));
    return out_blob;
}

int DNN_detect::net_forward(const Mat& blob, dnn::Net net) {
    net.setInput(blob);
    Mat output = net.forward();
    Point minLoc, maxcLoc;
    double min, max;
    cv::minMaxLoc(output, &min, &max, &minLoc, &maxcLoc);
    return maxcLoc.x;
}
