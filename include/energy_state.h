#include <opencv2/opencv.hpp>
#include "robot_status.h"
#include <Eigen/Dense>
//using namespace cv;
using namespace std;

struct energy_inf
        {
    cv::Point2f re_aim;
    cv::RotatedRect c_rect;

        };

class energy:public robot_state
        {
    cv::Mat l_hit;
    cv::Mat r_hit;
    cv::Mat l_uhit;
    cv::Mat r_uhit;
    double energy_threshold = 0.65;
    int thresh = 37;
    int thres_red = 34;
    int thres_blue = 60;
        public:
            energy();
            cv::Point2f dst_p[4] = {cv::Point2f(0,0),cv::Point2f(0,30),cv::Point2f(60,30),cv::Point2f(60,0)};
            cv::Mat src;
            cv::Mat center_mat;
            cv::Mat F_MAT,C_MAT;
            vector<cv::Mat> warp_vec;
            cv::Vec2d real_xy;
            int hit_count = 0;
            int hited_count = 0;
            int change_aim;
            double depth = 0;
            deque<double> distances;
            void show_all_dst();
            energy_inf detect_aim();
            Eigen::Vector3d pnp_get(cv::Rect &c_rect);
            void get_ap(Eigen::Vector3d &real_c,cv::Point2f &Aim,cv::RotatedRect &c_rect);
            double depth_filter(deque<double> &dis);
            void make_c_safe(cv::Rect &line);

        };

