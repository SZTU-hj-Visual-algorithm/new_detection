#include "ArmorDetector.hpp"


void ArmorDetector::setImage(const cv::Mat &src)
{

}

bool ArmorDetector::isLight(const Light& light)
{
    double hw_ratio = light.height / light.width;
    bool hw_ratio_ok = light.min_hw_ratio < hw_ratio && hw_ratio < light.max_hw_ratio;

    double area_ratio = light.height * light.width / light.boundingRect().area();
    bool area_ratio_ok = light.min_area_ratio < area_ratio && area_ratio < light.max_area_ratio;

    bool angle_ok = light.angle < light.max_angle;

    bool is_light = hw_ratio_ok && area_ratio_ok && angle_ok ;

    return is_light;
}

std::vector<Light> ArmorDetector::findLights()
{
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(_binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int index = 0; index < contours.size(); index++) {
        if (contours.size() < 2) continue;

        cv::RotatedRect r_rect = cv::minAreaRect(contours[index]);
        Light light = Light(r_rect);

        if (isLight(light)) {
            cv::Rect rect = r_rect.boundingRect();

            if (0 <= rect.x && 0 <= rect.width  && rect.x + rect.width  <= _src.cols &&
                0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= _src.rows)
            {
                int sum_r = 0, sum_b = 0;
                cv::Mat roi = _src(rect);
                // Iterate through the ROI
                for (int i = 0; i < roi.rows; i++)
                {
                    for (int j = 0; j < roi.cols; j++)
                    {
                        if (cv::pointPolygonTest(contours[index], cv::Point2f(j + rect.x, i + rect.y), false) >= 0) // 只加正矩形中的轮廓！！！
                        {
                            sum_r += roi.at<cv::Vec3b>(i, j)[2];
                            sum_b += roi.at<cv::Vec3b>(i, j)[0];
                        }
                    }
                }
                // Sum of red pixels > sum of blue pixels ?
                light.lightColor = sum_r > sum_b ? RED : BLUE;
                candidateLights.emplace_back(light);  //符合敌方颜色在加入vector；
            }
        }
    }
}

void ArmorDetector::matchLights()
{

}

aimInformation ArmorDetector::chooseTarget()
{

}


int ArmorDetector::detectNum(cv::RotatedRect &final_rect)
{

}


bool ArmorDetector::conTain(cv::RotatedRect &match_rect,std::vector<cv::RotatedRect> &Lights, size_t &i, size_t &j)
{

}