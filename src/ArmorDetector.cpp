#include "ArmorDetector.hpp"

using namespace cv;
using namespace std;

void ArmorDetector::setImage(const Mat &src)
{
    const Point &lastCenter = lastArmor.center;

    if (lastCenter.x == 0 || lastCenter.y == 0)
    {
        _src = src;
        detectRoi = Rect(0,0,src.cols,src.rows);
    }
    else
    {
        Rect rect = lastArmor.boundingRect();

        //这里的w和h系数需要实测一下
        double scale_w = 2;
        double scale_h = 2;

        //获取当前帧的roi
        int w = int(rect.width * scale_w);
        int h = int(rect.height * scale_h);
        int x = max(lastCenter.x - w/2, 0);
        int y = max(lastCenter.y - h/2, 0);
        Point luPoint = Point(x,y);
        x = min(lastCenter.x + w/2, src.cols);
        y = min(lastCenter.y + h/2, src.rows);
        Point rdPoint = Point(x,y);
        detectRoi = Rect(luPoint,rdPoint);

        if (!makeRectSafe(detectRoi, src.size())){
            lastArmor = cv::RotatedRect();
            detectRoi = Rect(0, 0, src.cols, src.rows);
            _src = src;
        }
        else src(detectRoi).copyTo(_src);

        //二值化
        Mat gray;
        cvtColor(_src,gray,COLOR_BGR2GRAY);
        threshold(gray,_binary,binThresh,255,THRESH_BINARY);
#ifdef BINARY_SHOW
        imshow("_binary",_binary);
#endif


    }

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


void ArmorDetector::findLights()
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
    for
}

void ArmorDetector::chooseTarget()
{

}

Armor ArmorDetector::transformPos()
{

}


int ArmorDetector::detectNum(RotatedRect &f_rect)
{
    int classid = 0;
    Point2f pp[4];
    f_rect.points(pp);

    Mat numSrc;
    Mat dst;
    _src.copyTo(numSrc);

    //找到能框住整个数字的四个点
    Point2f src_p[4];
    src_p[0].x = pp[1].x + (pp[0].x - pp[1].x)*1.6;
    src_p[0].y = pp[1].y + (pp[0].y - pp[1].y)*1.6;

    src_p[1].x = pp[0].x + (pp[1].x - pp[0].x)*1.6;
    src_p[1].y = pp[0].y + (pp[1].y - pp[0].y)*1.6;

    src_p[2].x = pp[3].x + (pp[2].x - pp[3].x)*1.6;
    src_p[2].y = pp[3].y + (pp[2].y - pp[3].y)*1.6;

    src_p[3].x = pp[2].x + (pp[3].x - pp[2].x)*1.6;
    src_p[3].y = pp[2].y + (pp[3].y - pp[2].y)*1.6;


    //仿射变换
    Mat matrix_per = getPerspectiveTransform(src_p,dst_p);
    warpPerspective(numSrc,dst,matrix_per,Size(30,60));

    Mat bin_dst,gray_dst;
    cvtColor(dst,gray_dst,COLOR_BGR2GRAY);
    threshold(gray_dst,bin_dst,0,255,THRESH_BINARY | THRESH_OTSU);
    vector<vector<Point>> cnts;
    findContours(bin_dst,cnts,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    sort(cnts.begin(),cnts.end(), area_sort);
    Rect dst_roi = boundingRect(cnts[0]);


    Mat num_roi;
    bin_dst(dst_roi).copyTo(num_roi);
    imshow("num",num_roi);




    double minVal,maxVal;
    Point minLoc,maxLoc;
    int max_index = -1;
    double max_val = 0;
    for (int i=0;i<6;i++)
    {
        Mat result;

        matchTemplate(num_roi,temps[i],result,TM_CCORR_NORMED);
        minMaxLoc(result,&minVal, &maxVal, &minLoc, &maxLoc);
        //printf("max_val:%lf\n",maxVal);
        if((maxVal > max_num)&&(maxVal > max_val))
        {
            max_index = i;
            max_val = maxVal;
        }
    }

    if (max_index == 0)
    {
        classid = 1;
    }
    else if (max_index == 1)
    {
        classid = 3;
    }
    else if (max_index == 2)
    {
        classid = 4;
    }
    else
    {
        classid = 0;
    }
    return classid;
}


bool ArmorDetector::conTain(Armor &match_rect,vector<Light> &Lights, size_t &i, size_t &j)
{
    Rect matchRoi = match_rect.boundingRect();
    for (size_t k=i+1;k<j;k++)
    {
        Point2f lightPs[4];
        if (matchRoi.contains(Lights[k].top) || matchRoi.contains(Lights[k].bottom))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}