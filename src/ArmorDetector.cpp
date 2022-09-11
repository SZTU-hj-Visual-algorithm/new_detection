#include "ArmorDetector.hpp"

//#define BINARY_SHOW
#define DRAW_LIGHTS_CONTOURS
#define DRAW_FINAL_ARMOR

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

        // 这里的w和h系数需要实测一下
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
#endif BINARY_SHOW

    }

}


bool ArmorDetector::isLight(Light& light, vector<Point> &cnt)
{
    int height = light.height;
    int width = light.width;

    //高一定要大于宽
    bool standing_ok = height> width;

    //高宽比条件
    double hw_ratio = height / width;
    bool hw_ratio_ok = light_min_hw_ratio < hw_ratio && hw_ratio < light_max_hw_ratio;

    //外接矩形面积和像素点面积之比条件
    double area_ratio = height * width / contourArea(cnt);
    bool area_ratio_ok = light_min_area_ratio < area_ratio && area_ratio < light_max_area_ratio;

    //灯条角度条件
    bool angle_ok = fabs(90.0 - light.angle) < light_max_angle;

    //灯条判断的条件总集
    bool is_light = hw_ratio_ok && area_ratio_ok && angle_ok && standing_ok;

    return is_light;
}


void ArmorDetector::findLights()
{
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(_binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


    if (contours.size() < 2)
    {
        lostCnt++;
        return;
    }

    for (int index = 0; index < contours.size(); index++)
    {
        RotatedRect r_rect = minAreaRect(contours[index]);
        Light light = Light(r_rect);

        if (isLight(light, contours[index]))
        {
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
                // 颜色不符合电控发的就不放入
                if(light.lightColor == enermy_color)
                {
                    candidateLights.emplace_back(light);
#ifdef DRAW_LIGHTS_CONTOURS
                    Mat lights = _src.clone();
                    Point2f vertice_lights[4];
                    light.points(vertice_lights);
                    for (int i = 0; i < 4; i++) {
                        line(lights, vertice_lights[i], vertice_lights[(i + 1) % 4], CV_RGB(0, 255, 0));
                    }
                    imshow("lights-show", lights);
#endif DRAW_LIGHTS_CONTOURS
                }
            }
        }
    }

    if(candidateLights.size()<2)
    {
        lostCnt++;
        return;
    }
}

void ArmorDetector::matchLights()
{
    for (size_t i=0;i<candidateLights.size()-1;i++)
    {
        Light lightI = candidateLights[i];
        Point centerI = lightI.center;
        for (size_t j=1;j<candidateLights.size();j++)
        {
            Light lightJ = candidateLights[j];
            Point centerJ = lightJ.center;
            double armorWidth = POINT_DIST(centerI,centerJ) - (lightI.width + lightJ.width)/2.0;
            double armorHeight = (lightI.height + lightJ.height) / 2.0;

            //宽高比筛选条件
            bool whratio_ok = armor_min_wh_ratio < armorWidth/armorHeight && armorWidth/armorHeight < armor_max_wh_ratio;

            //角度筛选条件
            bool angle_ok = fabs(lightI.angle - lightJ.angle) < armor_max_angle;

            //左右亮灯条中心点高度差筛选条件
            bool height_offset_ok = fabs(lightI.height - lightJ.height) / armorHeight < armor_height_offset;

            bool is_Armor = whratio_ok && angle_ok && height_offset_ok;

            if (is_Armor)
            {
                Point2f armorCenter = (centerI + centerJ) / 2.0;
                double armorAngle = atan2(fabs(centerI.y - centerJ.y),fabs(centerI.x - centerJ.x));
                RotatedRect armor_rrect = RotatedRect(armorCenter,
                                                      Size2f(armorWidth,armorHeight),
                                                      armorAngle * 180 / CV_PI);
                candidateArmors.emplace_back(armor_rrect);
            }
        }
    }
}

void ArmorDetector::chooseTarget()
{
    if(candidateArmors.size() == 0)
    {
        lostCnt++;
        finalArmor = Armor();
    }
    else if(candidateArmors.size() == 1)
    {
        finalArmor = candidateArmors[0];
    }
    else
    {
        //vector<double> pts_dis_every;
        //vector<int> ID_every;

        // 以下舍弃
        //double min_pts_dis;  // 屏幕中心点与识别到装甲板中心点的距离
        //int min_dis_index;   // 下标

        double min_angle = candidateArmors[0].angle;    // 识别到装甲板中倾斜程度最小的
        int min_angle_index = 0; // 下标

        //int best_index;  // 最佳目标

        // 装甲板中心点在屏幕中心部分，在中心部分中又是倾斜最小的，
        // 如何避免频繁切换目标：缩小矩形框就是跟踪到了，一旦陀螺则会目标丢失，
        // UI界面做数字选择，选几就是几号，可能在切换会麻烦，（不建议）

        for(int index = 1; index < candidateArmors.size(); index++)
        {
            /*
            double pts_dis = POINT_DIST(candidateArmors[index].center, Point2f(_src.cols,_src.rows));
            if(pts_dis < min_pts_dis)
            {
                min_pts_dis = pts_dis;
                min_dis_index = index;
            }
             */

            //打分制筛选装甲板优先级
            /*1、长宽比（筛选正面和侧面装甲板，尽量打正面装甲板）
             *2、装甲板宽高最大
             *3、装甲板靠近图像中心
             *4、装甲板倾斜角度最小
             */

            cv::Rect img_center_rect(_src.cols*0.3,_src.rows*0.3,_src.cols*0.7,_src.rows*0.7);
            Point2f vertice[4];
            candidateArmors[index].points(vertice);
            if(img_center_rect.contains(candidateArmors[index].center) &&
               img_center_rect.contains(vertice[0]) &&
               img_center_rect.contains(vertice[1]) &&
               img_center_rect.contains(vertice[2]) &&
               img_center_rect.contains(vertice[3]) )
            {
                if(min_angle > candidateArmors[index].angle)
                {
                    min_angle = candidateArmors[index].angle;
                    min_angle_index = index;
                }
            }
        }

        finalArmor = candidateArmors[min_angle_index];
    }

#ifdef DRAW_FINAL_ARMOR
    Mat final_armor = _src.clone();
    Point2f vertice_armor[4];
    finalArmor.points(vertice_armor);
    for (int i = 0; i < 4; i++) {
        line(final_armor, vertice_armor[i], vertice_armor[(i + 1) % 4], CV_RGB(0, 255, 0));
    }
    imshow("final_armor-show", final_armor);
#endif DRAW_FINAL_ARMOR
}

Armor ArmorDetector::transformPos()
{

    finalRect = finalArmor.boundingRect();
    if(!finalRect.empty())
    {
        lostCnt = 0;
    }
    else
    {
        ++lostCnt;

        if (lostCnt < 8)
            finalRect.size() = Size(finalRect.width * 1.0, finalRect.height * 1.0);
        else if(lostCnt == 9)
            finalRect.size() = Size(finalRect.width * 1.2, finalRect.height * 1.2);
        else if(lostCnt == 12)
            finalRect.size() = Size(finalRect.width * 1.5, finalRect.height * 1.5);
        else if(lostCnt == 15)
            finalRect.size() = Size(finalRect.width * 1.2, finalRect.height * 1.2);
        else if (lostCnt == 18)
            finalRect.size() = Size(finalRect.width * 1.2, finalRect.height * 1.2);
        else if (lostCnt > 33 )
            finalRect.size() = Size();
    }
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


    // 仿射变换
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
        // 加入判定为大装甲板
        enermy_type = BIG;
    }
    else if (max_index == 1)
    {
        classid = 3;
        enermy_type = SMALL;
    }
    else if (max_index == 2)
    {
        classid = 4;
        enermy_type = SMALL;
    }
    else if (max_index == 3)
    {
        classid = 2;
        enermy_type = SMALL;
    }
    else
    {
        classid = 0;
        enermy_type = BIG;
    }
    return classid;
}


bool ArmorDetector::conTain(Armor &match_rect,vector<Light> &Lights, size_t &i, size_t &j)
{
    Rect matchRoi = match_rect.boundingRect();
    for (size_t k=i+1;k<j;k++)
    {
        Point2f lightPs[4];
        // 灯条五等份位置的点
        if (matchRoi.contains(Lights[k].top)    ||
            matchRoi.contains(Lights[k].bottom) ||
            matchRoi.contains(Point2f(Lights[k].top.x+Lights[k].height*0.25 , Lights[k].top.y+Lights[k].height*0.25)) ||
            matchRoi.contains(Point2f(Lights[k].bottom.x-Lights[k].height*0.25 , Lights[k].bottom.y-Lights[k].height*0.25)) ||
            matchRoi.contains(Lights[k].center)  )
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