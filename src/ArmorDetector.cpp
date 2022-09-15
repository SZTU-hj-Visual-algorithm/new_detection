#include "ArmorDetector.hpp"

#define BINARY_SHOW
//#define DRAW_LIGHTS_CONTOURS
//#define DRAW_FINAL_ARMOR

using namespace cv;
using namespace std;

ArmorDetector::ArmorDetector()
{
    lastArmor = Armor();
    detectRoi = cv::Rect();
    smallArmor = false;
    lostCnt = 0;
    Lost = true;

    //binary_thresh
    binThresh = 150;

    //light_judge_condition
    light_max_angle = 30.0;
    light_min_hw_ratio = 3;
    light_max_hw_ratio = 10;   // different distance and focus
    light_min_area_ratio = 0.6;   // RotatedRect / Rect
    light_max_area_ratio = 1.0;

    //armor_judge_condition
    armor_max_wh_ratio = 4.5;
    armor_min_wh_ratio = 1.5;
    armor_max_angle = 20.0;
    armor_height_offset = 0.3;
    armor_ij_min_ratio = 0.5;
    armor_ij_max_ratio = 5.0;

    //armor_grade_condition
    big_wh_standard = 3.5; // 4左右最佳，3.5以上比较正，具体再修改
    small_wh_standard = 2; // 2.5左右最佳，2以上比较正，具体再修改
    near_standard = 500;

    //armor_grade_project_ratio
    id_grade_ratio = 0.2;
    wh_grade_ratio = 0.3;
    height_grade_ratio = 0.2;
    near_grade_ratio = 0.2;
    angle_grade_ratio = 0.1;

    grade_standard = 70; // 及格分
}

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
        //Rect rect = finalRect;
        Rect rect = lastArmor.boundingRect();

        double scale_w = 2;
        double scale_h = 2;
        int lu_x_offset = 0;
        int rd_x_offset = 0;
        // 获取偏移量
        if(lastArmor.light_height_rate > 1)
            lu_x_offset = 6 *( pow(lastArmor.light_height_rate - 1, 0.6) + 0.09) * rect.width;
        else
            rd_x_offset = 6 * (pow(1 - lastArmor.light_height_rate, 0.6) + 0.15) * rect.width;

        //获取当前帧的roi
        int w = int(rect.width * scale_w);
        int h = int(rect.height * scale_h);
        int x = max(lastCenter.x - w - lu_x_offset, 0);
        int y = max(lastCenter.y - h, 0);
        Point luPoint = Point(x,y);
        x = std::min(lastCenter.x + w + rd_x_offset, src.cols);
        y = std::min(lastCenter.y + h, src.rows);
        Point rdPoint = Point(x,y);
        detectRoi = Rect(luPoint,rdPoint);


        if (!makeRectSafe(detectRoi, src.size())){
            lastArmor = Armor();
            detectRoi = Rect(0, 0, src.cols, src.rows);
            _src = src;
        }
        else
            src(detectRoi).copyTo(_src);
    }
    //二值化
    Mat gray;
    cvtColor(_src,gray,COLOR_BGR2GRAY);
    threshold(gray,_binary,binThresh,255,THRESH_BINARY);
#ifdef BINARY_SHOW
    imshow("_binary",_binary);
#endif //BINARY_SHOW
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
        candidateLights.clear();
        return;
    }

    for (auto & contour : contours)
    {
        RotatedRect r_rect = minAreaRect(contour);
        Light light = Light(r_rect);

        if (isLight(light, contour))
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
                        if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) // 只加正矩形中的轮廓！！！
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
    if(candidateLights.size() < 2)
    {
        lostCnt++;
        candidateArmors.clear();
        return;
    }


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
            double armor_ij_ratio = lightI.height / lightJ.height;

            //宽高比筛选条件
            bool whratio_ok = armor_min_wh_ratio < armorWidth/armorHeight && armorWidth/armorHeight < armor_max_wh_ratio;

            //角度筛选条件
            bool angle_ok = fabs(lightI.angle - lightJ.angle) < armor_max_angle;

            //左右亮灯条中心点高度差筛选条件
            bool height_offset_ok = fabs(lightI.height - lightJ.height) / armorHeight < armor_height_offset;

            //左右灯条的高度比
            bool ij_ratio_ok = armor_ij_min_ratio < armor_ij_ratio && armor_ij_ratio < armor_ij_max_ratio;

            //条件集合
            bool is_like_Armor = whratio_ok && angle_ok && height_offset_ok && ij_ratio_ok;

            if (is_like_Armor)
            {
                Point2f armorCenter = (centerI + centerJ) / 2.0;
                double armorAngle = atan2(fabs(centerI.y - centerJ.y),fabs(centerI.x - centerJ.x));
                RotatedRect armor_rrect = RotatedRect(armorCenter,
                                                      Size2f(armorWidth,armorHeight),
                                                      armorAngle * 180 / CV_PI);
                if (!conTain(armor_rrect,candidateLights,i,j))
                {
                    candidateArmors.emplace_back(armor_rrect);
                    (candidateArmors.end()-1)->light_height_rate = armor_ij_ratio;
                }

            }
        }
    }
    if(candidateArmors.empty())
    {
        lostCnt++;
        return;
    }

}

void ArmorDetector::chooseTarget()
{
    if(candidateArmors.empty())
    {
        lostCnt++;
        finalArmor = Armor();
    }
    else if(candidateArmors.size() == 1)
    {
        detectNum(candidateArmors[0], candidateArmors[0]);
        if (candidateArmors[0].id == 0)
        {
            lostCnt++;
            finalArmor = Armor();
        }
        else if (candidateArmors[0].id == 1)
        {
            candidateArmors[0].type = BIG;
            finalArmor = candidateArmors[0];
        }
        else if (candidateArmors[0].id == 3 || candidateArmors[0].id == 4)
        {
            candidateArmors[0].type = SMALL;
            finalArmor = candidateArmors[0];
        }
    }
    else
    {

        int best_index = 0; // 下标
        int best_record = armorGrade(candidateArmors[0]);

        sort(candidateArmors.begin(),candidateArmors.end(), height_sort);

        // 获取每个候选装甲板的id和type

        for(int i = 0; i < candidateArmors.size(); ++i) {
            candidateArmors[i].id = detectNum(candidateArmors[i]);
            if (candidateArmors[i].id == 0 && candidateArmors[i].id == 2) {
                continue;
            }
            // 暂时只有五个类别
            if (candidateArmors[i].id == 1)
                candidateArmors[i].type = BIG;
            else if (candidateArmors[i].id == 3 || candidateArmors[i].id == 4)
                candidateArmors[i].type = SMALL;

            // int best_index;  // 最佳目标
            // 装甲板中心点在屏幕中心部分，在中心部分中又是倾斜最小的，
            // 如何避免频繁切换目标：缩小矩形框就是跟踪到了，一旦陀螺则会目标丢失，
            // UI界面做数字选择，选几就是几号，可能在切换会麻烦，（不建议）

//                if (finalRect.contains(candidateArmors[index].center) &&
//                    candidateArmors[index].id == finalArmor.id) { break; }  // 追踪上一帧装甲板//??

            //打分制筛选装甲板优先级
            /*最高优先级数字识别英雄1号装甲板，其次3和4号（如果打分的话1给100，3和4给80大概这个比例）
             *1、宽高比（筛选正面和侧面装甲板，尽量打正面装甲板）
             *2、装甲板靠近图像中心
             *3、装甲板倾斜角度最小
             *4、装甲板高最大
             */
            //1、宽高比用一个标准值和当前值做比值（大于标准值默认置为1）乘上标准分数作为得分
            //2、在缩小roi内就给分，不在不给分（分数占比较低）
            //3、90度减去装甲板的角度除以90得到比值乘上标准分作为得分
            //4、在前三步打分之前对装甲板进行高由大到小排序，获取最大最小值然后归一化，用归一化的高度值乘上标准分作为得分
            Armor checkArmor = candidateArmors[i];
            int final_record = armorGrade(checkArmor);
            if (final_record > best_record && final_record > grade_standard)
            {
                best_record = final_record;
                best_index = i;
            }
        }
        finalArmor = candidateArmors[best_index];
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

Armor ArmorDetector::transformPos(const cv::Mat &src)
{

    setImage(src);
    findLights();
    matchLights();
    chooseTarget();

    if(!finalArmor.size.empty())
    {
        finalArmor.center.x += detectRoi.x;
        finalArmor.center.y += detectRoi.y;
        lostCnt = 0;
        lastArmor = finalArmor;
    }
    else
    {
        //这里整体逻辑也要改一下
        ++lostCnt;
        if (lostCnt < 8)
            lastArmor.size = Size(lastArmor.size.width * 1.1, lastArmor.size.height * 1.4);
        else if(lostCnt == 9)
            lastArmor.size = Size(lastArmor.size.width * 1.5, lastArmor.size.height * 1.5);
        else if(lostCnt == 12)
            lastArmor.size = Size(lastArmor.size.width * 1.2, lastArmor.size.height * 1.2);
        else if(lostCnt == 15)
            lastArmor.size = Size(lastArmor.size.width * 1.2, lastArmor.size.height * 1.2);
        else if (lostCnt == 18)
            lastArmor.size = Size(lastArmor.size.width * 1.2, lastArmor.size.height * 1.2);
        else if (lostCnt > 33 )lastArmor.size = Size();
    }
    candidateArmors.clear();
    candidateLights.clear();
    return finalArmor;
}


void ArmorDetector::detectNum(RotatedRect &f_rect, Armor& armor)
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
    DNN_detect::dnn_detect(dst, armor);
}


bool ArmorDetector::conTain(RotatedRect &match_rect,vector<Light> &Lights, size_t &i, size_t &j)
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

int ArmorDetector::armorGrade(const Armor& checkArmor)
{
    //打分前先对装甲板进行高排序
    sort(candidateArmors.begin(),candidateArmors.end(), height_sort);
    // 看看裁判系统的通信机制，雷达的制作规范；

    // 选择用int截断double

    /////////id优先级打分项目////////////////////////
    int id_grade;
    int check_id = checkArmor.id;
    if (check_id == lastArmor.id)
    {
        id_grade = check_id == 1 ? 100 : 80;
    }
    else
    {
        id_grade = check_id == 1 ? 80 : 60;
    }
    ////////end///////////////////////////////////

    /////////长宽比打分项目/////////////////////////
    int wh_grade;
    double wh_ratio = checkArmor.size.width > checkArmor.size.height;
    if(checkArmor.type == BIG)
    {
        wh_grade = wh_ratio/big_wh_standard > 1 ? 100 : (wh_ratio/big_wh_standard) * 100;
    }
    else
    {
        wh_grade = wh_ratio/small_wh_standard > 1 ? 100 : (wh_ratio/small_wh_standard) * 100;
    }
    /////////end///////////////////////////////////

    /////////最大装甲板板打分项目/////////////////////
    // 最大装甲板，用面积，找一个标准值（固定距离（比如3/4米），装甲板大小（Armor.area）大约是多少，分大小装甲板）
    // 比标准大就是100，小就是做比例，，，，可能小的得出来的值会很小
    int height_grade;
    double armor_height = checkArmor.size.height;
    double end_height = (candidateArmors.end()-1)->size.height;
    double begin_height = candidateArmors.begin()->size.height;
    double hRotation = (armor_height - end_height) / (begin_height - end_height);
    height_grade = hRotation * 100;
    //////////end/////////////////////////////////

    ////////靠近图像中心打分项目//////////////////////
    // 靠近中心，与中心做距离，设定标准值，看图传和摄像头看到的画面的差异
    int near_grade;
    double pts_distance = POINT_DIST(checkArmor.center, Point2f(_src.cols * 0.5, _src.rows * 0.5));
    near_grade = pts_distance/near_standard > 1 ? 100 : (pts_distance/near_standard) * 100;
    ////////end//////////////////////////////////

    /////////角度打分项目//////////////////////////
    // 角度不歪
    int angle_grade;
    angle_grade = (90.0 - checkArmor.angle) / 90.0 * 100;
    /////////end///////////////////////////////

    // 下面的系数得详细调节；
    int final_grade = id_grade * id_grade_ratio +
                      wh_grade  * wh_grade_ratio +
                      height_grade  * height_grade_ratio +
                      near_grade  * near_grade_ratio +
                      angle_grade * angle_grade_ratio;

    return final_grade;
}
