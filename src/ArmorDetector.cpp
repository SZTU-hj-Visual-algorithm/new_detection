#include "ArmorDetector.hpp"

#define BINARY_SHOW

//#define DRAW_LIGHTS_CONTOURS
#define DRAW_LIGHTS_RRT

#define DRAW_ARMORS_RRT
#define DRAW_FINAL_ARMOR_S_CLASS
//#define DRAW_FINAL_ARMOR_MAIN

using namespace cv;
using namespace std;

string convertToString(double d) {
    ostringstream os;
    if (os << d)
        return os.str();
    return "invalid conversion";
}

ArmorDetector::ArmorDetector()
{
    lastArmor = Armor();
    detectRoi = cv::Rect();
    lostCnt = 0;

    cnt=0;

    //binary_thresh
    binThresh = 180;   // blue 100  red  70

    //light_judge_condition
    light_max_angle = 45.0;
    light_min_hw_ratio = 1;
    light_max_hw_ratio = 25;   // different distance and focus
    light_min_area_ratio = 0.6;   // RotatedRect / Rect
    light_max_area_ratio = 1.0;

    //armor_judge_condition
    armor_big_max_wh_ratio = 5;
    armor_big_min_wh_ratio = 3;
    armor_small_max_wh_ratio = 3;
    armor_small_min_wh_ratio = 0.5;
    armor_max_offset_angle = 40.0;
    armor_height_offset = 1;
    armor_ij_min_ratio = 0.5;
    armor_ij_max_ratio = 2.0;
    armor_max_angle = 40.0;


    //armor_grade_condition
    big_wh_standard = 3.5; // 4左右最佳，3.5以上比较正，具体再修改
    small_wh_standard = 2; // 2.5左右最佳，2以上比较正，具体再修改
    near_standard = 200;

    //armor_grade_project_ratio
    id_grade_ratio = 0.4;
    wh_grade_ratio = 0.2;
    height_grade_ratio = 0.2;
    near_grade_ratio = 0.1;
    angle_grade_ratio = 0.1;

    grade_standard = 60; // 及格分
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
        //if(lastArmor.light_height_rate > 1)
        //    lu_x_offset = 6 *( pow(lastArmor.light_height_rate - 1, 0.6) + 0.09) * rect.width;
        //else
        //    rd_x_offset = 6 * (pow(1 - lastArmor.light_height_rate, 0.6) + 0.15) * rect.width;

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

    double height = light.height;
    double width = light.width;

    if(height <= 0 || width <= 0)
        return false;

    //高一定要大于宽
    bool standing_ok = height > width;

    //高宽比条件
    double hw_ratio = height / width;
    bool hw_ratio_ok = light_min_hw_ratio < hw_ratio && hw_ratio < light_max_hw_ratio;

    //外接矩形面积和像素点面积之比条件
    double area_ratio =  contourArea(cnt) / (height * width);
    bool area_ratio_ok = light_min_area_ratio < area_ratio && area_ratio < light_max_area_ratio;
    area_ratio_ok = true;

    //灯条角度条件
    bool angle_ok = fabs(90.0 - light.angle) < light_max_angle;

    //灯条判断的条件总集
    bool is_light = hw_ratio_ok && area_ratio_ok && angle_ok && standing_ok;


    if(!is_light)
    {
        //cout<<hw_ratio<<"    "<<area_ratio<<"    "<<light.angle<<endl;
    }


    return is_light;
}

void ArmorDetector::findLights()
{
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(_binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

#ifdef DRAW_LIGHTS_CONTOURS
    for(int i=0;i< contours.size();i++)
    cv::drawContours(_src,contours,i,Scalar(255,0,0),2,LINE_8);
    imshow("contours_src",_src);
#endif

    if (contours.size() < 2)
    {
        printf("no 2 contours\n");
        return;
    }

    //for show
    Mat light_show = _src.clone();

    for (auto & contour : contours)
    {
        RotatedRect r_rect = minAreaRect(contour);
        Light light = Light(r_rect);

        if (isLight(light, contour))
        {
            //cout<<"is_Light   "<<endl;
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
                //cout<<sum_r<<"           "<<sum_b<<endl;
                // Sum of red pixels > sum of blue pixels ?
                light.lightColor = sum_r > sum_b ? RED : BLUE;

                //enermy_color ==  BLUE;
                //cout<<"enermy_color  ==  "<<enermy_color<<endl;
                //cout<<"light.lightColor  ==  "<<light.lightColor<<endl;

                // 颜色不符合电控发的就不放入

                if(light.lightColor == 2)
                {
                    candidateLights.emplace_back(light);
#ifdef DRAW_LIGHTS_RRT

                    Point2f vertice_lights[4];
                    light.points(vertice_lights);
                    for (int i = 0; i < 4; i++) {
                        line(light_show, vertice_lights[i], vertice_lights[(i + 1) % 4], CV_RGB(255, 0, 0),2,LINE_8);
                    }
                    //circle(_src,light.center,5,Scalar(0,0,0),-1);
                    imshow("lights-show-_src", light_show);
#endif //DRAW_LIGHTS_RRT
                }
            }
        }
    }
//cout<<"dengtiao  geshu:  "<<candidateLights.size()<<endl;
}

void ArmorDetector::matchLights()
{
    if(candidateLights.size() < 2)
    {
        printf("no 2 lights\n");
        return;
    }

    // 将旋转矩形从左到右排序
    sort(candidateLights.begin(), candidateLights.end(),
         [](RotatedRect& a1, RotatedRect& a2) {
             return a1.center.x < a2.center.x; });

    //for show
    Mat armors_show = _src.clone();

    for (size_t i = 0; i < candidateLights.size() - 1; i++)
    {
        Light lightI = candidateLights[i];
        Point2f centerI = lightI.center;

        for (size_t j = i + 1; j < candidateLights.size(); j++)
        {
            Light lightJ = candidateLights[j];
            Point2f centerJ = lightJ.center;
            double armorWidth = POINT_DIST(centerI,centerJ) - (lightI.width + lightJ.width)/2.0;
            double armorHeight = (lightI.height + lightJ.height) / 2.0;
            double armor_ij_ratio = lightI.height / lightJ.height;
            double armorAngle = atan2((centerI.y - centerJ.y),fabs(centerI.x - centerJ.x));

            //宽高比筛选条件
            bool small_wh_ratio_ok = armor_small_min_wh_ratio < armorWidth/armorHeight && armorWidth/armorHeight < armor_small_max_wh_ratio;
            bool big_wh_ratio_ok = armor_big_min_wh_ratio < armorWidth/armorHeight && armorWidth/armorHeight < armor_big_max_wh_ratio;
            bool wh_ratio_ok = small_wh_ratio_ok || big_wh_ratio_ok;

            //左右灯条角度差筛选条件
            bool angle_offset_ok = fabs(lightI.angle - lightJ.angle) < armor_max_offset_angle;

            //左右亮灯条中心点高度差筛选条件
            bool height_offset_ok = fabs(lightI.height - lightJ.height) / armorHeight < armor_height_offset;

            //左右灯条的高度比
            bool ij_ratio_ok = armor_ij_min_ratio < armor_ij_ratio && armor_ij_ratio < armor_ij_max_ratio;

            //候选装甲板角度筛选条件
            bool angle_ok = fabs(armorAngle) < armor_max_angle;

            //条件集合
            bool is_like_Armor = wh_ratio_ok && angle_offset_ok && height_offset_ok && ij_ratio_ok && angle_ok;

            if (is_like_Armor)
            {
                //origin
                Point2f armorCenter = (centerI + centerJ) / 2.0;
                RotatedRect armor_rrect = RotatedRect(armorCenter,
                                                      Size2f(armorWidth,armorHeight),
                                                      -armorAngle * 180 / CV_PI);

                //test
//                vector<Point2f> pts = { lightI.top, lightI.bottom, lightJ.bottom, lightJ.top };
//                RotatedRect test = minAreaRect(pts);

                Point2f pt4[4] = { lightI.bottom, lightJ.bottom, lightJ.top, lightI.top };

                if (!conTain(armor_rrect,candidateLights,i,j))
                {
                    Armor armor(armor_rrect);

                    for(int index = 0; index < 4; index++)
                        armor.armor_pt4[index] = pt4[index];

                    if(small_wh_ratio_ok)
                        armor.type = SMALL;
                    else
                        armor.type = BIG;

                    candidateArmors.emplace_back(armor);
#ifdef DRAW_ARMORS_RRT
                    //cout<<"LightI_angle :   "<<lightI.angle<<"   LightJ_angle :   "<<lightJ.angle<<"     "<<fabs(lightI.angle - lightJ.angle)<<endl;
                    //cout<<"armorAngle   :   "<<armorAngle * 180 / CV_PI <<endl;
                    //cout<<"    w/h      :   "<<armorWidth/armorHeight<<endl;
                    //cout<<"height-offset:   "<<fabs(lightI.height - lightJ.height) / armorHeight<<endl;
                    //cout<<" height-ratio:   "<<armor_ij_ratio<<endl;

                    Point2f vertice_armors[4];
                    armor.points(vertice_armors);
                    for (int m = 0; m < 4; m++)
                    {
                        line(armors_show, vertice_armors[m], vertice_armors[(m + 1) % 4], CV_RGB(0, 255, 255),2,LINE_8);
                    }
                    //circle(_src,armorCenter,15,Scalar(0,255,255),-1);
                    imshow("armors-show-_src", armors_show);
#endif //DRAW_ARMORS_RRT
                    (candidateArmors.end()-1)->light_height_rate = armor_ij_ratio;
                }
            }

        }
    }
}

void ArmorDetector::chooseTarget()
{

    if(candidateArmors.empty())
    {
        cout<<"no target!!"<<endl;
        finalArmor = Armor();
    }
    else if(candidateArmors.size() == 1)
    {
        cout<<"get 1 target!!"<<endl;
        detectNum(candidateArmors[0]);

        candidateArmors[0].grade = armorGrade(candidateArmors[0]);
        if (candidateArmors[0].grade > grade_standard)
        {
            finalArmors.emplace_back(candidateArmors[0]);
        }
    }
    else
    {
        cout<<"get "<<candidateArmors.size()<<" target!!"<<endl;

//        int best_index = 0; // 下标
//        int best_record = 0;

        sort(candidateArmors.begin(),candidateArmors.end(), height_sort);

        // 获取每个候选装甲板的id和type

        for(int i = 0; i < candidateArmors.size(); ++i) {
            detectNum(candidateArmors[i]);

//            if (candidateArmors[i].id == 0 && candidateArmors[i].id == 2) {
//                continue;
//            }
//            // 暂时只有五个类别
//            if (candidateArmors[i].id == 1)
//                candidateArmors[i].type = BIG;
//            else if (candidateArmors[i].id == 3 || candidateArmors[i].id == 4)
//                candidateArmors[i].type = SMALL;

            // 装甲板中心点在屏幕中心部分，在中心部分中又是倾斜最小的，
            // 如何避免频繁切换目标：缩小矩形框就是跟踪到了，一旦陀螺则会目标丢失，
            // UI界面做数字选择，选几就是几号，可能在切换会麻烦，（不建议）

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

            candidateArmors[i].grade = armorGrade(candidateArmors[i]);


            if (candidateArmors[i].grade > grade_standard)
            {
                finalArmors.emplace_back(candidateArmors[i]);
            }

//            Armor checkArmor = candidateArmors[i];
//            int grade_record = armorGrade(checkArmor);
//
//            double ff=grade_record;
//            string fff=convertToString(ff);
//            putText(_src,fff,checkArmor.center,FONT_HERSHEY_COMPLEX, 1.0, Scalar(12, 23, 200), 1, 8);
//
//            if (grade_record > best_record && grade_record > grade_standard)
//            {
//                best_record = grade_record;
//                best_index = i;
//            }
        }
//        finalArmor = candidateArmors[best_index];
    }

#ifdef DRAW_FINAL_ARMOR_S_CLASS
    Mat final_armors_src = _src.clone();
    cout<<"final_armors_size:   "<<finalArmors.size()<<endl;

    for(size_t i = 0; i < finalArmors.size(); i++)
    {
        Point2f armor_pts[4];
        finalArmors[i].points(armor_pts);
        for (int j = 0; j < 4; j++)
        {
            line(final_armors_src, armor_pts[j], armor_pts[(j + 1) % 4], CV_RGB(0, 255, 255), 2);
        }

        double ff = candidateArmors[i].grade;
        string fff = convertToString(ff);
        putText(final_armors_src,fff,finalArmors[i].center,FONT_HERSHEY_COMPLEX, 1.0, Scalar(12, 23, 200), 1, 8);

    }

    imshow("final_armors-show", final_armors_src);
#endif //DRAW_FINAL_ARMOR_S_CLASS
}

vector<Armor> ArmorDetector::autoAim(const cv::Mat &src)
{
    finalArmors.clear();
    candidateArmors.clear();
    candidateLights.clear();

    setImage(src);
    findLights();
    matchLights();
    chooseTarget();

/*
    if(!finalArmor.size.empty())
    {
        finalArmor.center.x += detectRoi.x;
        finalArmor.center.y += detectRoi.y;
        lostCnt = 0;
        lastArmor = finalArmor;
    }
    else
    {
        cout<<"no target"<<endl;

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
*/

#ifdef DRAW_FINAL_ARMOR_MAIN
    Mat target = src.clone();
    Point2f vertice_armor[4];
    finalArmor.points(vertice_armor);
    for (int i = 0; i < 4; i++) {
        line(target, vertice_armor[i], vertice_armor[(i + 1) % 4], CV_RGB(0, 255, 0));
    }
    imshow("target-show", target);
#endif //DRAW_FINAL_ARMOR_MAIN


    return finalArmors;
}

void ArmorDetector::detectNum(Armor& armor)
{

    Mat numSrc = _src.clone();
    Mat numDst;
    Mat num;

    // Light length in image
    const int light_length = 30;
    // Image size after warp
    const int warp_height = 60;
    const int small_armor_width = 48;
    const int large_armor_width = 80;
    // Number ROI size
    const cv::Size roi_size(30, 60);

    const int top_light_y = (warp_height - light_length) / 2;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor.type == SMALL ? small_armor_width : large_armor_width;

    cv::Point2f target_vertices[4] = {
            cv::Point(0, bottom_light_y),
            cv::Point(warp_width, bottom_light_y),
            cv::Point(warp_width, top_light_y),
            cv::Point(0, top_light_y),
    };

    Mat rotation_matrix = cv::getPerspectiveTransform(armor.armor_pt4, target_vertices);
    cv::warpPerspective(numSrc, numDst, rotation_matrix, cv::Size(warp_width, warp_height));

    // Get ROI
    numDst = numDst(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Binarize
    //cvtColor(numDst, numDst, cv::COLOR_RGB2GRAY);
    //threshold(numDst, numDst, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    dnn_detect(numDst, armor);


    resize(numDst,numDst,Size(300,600));
    imshow("number_show",numDst);

    cout<<"number:   "<<armor.id<<"   type:   "<<armor.type<<endl;
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
    // 看看裁判系统的通信机制，雷达的制作规范

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
    double wh_ratio = checkArmor.size.width / checkArmor.size.height;
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
    if(candidateArmors.size()==1)  hRotation=1;
    height_grade = hRotation * 100;
    //////////end/////////////////////////////////

    ////////靠近图像中心打分项目//////////////////////
    // 靠近中心，与中心做距离，设定标准值，看图传和摄像头看到的画面的差异
    int near_grade;
    double pts_distance = POINT_DIST(checkArmor.center, Point2f(_src.cols * 0.5, _src.rows * 0.5));
    near_grade = pts_distance/near_standard < 1 ? 100 : (near_standard/pts_distance) * 100;
    ////////end//////////////////////////////////

    /////////角度打分项目//////////////////////////
    // 角度不歪
    int angle_grade;
    angle_grade = (90.0 - fabs(checkArmor.angle)) / 90.0 * 100;
    //cout<<fabs(checkArmor.angle)<<"    "<<angle_grade<<endl;  //55~100
    /////////end///////////////////////////////

    // 下面的系数得详细调节；
    int final_grade = id_grade * id_grade_ratio +
                      wh_grade  * wh_grade_ratio +
                      height_grade  * height_grade_ratio +
                      near_grade  * near_grade_ratio +
                      angle_grade * angle_grade_ratio;


    //cout<<pts_distance<<endl;
    cout<<id_grade<<"   "<<wh_grade<<"   "<<height_grade<<"   "<<near_grade<<"   "<<angle_grade<<"    "<<final_grade<<endl;

    return final_grade;
}
