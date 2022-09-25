#include "ArmorDetector.hpp"

#define BINARY_SHOW

//#define DRAW_LIGHTS_CONTOURS
//#define DRAW_LIGHTS_RRT

//#define DRAW_ARMORS_RRT
//#define DRAW_FINAL_ARMOR_CLASS
#define DRAW_FINAL_ARMOR_MAIN

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
    binThresh = 100;   // blue 100  red  70

    //light_judge_condition
    light_max_angle = 45.0;
    light_min_hw_ratio = 1;
    light_max_hw_ratio = 15;   // different distance and focus
    light_min_area_ratio = 0.6;   // RotatedRect / Rect
    light_max_area_ratio = 1.0;

    //armor_judge_condition
    armor_max_wh_ratio = 3;
    armor_min_wh_ratio = 1.0;
    armor_max_angle = 20.0;
    armor_height_offset = 0.25;
    armor_ij_min_ratio = 0.5;
    armor_ij_max_ratio = 2.0;

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

    double height = light.height;
    double width = light.width;

    if(height <= 0 || width <= 0)
        return false;

    //高一定要大于宽
    bool standing_ok = height > width;

    //高宽比条件
    double hw_ratio = height / width;
    bool hw_ratio_ok = light_min_hw_ratio < hw_ratio && hw_ratio < light_max_hw_ratio;

    double area_ratio = contourArea(cnt) / (height*width)  ;
    bool area_ratio_ok = light_min_area_ratio < area_ratio && area_ratio < light_max_area_ratio;


    //灯条角度条件
    bool angle_ok = fabs(90.0 - light.angle) < light_max_angle || light.angle == 0;

    //灯条判断的条件总集
    bool is_light = hw_ratio_ok && area_ratio_ok && angle_ok && standing_ok;


//    if(!is_light)
//    {
//        cout<<hw_ratio<<"    "<<area_ratio<<"    "<<light.angle<<endl;
//    }


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
        candidateLights.clear();
        return;
    }

    for (auto & contour : contours)
    {
        RotatedRect r_rect = minAreaRect(contour);
        Light light = Light(r_rect);

        if (isLight(light, contour))
        {
//            cout<<"is_Light   "<<endl;
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

                // 颜色不符合电控发的就不放入
                if(light.lightColor == 1)
                {
                    candidateLights.emplace_back(light);
#ifdef DRAW_LIGHTS_RRT
                    Point2f vertice_lights[4];
                    light.points(vertice_lights);
                    for (int i = 0; i < 4; i++) {
                        line(_src, vertice_lights[i], vertice_lights[(i + 1) % 4], CV_RGB(255, 0, 0),2,LINE_8);
                    }
                    //circle(_src,light.center,5,Scalar(0,0,0),-1);
                    imshow("lights-show-_src", _src);
#endif //DRAW_LIGHTS_RRT
                }
            }
        }
    }
    //cout<<"dengtiao  geshu:  "<<candidateLights.size()<<endl;
    if(candidateLights.size()<2)
    {
        return;
    }
}

void ArmorDetector::matchLights()
{
    if(candidateLights.size() < 2)
    {
        printf("no 2 lights\n");
        candidateLights.clear();
        return;
    }

    // 将旋转矩形从左到右排序
    sort(candidateLights.begin(), candidateLights.end(),
         [](RotatedRect& a1, RotatedRect& a2) {
        return a1.center.x < a2.center.x; });
//    cout << "candidateLights size: " << candidateLights.size() << endl;
    for (size_t i=0;i<candidateLights.size()-1;i++)
    {
        Light lightI = candidateLights[i];
        Point2f centerI = lightI.center;

        for (size_t j=i+1;j<candidateLights.size();j++)
        {

            Light lightJ = candidateLights[j];
            Point2f centerJ = lightJ.center;
            double armorWidth = POINT_DIST(centerI,centerJ) - (lightI.width + lightJ.width)/2.0;
            double armorHeight = (lightI.height + lightJ.height) / 2.0;
            double armor_ij_ratio = lightI.height / lightJ.height;

            //宽高比筛选条件
            bool whratio_ok = armor_min_wh_ratio < armorWidth/armorHeight && armorWidth/armorHeight < armor_max_wh_ratio;

            //角度筛选条件
//            bool angle_ok = fabs(lightI.angle - lightJ.angle) < armor_max_angle;

            //左右亮灯条中心点高度差筛选条件
            bool height_offset_ok = fabs(lightI.height - lightJ.height) / armorHeight < armor_height_offset;

            //左右灯条的高度比
            bool ij_ratio_ok = armor_ij_min_ratio < armor_ij_ratio && armor_ij_ratio < armor_ij_max_ratio;

            //条件集合
            bool is_like_Armor = whratio_ok  && height_offset_ok && ij_ratio_ok;
//            cout << "-----------------------------------------------------------------" << endl;
//            cout << "is_like_Armor: " << is_like_Armor << endl;
//            cout << "LightI_angle :   "<<lightI.angle<<"   LightJ_angle :   "<<lightJ.angle<<"     "<<fabs(lightI.angle - lightJ.angle)<<endl;
//            cout << "    w/h      :   "<<armorWidth/armorHeight<<endl;
//            cout << "height-offset:   "<<fabs(lightI.height - lightJ.height) / armorHeight<<endl;
//            cout << " height-ratio:   "<<armor_ij_ratio<<endl;
//            cout << "-----------------------------------------------------------------" << endl;
            if (is_like_Armor)
            {
                //origin
                Point2f armorCenter = (centerI + centerJ) / 2.0;
                double armorAngle = atan2((centerI.y - centerJ.y),fabs(centerI.x - centerJ.x));
                RotatedRect armor_rrect = RotatedRect(armorCenter,
                                                      Size2f(armorWidth,armorHeight),
                                                      -armorAngle * 180 / CV_PI);
//                cout << "is_like_armor" << endl;
                if (!conTain(armor_rrect,candidateLights,i,j))
                {
                    candidateArmors.emplace_back(armor_rrect);
//#ifdef DRAW_ARMORS_RRT
////                    cout<<"LightI_angle :   "<<lightI.angle<<"   LightJ_angle :   "<<lightJ.angle<<"     "<<fabs(lightI.angle - lightJ.angle)<<endl;
////                    cout<<"armorAngle   :   "<<armorAngle * 180 / CV_PI <<endl;
////                    cout<<"    w/h      :   "<<armorWidth/armorHeight<<endl;
////                    cout<<"height-offset:   "<<fabs(lightI.height - lightJ.height) / armorHeight<<endl;
////                    cout<<" height-ratio:   "<<armor_ij_ratio<<endl;
//
//                    Point2f vertice_armors[4];
//                    armor_rrect.points(vertice_armors);
//                    for (int i = 0; i < 4; i++) {
//                        line(_src, vertice_armors[i], vertice_armors[(i + 1) % 4], CV_RGB(0, 255, 0),2,LINE_8);
//                    }
//                    circle(_src,armorCenter,15,Scalar(0,255,255),-1);
//                    imshow("armors-show-_src", _src);
//#endif //DRAW_ARMORS_RRT
                    (candidateArmors.end()-1)->light_height_rate = armor_ij_ratio;
                }
            }

        }
    }


    if(candidateArmors.empty())
    {
//        cout << "return" << endl;
        return;
    }

}

void ArmorDetector::chooseTarget(int timestamp)
{
    if(candidateArmors.empty())
    {
        cout<<"no target!!"<<endl;
        finalArmor = Armor();
        lostCnt += 1;
    }
    else
    {

        cout<<"get "<<candidateArmors.size()<<" target!!"<<endl;

        int best_index = -1; // 下标
        int best_record = 0;

        sort(candidateArmors.begin(),candidateArmors.end(), height_sort);

        new_armors_cnt_map.clear();

        for(int i = 0; i < candidateArmors.size(); ++i) {
            // 获取每个候选装甲板的id和type
            detectNum(candidateArmors[i]);
//            candidateArmors[i].id = 4;
//            candidateArmors[i].confidence = 99;

            int armor_id = candidateArmors[i].id;
            if (candidateArmors[i].confidence < 0.95){
                continue;
            }
            if (armor_id == 0 || armor_id == 2) {
                candidateArmors[i].type = SMALL;
                continue;
            }

            // 暂时只有五个类别
            if (armor_id == 1)
                candidateArmors[i].type = BIG;
            else if (armor_id == 3 || armor_id == 4)
                candidateArmors[i].type = SMALL;


            auto predictors_with_same_key = trackers_map.count(armor_id);  // 已存在类型的预测器数量

            if (predictors_with_same_key == 0)
            {
                SpinTracker tracker(candidateArmors[i], timestamp);
                auto target_predictor = trackers_map.insert(make_pair(armor_id, tracker));
                new_armors_cnt_map[armor_id]++;  // 新类型创建
            }
            else if (predictors_with_same_key == 1)
            {
                auto candidate = trackers_map.find(armor_id);  // 原有的同ID装甲板
                auto delta_dist = POINT_DIST(candidateArmors[i].center, (*candidate).second.last_armor.center);  // 距离

                //若匹配则使用此ArmorTracker
                if (delta_dist <= max_delta_dist && (*candidate).second.last_armor.boundingRect().contains(candidateArmors[i].center))
                {
                    (*candidate).second.update_tracker(candidateArmors[i], timestamp);  // 更新装甲板
//                    cout << "updata tracker" << endl;
                }
                //若不匹配则创建新ArmorTracker
                else
                {
                    SpinTracker tracker(candidateArmors[i], timestamp);
                    trackers_map.insert(make_pair(armor_id, tracker));
                    new_armors_cnt_map[armor_id]++;  // 同类型不同位置创建
//                    cout << "create tracker" << endl;

                }
            }


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

            int final_record = armorGrade(candidateArmors[i]);
//            cout << "final_record: " << final_record << endl;
            if (final_record > best_record  && final_record > grade_standard)
            {
                best_record = final_record;
                best_index = i;
//                cout << "best_index: " << best_index <<endl;
            }
        }


//        cout << "tracker size: " << trackers_map.size() << endl;
        if (!trackers_map.empty())
        {
            //维护预测器Map，删除过久之前的装甲板
            for (auto iter = trackers_map.begin(); iter != trackers_map.end();)
            {
                //删除元素后迭代器会失效，需先行获取下一元素
                auto next = iter;
                // cout<<(*iter).second.last_timestamp<<"  "<<src.timestamp<<endl;
                if ((timestamp - (*iter).second.last_timestamp) > max_delta_t)
                    next = trackers_map.erase(iter);
                else
                    ++next;
                iter = next;
            }
        }


        for (const auto& cnt : new_armors_cnt_map)
        {
            // 新增装甲板数量为1时计算陀螺分数
            if (cnt.second == 1)
            {
                auto same_armors_cnt = trackers_map.count(cnt.first);  // 相同的装甲板数量
                if (same_armors_cnt == 2)
                {
                    cout << "spin detect" << endl;
                    //遍历所有同Key预测器，确定左右侧的Tracker
                    SpinTracker *new_tracker = nullptr;
                    SpinTracker *last_tracker = nullptr;
                    double last_armor_center;
                    double last_armor_timestamp;
                    double new_armor_center;
                    double new_armor_timestamp;
                    int best_prev_timestamp = 0;    //候选ArmorTracker的最近时间戳
                    auto candiadates = trackers_map.equal_range(cnt.first);  // 获取同类型的装甲板
                    for (auto iter = candiadates.first; iter != candiadates.second; ++iter)
                    {
                        //若未完成初始化则视为新增tracker
                        if (!(*iter).second.is_initialized && (*iter).second.last_timestamp == timestamp)
                        {
                            new_tracker = &(*iter).second;
                        }
                        else if ((*iter).second.last_timestamp > best_prev_timestamp && (*iter).second.is_initialized)
                        {
                            best_prev_timestamp = (*iter).second.last_timestamp;
                            last_tracker = &(*iter).second;
                        }

                    }
                    if (new_tracker != nullptr && last_tracker != nullptr)
                    {
                        new_armor_center = new_tracker->last_armor.center.x;
                        new_armor_timestamp = new_tracker->last_timestamp;
                        last_armor_center = last_tracker->last_armor.center.x;
                        last_armor_timestamp = last_tracker->last_timestamp;
                        auto spin_movement = new_armor_center - last_armor_center;  // 中心x坐标： 新 - 旧

                        cout <<cnt.first<<" : "<<spin_movement << endl;
                        if (abs(spin_movement) > 10 && new_armor_timestamp == timestamp && last_armor_timestamp == timestamp)
                        {

                            // 若无该元素则插入新元素
                            if (spin_score_map.count(cnt.first) == 0)
                            {
                                spin_score_map[cnt.first] = 1000 * spin_movement / abs(spin_movement);
                            }
                            //  若已有该元素且目前旋转方向与记录不同,则对目前分数进行减半惩罚
                            else if (spin_movement * spin_score_map[cnt.first] < 0)
                            {
                                spin_score_map[cnt.first] *= 0.5;
                            }
                            // 若已有该元素则更新元素
                            else
                            {
                                spin_score_map[cnt.first] = anti_spin_max_r_multiple * spin_score_map[cnt.first];
                            }
                        }
                    }
                }
            }
        }

        updateSpinScore();

        bool is_target_spinning;
        std::vector<Armor> final_armors;
        SpinHeading spin_status;

        if (spin_status_map.count(candidateArmors[best_index].id) == 0)
        {
            spin_status = UNKNOWN;
            is_target_spinning = false;
        }
        else
        {
            spin_status = spin_status_map[candidateArmors[best_index].id];
            if (spin_status != UNKNOWN)
                is_target_spinning = true;
            else
                is_target_spinning = false;
        }
//        cout << "is target spinning: " << is_target_spinning << endl;
        if(best_index != -1) {
//            cout << "ID: " << candidateArmors[best_index].id << endl;
//            cout << "Confidence: " << candidateArmors[best_index].confidence << endl;
//            cout << "type: " << candidateArmors[best_index].type << endl;
            if (is_target_spinning) {
                auto ID_candiadates = trackers_map.equal_range(candidateArmors[best_index].id);

                for (auto iter = ID_candiadates.first; iter != ID_candiadates.second; ++iter) {
                    if ((*iter).second.last_timestamp == timestamp)  // 同一帧图像的装甲板
                    {
                        final_armors.push_back((*iter).second.last_armor);
                    } else {
                        continue;
                    }
                }

                //若存在一块装甲板
                if (final_armors.size() == 1) {
                    finalArmor = final_armors.at(0);
                }
                    // 若存在两块装甲板
                else if (final_armors.size() == 2) {
                    // 对最终装甲板进行排序，选取与旋转方向相同的装甲板进行更新
                    sort(final_armors.begin(), final_armors.end(),
                         [](Armor &prev, Armor &next) { return prev.center.x < next.center.x; });
                    // 若顺时针旋转选取右侧装甲板更新
                    if (spin_status == CLOCKWISE)
                        finalArmor = final_armors.at(1);
                        // 若逆时针旋转选取左侧装甲板更新
                    else if (spin_status == COUNTER_CLOCKWISE)
                        finalArmor = final_armors.at(0);
                }
            } else {
                finalArmor = candidateArmors[best_index];
            }
        }
    }

#ifdef DRAW_FINAL_ARMOR_CLASS
    Mat final_armor = _src.clone();
    Point2f vertice_armor[4];
    finalArmor.points(vertice_armor);
    for (int i = 0; i < 4; i++) {
        line(final_armor, vertice_armor[i], vertice_armor[(i + 1) % 4], CV_RGB(0, 255, 0));
    }
    imshow("final_armor-show", final_armor);
#endif //DRAW_FINAL_ARMOR_CLASS
}

Armor ArmorDetector::autoAim(const Mat &src, int timestamp)
{

    candidateArmors.clear();
    candidateLights.clear();
    finalArmor = Armor();
    setImage(src);
    findLights();
    matchLights();
    chooseTarget(timestamp);
    if(!finalArmor.size.empty())
    {
//        cout << "finalarmor height: " << finalArmor.size.height << endl;
        finalArmor.center.x += detectRoi.x;
        finalArmor.center.y += detectRoi.y;
        lostCnt = 0;
        lastArmor = finalArmor;
    }
    else
    {

        cout << "lostCnt: " <<lostCnt <<endl;

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
        else if (lostCnt > 33 )lastArmor.center = Point2f(0,0);
    }


#ifdef DRAW_FINAL_ARMOR_MAIN
    Mat target = src.clone();
    Point2f vertice_armor[4];
    finalArmor.points(vertice_armor);
    for (int i = 0; i < 4; i++) {
        line(target, vertice_armor[i], vertice_armor[(i + 1) % 4], CV_RGB(0, 255, 0));
    }
    imshow("target-show", target);
#endif //DRAW_FINAL_ARMOR_MAIN


    return finalArmor;
}


void ArmorDetector::detectNum(Armor& armor)
{
    Point2f pp[4];
    armor.points(pp);

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
//    imshow("dst_num", dst);
    dnn_detect(dst, armor);
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
    id_grade=100;
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
//    cout<<wh_grade<<"   "<<height_grade<<"   "<<near_grade<<"   "<<angle_grade<<"    "<<final_grade<<endl;

    return final_grade;

}
bool ArmorDetector::updateSpinScore()
{
    for (auto score = spin_score_map.begin(); score != spin_score_map.end();)
    {
        SpinHeading spin_status;

        //若Status_Map不存在该元素
        if (spin_status_map.count((*score).first) == 0)
            spin_status = UNKNOWN;
        else
            spin_status = spin_status_map[(*score).first];
        // 若分数过低移除此元素
        if (abs((*score).second) <= anti_spin_judge_low_thres && spin_status != UNKNOWN)
        {
            spin_status_map.erase((*score).first);
            score = spin_score_map.erase(score);
            continue;
        }

        if (spin_status != UNKNOWN)
            (*score).second = 0.838 * (*score).second - 1 * abs((*score).second) / (*score).second;
        else
            (*score).second = 0.997 * (*score).second - 1 * abs((*score).second) / (*score).second;
        // 当小于该值时移除该元素
//        cout << "score: " << (*score).second << endl;
        if (abs((*score).second) < 20 || isnan((*score).second))
        {
            spin_status_map.erase((*score).first);
            score = spin_score_map.erase(score);
            continue;
        }
        else if (abs((*score).second) >= anti_spin_judge_high_thres)
        {
            (*score).second = anti_spin_judge_high_thres * abs((*score).second) / (*score).second;
            if ((*score).second > 0)
                spin_status_map[(*score).first] = CLOCKWISE;
            else if((*score).second < 0)
                spin_status_map[(*score).first] = COUNTER_CLOCKWISE;
        }
        ++score;
    }

    // cout<<"++++++++++++++++++++++++++"<<endl;
    // for (auto status : spin_status_map)
    // {
    //     cout<<status.first<<" : "<<status.second<<endl;
    // }
    return true;
}