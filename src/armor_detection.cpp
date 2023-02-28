#include "armor_detection.hpp"

// #define BINARY_SHOW
// #define DRAW_LIGHTS_CONTOURS
// #define DRAW_LIGHTS_RRT
#define SHOW_NUMROI
// #define ERROR_DETECTION
// #define DRAW_ARMORS_RRT
// #define DRAW_FINAL_ARMOR_S_CLASS

using namespace cv;
using namespace std;

//namespace robot_detection {

    ArmorDetector::ArmorDetector()
    {
        cnt=0;

        FileStorage fs("../other/detect_data.yaml", FileStorage::READ);

        //binary_thresh
        binThresh = (int)fs["binThresh"];   // blue 100  red  70

        //light_judge_condition
        light_max_angle = (double)fs["light_max_angle"];
        light_min_hw_ratio = (double)fs["light_min_hw_ratio"];
        light_max_hw_ratio = (double)fs["light_max_hw_ratio"];   // different distance and focus
        light_min_area_ratio = (double)fs["light_min_area_ratio"];   // contourArea / RotatedRect
        light_max_area_ratio = (double)fs["light_max_area_ratio"];
        light_area_max = (double)fs["light_area_max"];

        //armor_judge_condition
        armor_big_max_wh_ratio = (double)fs["armor_big_max_wh_ratio"];
        armor_big_min_wh_ratio = (double)fs["armor_big_min_wh_ratio"];
        armor_small_max_wh_ratio = (double)fs["armor_small_max_wh_ratio"];
        armor_small_min_wh_ratio = (double)fs["armor_small_min_wh_ratio"];//装甲板宽高比真的这么设吗
        armor_max_offset_angle = (double)fs["armor_max_offset_angle"];
        armor_height_offset = (double)fs["armor_height_offset"];
        armor_ij_min_ratio = (double)fs["armor_ij_min_ratio"];
        armor_ij_max_ratio = (double)fs["armor_ij_max_ratio"];
        armor_max_angle = (double)fs["armor_max_angle"];

        //armor_grade_condition
        near_standard = (double)fs["near_standard"];
        height_standard = (double)fs["armor_max_angle"];

        //armor_grade_project_ratio
        id_grade_ratio = (double)fs["id_grade_ratio"];
        near_grade_ratio = (double)fs["near_grade_ratio"];
        height_grade_ratio = (double)fs["height_grade_ratio"];
        grade_standard = (int)fs["grade_standard"]; // 及格分

        //thresh_confidence
        thresh_confidence = (double)fs["thresh_confidence"];
        //enemy_color
        enemy_color = COLOR(((string)fs["enemy_color"]));

        fs.release();
    }

    void ArmorDetector::setImage(const Mat &src)
    {
        src.copyTo(_src);
        src.copyTo(showSrc);

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
//        area_ratio_ok = true;

        //灯条角度条件
        bool angle_ok = fabs(90 - light.angle) <= light_max_angle;
        // cout<<"angle: "<<light.angle<<endl;
//         angle_ok = true;

        //灯条判断的条件总集
        bool is_light = hw_ratio_ok && area_ratio_ok && angle_ok && standing_ok;

//    if(!is_light)
//    {
//        cout<<hw_ratio<<"    "<<area_ratio<<"    "<<light.angle<<endl;
//    }

//        is_light = true;
        return is_light;
    }

    void ArmorDetector::findLights()
    {
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(_binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

#ifdef DRAW_LIGHTS_CONTOURS
        Mat lights_contour_src;
        _src.copyTo(lights_contour_src);
        for(int i=0;i< contours.size();i++)
            cv::drawContours(lights_contour_src,contours,i,Scalar(255,0,255),2,LINE_8);
        imshow("DRAW_LIGHTS_CONTOURS",lights_contour_src);
#endif

        if (contours.size() < 2)
        {
            printf("no 2 contours\n");
            return;
        }

        for (auto & contour : contours)
        {
            RotatedRect r_rect = minAreaRect(contour);
            Light light = Light(r_rect);

            double contour_area = contourArea(contour);

            if (isLight(light, contour) && contour_area < light_area_max)
            {
//                cout<<"ftguhjkl"<<contour_area<<endl;
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
                    // std::cout<<sum_r<<"           "<<sum_b<<std::endl;
                    // Sum of red pixels > sum of blue pixels ?
                    light.lightColor = sum_r > sum_b ? RED : BLUE;

                    //enermy_color ==  BLUE;
                    //cout<<"enermy_color  ==  "<<enermy_color<<endl;
                    // cout<<"light.lightColor  ==  "<<light.lightColor<<endl;

                    // 颜色不符合电控发的就不放入

                    if(light.lightColor == enemy_color)
                    {
                        candidateLights.emplace_back(light);
                    }
                }
            }
        }
#ifdef DRAW_LIGHTS_RRT
        Mat lights_show;
        _src.copyTo(lights_show);
        for(int i=0;i<candidateLights.size();++i)
        {
            Point2f vertice_lights[4];
            candidateLights[i].points(vertice_lights);
            for (int i = 0; i < 4; ++i) {
                line(lights_show, vertice_lights[i], vertice_lights[(i + 1) % 4], CV_RGB(255, 0, 255),2,LINE_8);
            }
        }
        imshow("DRAW_LIGHTS_RRT", lights_show);
#endif //DRAW_LIGHTS_RRT
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
                double armorAngle = atan2((centerI.y - centerJ.y),fabs(centerI.x - centerJ.x))/CV_PI*180.0;


                //宽高比筛选条件
                bool small_wh_ratio_ok = armor_small_min_wh_ratio < armorWidth/armorHeight && armorWidth/armorHeight < armor_small_max_wh_ratio;
                bool big_wh_ratio_ok = armor_big_min_wh_ratio < armorWidth/armorHeight && armorWidth/armorHeight < armor_big_max_wh_ratio;
                bool wh_ratio_ok = small_wh_ratio_ok || big_wh_ratio_ok;

                //左右灯条角度差筛选条件
                bool angle_offset_ok = fabs(lightI.angle - lightJ.angle) < armor_max_offset_angle;

                //左右亮灯条中心点高度差筛选条件
                bool height_offset_ok = fabs(lightI.center.y - lightJ.center.y) / armorHeight < armor_height_offset;


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
                                                          -armorAngle);

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
#endif //DRAW_ARMORS_RRT
                    }
                }
            }
        }
#ifdef DRAW_ARMORS_RRT
        Mat armors_show;
        _src.copyTo(armors_show);
        for(int i=0;i<candidateArmors.size();++i)
        {
            Point2f vertice_armors[4];
            candidateArmors[i].points(vertice_armors);
            for (int m = 0; m < 4; ++m)
            {
                line(armors_show, vertice_armors[m], vertice_armors[(m + 1) % 4], CV_RGB(255, 0, 255),2,LINE_8);
            }
        }
        imshow("DRAW_ARMORS_RRT", armors_show);
#endif //DRAW_ARMORS_RRT
    }

    void ArmorDetector::chooseTarget()
    {
#ifdef ERROR_DETECTION
        Mat error_aim;
        _src.copyTo(error_aim);
#endif //ERROR_DETECTION

        if(candidateArmors.empty())
        {
            // cout<<"no target!!"<<endl;
            // finalArmor = Armor();
            return;
        }
        else if(candidateArmors.size() == 1)
        {
            //cout<<"get 1 target!!"<<endl;
            detectNum(candidateArmors[0]);
            if(candidateArmors[0].confidence < thresh_confidence || candidateArmors[0].id == 2 || candidateArmors[0].id == 0)
            {
#ifdef ERROR_DETECTION
                string information = to_string(candidateArmors[0].id) + ":" + to_string(candidateArmors[0].confidence*100) + "%";
                putText(error_aim, information,candidateArmors[0].armor_pt4[3],FONT_HERSHEY_COMPLEX,1,Scalar(255,0,255),1,8);
#endif //ERROR_DETECTION
                return;
            }
            if(candidateArmors[0].confidence > thresh_confidence && candidateArmors[0].id != 2 && candidateArmors[0].id != 0)
            {
                candidateArmors[0].grade = 100;
                finalArmors.emplace_back(candidateArmors[0]);
            }
        }
        else
        {
            //cout<<"get "<<candidateArmors.size()<<" target!!"<<endl;

            sort(candidateArmors.begin(),candidateArmors.end(),
                 [](Armor &candidate1,Armor &candidate2){return candidate1.size.height > candidate2.size.height;});

            // 获取每个候选装甲板的id和type

            for(int i = 0; i < candidateArmors.size(); ++i) {
                detectNum(candidateArmors[i]);
                if(candidateArmors[i].confidence < thresh_confidence || candidateArmors[i].id == 2 || candidateArmors[i].id == 0)
                {
#ifdef ERROR_DETECTION
                    string information = to_string(candidateArmors[i].id) + ":" + to_string(candidateArmors[i].confidence*100) + "%";
                    putText(error_aim, information,candidateArmors[i].armor_pt4[3],FONT_HERSHEY_COMPLEX,1,Scalar(0,255,255),1,8);
#endif //ERROR_DETECTION
                    continue;
                }
                // 装甲板中心点在屏幕中心部分，在中心部分中又是倾斜最小的，
                // 如何避免频繁切换目标：缩小矩形框就是跟踪到了，一旦陀螺则会目标丢失，
                // UI界面做数字选择，选几就是几号，可能在切换会麻烦，（不建议）

                //打分制筛选装甲板优先级(！！！！最后只保留了优先级条件2和4和id优先级，其他有些冗余！！！！)
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
                if(candidateArmors[i].confidence > thresh_confidence && candidateArmors[i].id != 2 && candidateArmors[i].id != 0)
                {
                    candidateArmors[i].grade = armorGrade(candidateArmors[i]);
                    if (candidateArmors[i].grade > grade_standard)
                    {
                        finalArmors.emplace_back(candidateArmors[i]);
                    }
                }
            }
        }

#ifdef ERROR_DETECTION
        imshow("ERROR_DETECTION", error_aim);
#endif //ERROR_DETECTION

#ifdef DRAW_FINAL_ARMOR_S_CLASS
        Mat final_armors_src;
        _src.copyTo(final_armors_src);
        // std::cout<<"final_armors_size:   "<<finalArmors.size()<<std::endl;

        for(size_t i = 0; i < finalArmors.size(); ++i)
        {
            // Point2f armor_pts[4];
            // finalArmors[i].points(armor_pts);
            for (int j = 0; j < 4; ++j)
            {
                line(final_armors_src, finalArmors[i].armor_pt4[j], finalArmors[i].armor_pt4[(j + 1) % 4], CV_RGB(255, 0, 255), 2);
            }

            string info_id = to_string(finalArmors[i].id) + ":" + to_string(finalArmors[i].confidence*100) + "%";
            putText(final_armors_src,info_id   ,finalArmors[i].armor_pt4[3],FONT_HERSHEY_COMPLEX,2,Scalar(0,255,255),1,8);
            string info_grade = "grade: "+to_string(finalArmors[i].grade);
            putText(final_armors_src,info_grade,finalArmors[i].armor_pt4[0],FONT_HERSHEY_COMPLEX,2,Scalar(0,255,255),1,8);
        }

        if(finalArmors.size() != 0)
            imshow("DRAW_FINAL_ARMOR_S_CLASS", final_armors_src);
#endif //DRAW_FINAL_ARMOR_S_CLASS
    }

    vector<Armor> ArmorDetector::autoAim(const cv::Mat &src)
    {
        if (enemy_color == RED)
            binThresh = 40;
        else
            binThresh = 100;

        //init
        finalArmors.clear();
        candidateArmors.clear();
        candidateLights.clear();

        //do autoaim task
        setImage(src);
        findLights();
        matchLights();
        chooseTarget();

        return finalArmors;
    }

    void ArmorDetector::detectNum(Armor& armor)
    {

        Mat numSrc = _src.clone();
        Mat numDst;
        Mat num;

        // Light length in image
        const int light_length = 14;//大致为高的一半
        // Image size after warp
        const int warp_height = 30;
        const int small_armor_width = 32;//为48/3*2
        const int large_armor_width = 44;//约为70/3*2
        // Number ROI size
        const cv::Size roi_size(20, 30);

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

        dnn_detect(numDst, armor);
#ifdef SHOW_NUMROI
        if ((armor.id!=0)&&(armor.confidence > thresh_confidence))
        {
            resize(numDst, numDst,Size(200,300));
            cvtColor(numDst, numDst, cv::COLOR_BGR2GRAY);
            threshold(numDst, numDst, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            string name = to_string(armor.id) + ":" + to_string(armor.confidence*100) + "%";
            imshow("name", numDst);
            std::cout<<"number:   "<<armor.id<<"   type:   "<<armor.type<<std::endl;
        }
#endif
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
                continue;
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
        id_grade = check_id == 1 ? 100 : 80;
        ////////end///////////////////////////////////

        /////////最大装甲板板打分项目/////////////////////
        // 最大装甲板，用面积，找一个标准值（固定距离（比如3/4米），装甲板大小（Armor.area）大约是多少，分大小装甲板）
        // 比标准大就是100，小就是做比例，，，，可能小的得出来的值会很小
        int height_grade;
        double hRotation = checkArmor.size.height / height_standard;
        if(candidateArmors.size()==1)  hRotation=1;
        height_grade = hRotation * 60;
        //////////end/////////////////////////////////

        ////////靠近图像中心打分项目//////////////////////
        // 靠近中心，与中心做距离，设定标准值，看图传和摄像头看到的画面的差异
        int near_grade;
        double pts_distance = POINT_DIST(checkArmor.center, Point2f(640, 512));
        near_grade = pts_distance/near_standard < 1 ? 100 : (near_standard/pts_distance) * 100;
        ////////end//////////////////////////////////

        // 下面的系数得详细调节；
        int final_grade = id_grade * id_grade_ratio +
                          height_grade * height_grade_ratio +
                          near_grade  * near_grade_ratio;

        // std::cout<<id_grade * id_grade_ratio<<"   "<<height_grade * height_grade_ratio<<"   "<<near_grade * near_grade_ratio<<"    "<<final_grade<<std::endl;

        return final_grade;
    }

    void ArmorDetector::dnn_detect(cv::Mat frame, Armor& armor)// 调用该函数即可返回数字ID
    {
        return dnnDetect.net_forward(dnnDetect.img_processing(std::move(frame)), armor.id, armor.confidence);
    }

//}