#include "armor_track.h"
#include "gimbal_control.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

//#define DRAW_MATCH_ARMOR
#define SHOW_TRACK_PREDICT
#define SHOW_SINGER_PREDICT

//namespace robot_detection {

ArmorTracker::ArmorTracker()
{
    cv::FileStorage fs("../other/track_data.yaml", cv::FileStorage::READ);

    KF.initial_KF();

    locate_target = false;
    wait_start = true;
    enemy_armor = Armor();

    tracker_state = MISSING;
    tracking_id = 0;

    find_aim_cnt = 0;
    find_threshold = (int)fs["find_threshold"];

    lost_aim_cnt = 0;
    lost_threshold = (int)fs["lost_threshold"];

    new_old_threshold = (double)fs["new_old_threshold"];

    anti_spin_max_r_multiple = (double)fs["anti_spin_max_r_multiple"];
    anti_spin_judge_low_thres = (int)fs["anti_spin_judge_low_thres"];
    anti_spin_judge_high_thres = (int)fs["anti_spin_judge_high_thres"];
    max_delta_t = (int)fs["max_delta_t"];
    max_delta_dist = (int)fs["max_delta_dist"];

    isChangeSameID = false;
    fs.release();
}
/**
 * @brief
 * @return
 */
void ArmorTracker::reset()
{
    wait_start = true;

    KF.initial_KF();
    Singer.Reset();

    locate_target = false;
    enemy_armor = Armor();
    tracker_state = MISSING;
    tracking_id = 0;
    find_aim_cnt = 0;
    lost_aim_cnt = 0;
}


/**
 * @brief 初始化，选择最优装甲板，设置卡尔曼的F和x_1
 * @param find_armors
 * @return
 */
bool ArmorTracker::initial(std::vector<Armor> &find_armors)
{
    if(find_armors.empty())
    {
        return false;
    }

    sort(find_armors.begin(),find_armors.end(),
      [](Armor &armor1,Armor &armor2){
         return armor1.grade > armor2.grade;});

    // select enemy
    enemy_armor = find_armors[0];
    tracker_state = DETECTING;
    tracking_id = enemy_armor.id;
    KF.initial_KF();
    enemy_armor.world_position = AS.pixel2imu(enemy_armor,1);
    KF.setXPost(enemy_armor.world_position);
    Singer.setXpos(enemy_armor.world_position.head(2));
//    std::cout<<"track_initial"<<std::endl;
    return true;
}

/**
 * @brief
 * @param find_armors
 * @param dt 两帧之间时间间隔, 跟得住目标
 * @return
 */
bool ArmorTracker::selectEnemy2(std::vector<Armor> &find_armors, double dt) {

    KF.setF(dt);
    predicted_enemy = KF.predict();
#ifdef SHOW_TRACK_PREDICT
//    cv::Mat PointTrack = AS._src.clone();
    Eigen::Vector3d predictedE = Eigen::Vector3d(predicted_enemy.head(3));
    cv::Point2f preTrack = AS.imu2pixel(predictedE);
    cv::circle(_src, preTrack, 5, cv::Scalar(0, 0, 255), -1);
    cv::imshow("_src", _src);
#endif

    Armor matched_armor;
    SpinHeading spin_status;
    bool matched = false;
    if (!find_armors.empty()) {
        spin_status = spin_status_map[tracking_id];
        jump_tracker = {};
        if (spin_status == UNKNOWN) {
            double min_position_diff = DBL_MAX;
            for (auto &armor: find_armors) {
                armor.world_position = AS.pixel2imu(armor,1);
                Eigen::Vector3d pre = predicted_enemy.head(3);
                double position_diff = (pre - armor.world_position).norm();
                if (position_diff < min_position_diff) {
                    min_position_diff = position_diff;
                    matched_armor = armor;
                }
            }
//        std::cout<<"min_position_diff:"<<min_position_diff<<std::endl;
            if (min_position_diff < new_old_threshold && matched_armor.id == tracking_id) {
                matched = true;
                predicted_enemy = KF.update(matched_armor.world_position);
            } else {
                // 本帧内是否有相同ID
                double same_armor_distance = DBL_MAX;
                for (auto &armor: find_armors)
                {
                    double dis_tmp = (enemy_armor.world_position - armor.world_position).norm();
                    if (armor.id == tracking_id && dis_tmp < same_armor_distance)
                    {
                        matched = true;
                        //TODO:封装
                        KF.initial_KF();
                        Eigen::Matrix<double, 6, 1> pos_speed;
                        pos_speed << armor.world_position, predicted_enemy.tail(3);
                        KF.setPosAndSpeed(armor.world_position, predicted_enemy.tail(3));
                        predicted_enemy = pos_speed;

                        same_armor_distance = dis_tmp;
                        matched_armor = armor;
                    }
                }
            }
        } else {
            auto ID_candiadates = trackers_map.equal_range(tracking_id);
            auto backward_with_same_ID = trackers_map.count(tracking_id);  // 已存在类型的预测器数量

            std::vector<Armor> final_armors;
            for (auto &armor: find_armors) {
                if (armor.id == tracking_id)  // 同一帧图像的装甲板
                {
                    armor.world_position = AS.pixel2imu(armor,1);
                    final_armors.push_back(armor);
                } else {
                    continue;
                }
            }


            //若存在一块装甲板
            if (final_armors.size() == 1)
            {
                int kf_state = 0; // 1: 1->1(跳变); 2: 1->1/2->1(无跳变); 3: 虚拟装甲板
                double delay_time = 0.5; // TODO：改为子弹飞行时间+系统时延
//                double relative_time = seconds_duration(t - jump_tracker.jump_time).count();

                matched = true;
                matched_armor = final_armors.at(0);
                if (backward_with_same_ID == 1 &&  // 单装甲板跳变；同时发生装甲板出现和消失
                (matched_armor.world_position - trackers_map.find(tracking_id)->second.last_armor.world_position).norm()
                >new_old_threshold)
                {
                    spin_T = seconds_duration(t - jump_trackers.at(0).jump_time).count();

                    jump_tracker.jump_armor = matched_armor;
                    jump_tracker.jump_time = t;
                    jump_trackers.push_back(jump_tracker);
                    jump_trackers.erase(jump_trackers.begin());

                    disappear_tracker.disappear_armor = matched_armor;
                    kf_state = 1;
                    first_vir_armor = true;
                }
                else  // 二/一 到一，追踪；无变化
                {
                    if(backward_with_same_ID == 2)
                    {
                        spin_T = seconds_duration(t - jump_trackers.front().jump_time).count();
                        disappear_tracker.disappear_time = t;
                        disappear_tracker.disappear_armor = matched_armor;
                    }
                    kf_state = 2;
                }

                if (jump_trackers.size() == 2)  // 维护jump_trackers
                {
                    jump_trackers.erase(jump_trackers.begin());
                }

                // 如果超过限制时间，构造虚拟装甲板。
                if(spin_T > 0 &&
                seconds_duration(t - jump_tracker.jump_time) > seconds_duration(spin_T - delay_time))
                {
                    /// 根据当前装甲板、当前装甲板出现的第一帧装甲板、以及上一个消失的装甲板坐标用最小二乘法拟合圆，计算圆心、半径
                    // 将Eigen::Vector3d转换为cv::Mat
                    cv::Mat temp_mat1, temp_mat2, temp_mat3;
                    cv::eigen2cv(matched_armor.world_position, temp_mat1);
                    cv::eigen2cv(jump_trackers.front().jump_armor.world_position, temp_mat2);
                    cv::eigen2cv(disappear_tracker.disappear_armor.world_position, temp_mat3);
                    // 将cv::Mat转换为cv::Point2f
                    Circle circle = fitCircle((cv::Point2f)temp_mat1, (cv::Point2f)temp_mat2, (cv::Point2f)temp_mat3);

                    /// 转化世界坐标
                    double new_world_position_x;
                    double new_world_position_y;
                    // 该点与圆心连线和水平方向的夹角θ
                    double theta = atan2(matched_armor.world_position[1] - circle.y,
                                         matched_armor.world_position[0] - circle.x);
                    // 逆时针+，顺时针-
                    if(spin_status == COUNTER_CLOCKWISE)
                    {
                        double alpha = theta + M_PI / 2;
                        new_world_position_x = circle.r * cos(alpha) + circle.x;
                        new_world_position_y = circle.r * sin(alpha) + circle.y;
                    }
                    else
                    {
                        double beta = theta - M_PI / 2;
                        new_world_position_x = circle.r * cos(beta) + circle.x;
                        new_world_position_y = circle.r * sin(beta) + circle.y;
                    }
                    matched_armor.world_position.x() = new_world_position_x;
                    matched_armor.world_position.y() = new_world_position_y;

                    /// 转化相机坐标
                    matched_armor.camera_position = AS.imu2cam(matched_armor.world_position);
                    kf_state = 3;
                    if(first_vir_armor)first_vir_armor = false;
                }
                /// 初始化/更新KF参数
                if(kf_state == 1 ||( kf_state == 3 && first_vir_armor))
                {
                    KF.initial_KF();
                    Eigen::Matrix<double, 6, 1> pos_speed;
                    pos_speed << matched_armor.world_position, predicted_enemy.tail(3);
                    KF.setPosAndSpeed(matched_armor.world_position, predicted_enemy.tail(3));
                    predicted_enemy = pos_speed;
                }
                else if(kf_state == 2 || (kf_state == 3 && !first_vir_armor))
                {
                    predicted_enemy = KF.update(matched_armor.world_position);
                }
            }
            // 若存在两块装甲板
            else if (final_armors.size() == 2) {
                // 对最终装甲板进行排序，选取与旋转方向相同的装甲板进行更新
                sort(final_armors.begin(), final_armors.end(),
                     [](Armor &prev, Armor &next) { return prev.center.x < next.center.x; });
                // 若顺时针旋转选取右侧装甲板
                if (spin_status == CLOCKWISE)
                    matched_armor = final_armors.at(1);
                // 若逆时针旋转选取左侧装甲板
                else if (spin_status == COUNTER_CLOCKWISE)
                    matched_armor = final_armors.at(0);
                matched = true;

                // 一到二，跳变；新装甲板出现
                if (backward_with_same_ID == 1 && !first_vir_armor)
                {
                    KF.initial_KF();
                    Eigen::Matrix<double, 6, 1> pos_speed;
                    pos_speed << matched_armor.world_position, predicted_enemy.tail(3);
                    KF.setPosAndSpeed(matched_armor.world_position, predicted_enemy.tail(3));
                    predicted_enemy = pos_speed;

                    jump_tracker.jump_armor = matched_armor;
                    jump_tracker.jump_time = t;
                    jump_trackers.push_back(jump_tracker);
                }
                else  // 二到二，追踪；无变化
                {
                    predicted_enemy = KF.update(matched_armor.world_position);
                }
                first_vir_armor = true;
            }
        }
    }
    if (matched)
    {
#ifdef DRAW_MATCH_ARMOR
        cv::Mat m_a = AS._src.clone();
        cv::circle(m_a,matched_armor.center,matched_armor.size.width/10,cv::Scalar(255,255,0),-1);
        Eigen::Vector3d predicted_track;
        predicted_track << predicted_enemy.head(3);
        cv::Point2f p = AS.imu2pixel(predicted_track);
        cv::circle(m_a,p,matched_armor.size.width/15,cv::Scalar(0,0,255),-1);
        cv::imshow("DRAW_MATCH_ARMOR",m_a);
#endif
        enemy_armor = matched_armor;
    }

    if (tracker_state == DETECTING) {
        // DETECTING
        if (matched) {
            find_aim_cnt++;
            if (find_aim_cnt > find_threshold) {
                find_aim_cnt = 0;
                tracker_state = TRACKING;
            }
        } else {
            find_aim_cnt = 0;
            tracker_state = MISSING;
        }

    } else if (tracker_state == TRACKING) {
        // TRACKING
        if (!matched) {
            tracker_state = LOSING;
            lost_aim_cnt++;
        }

    } else if (tracker_state == LOSING) {
        if (!matched) {
            lost_aim_cnt++;
            if (lost_aim_cnt > lost_threshold) {
                lost_aim_cnt = 0;
                tracker_state = MISSING;
            }
        } else {
            tracker_state = TRACKING;
            lost_aim_cnt = 0;
        }
    }

    if (tracker_state == MISSING)
    {
        reset();
        return false;
    }
    KF.setPosAndSpeed(matched_armor.world_position,predicted_enemy.tail(3));

    if(tracker_state == LOSING)
    {
        enemy_armor.world_position = predicted_enemy.head(3);
        KF.setPosAndSpeed(enemy_armor.world_position,predicted_enemy.tail(3));
    }

    return true;
}


/**
 * @brief 对处于跟踪和正在丢失状态时做 预测，引入各种时间
 * @param dt
 * @return
 */
bool ArmorTracker::estimateEnemy(double dt)
{
    cv::putText(_src, std::to_string(tracker_state),cv::Point(50,100),1,5,cv::Scalar(0,0,255),3);
    if(tracker_state == TRACKING || tracker_state == LOSING)
    {
        double fly_time = AS.getFlyTime(enemy_armor.world_position);
        ////////////////Singer predictor//////////////////////////////
        if(!Singer.SingerPrediction(dt,fly_time,
                                    enemy_armor.world_position,
                                    predicted_position))
        {
            return false;
        }
		////////////////Singer predictor//////////////////////////////
        bullet_point = AS.airResistanceSolve(predicted_position);
#ifdef SHOW_SINGER_PREDICT
        cv::Point2f pixelPos = AS.imu2pixel(bullet_point);
//        std::cout<<"pixelPos"<<pixelPos<<std::endl;
		circle(_src,enemy_armor.center,5,cv::Scalar(255,0,0),-1);
		circle(_src,pixelPos,5,cv::Scalar(0,255,255),-1);
        cv::imshow("_src",_src);
        //        if(tracker_state == LOSING)cv::waitKey(0);
#endif
		return true;
    }
    else if (tracker_state == DETECTING)
    {
        double fly_time = AS.getFlyTime(enemy_armor.world_position);
        ////////////////Singer predictor//////////////////////////////
        if(!Singer.SingerPrediction(dt,fly_time,
                                    enemy_armor.world_position,
                                    predicted_position))
        {
            return false;
        }
        ////////////////Singer predictor//////////////////////////////
#ifdef SHOW_SINGER_PREDICT
        cv::Point2f pixelPos = AS.imu2pixel(bullet_point);
//        std::cout<<"pixelPos"<<pixelPos<<std::endl;
		circle(_src,enemy_armor.center,5,cv::Scalar(255,0,0),-1);
        circle(_src,pixelPos,5,cv::Scalar(0,0,255),-1);
        cv::imshow("_src",_src);
//        cv::waitKey(0);
#endif
//        locate_target = false;
        return false;
    }
//    else
//    {
//        locate_target = false;
//        return false;
//    }
}
/**
 * @brief
 * @param src
 * @param armors
 * @param time
 * @return
 */
bool ArmorTracker::locateEnemy(const cv::Mat& src, std::vector<Armor> &armors, const chrono_time &time)
{
	_src=src;

    AS.RotationMatrix_imu = AS.quaternionToRotationMatrix(AS.quaternion);

    if(!locate_target)
    {
        if(initial(armors))
        {
            locate_target = true;
        }
        else
        {
            locate_target = false;
        }
        Eigen::Vector3d gun_offset = {0,0,0};   // m
        gun_offset +=enemy_armor.camera_position;

        bullet_point = AS.airResistanceSolve(gun_offset);

        pitch = -atan2(bullet_point[1],bullet_point[2])/CV_PI*180;
        yaw   = -atan2(bullet_point[0],bullet_point[2])/CV_PI*180;
        return false;
    }
    else
    {
        if (wait_start)
        {
            t = time;
            wait_start = false;
            return false;
        }
        double dt = seconds_duration (time - t).count();
//        std::cout<<"dt:"<<dt<<std::endl;
        t = time;
        if(!selectEnemy2(armors,dt))
        {
            return false;
        }
        if (tracker_state == TRACKING)
        {
            spin_detect();
        }
        if(!estimateEnemy(dt))
        {
            return false;
        }

        Eigen::Vector3d gun_offset = {0,0,0};   // m
        gun_offset +=enemy_armor.camera_position;
        bullet_point = AS.airResistanceSolve(gun_offset);
        pitch = -atan2(bullet_point[1],bullet_point[2])/CV_PI*180;
        yaw   = -atan2(bullet_point[0],bullet_point[2])/CV_PI*180;
        return true;
    }
}
/**
 * @brief 显示结果
 */
void ArmorTracker::show()
{
    switch (tracker_state)
    {
        case 0: // MISSING
            cv::putText(_src,"MISSING   ",cv::Point2f(1280 - 200,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
            break;
        case 1: // DETECTING
            cv::putText(_src,"DETECTING " + std::to_string(tracking_id),cv::Point2f(1280 - 200,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
            break;
        case 2: // LOSING
            cv::putText(_src,"LOSING    " + std::to_string(tracking_id),cv::Point2f(1280 - 200,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
            break;
        case 3: // TRACKING
            cv::putText(_src,"TRACKING  " + std::to_string(tracking_id),cv::Point2f(1280 - 200,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
            break;
    }
    cv::putText(_src,std::to_string(1) +" ARMOR",cv::Point2f(1280 - 200,30),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(_src,"PITCH    : "+std::to_string(pitch),cv::Point2f(0,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(_src,"YAW      : "+std::to_string(yaw),cv::Point2f(0,90),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(_src,"DISTANCE : "+std::to_string(enemy_armor.camera_position.norm())+"m",cv::Point2f(0,30),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    std::string info_grade = "grade: "+std::to_string(enemy_armor.grade);
    putText(_src,info_grade,enemy_armor.armor_pt4[0],cv::FONT_HERSHEY_COMPLEX,2,cv::Scalar(0,255,255),1,8);

    cv::Point2f vertice_lights[4];
    enemy_armor.points(vertice_lights);
    for (int i = 0; i < 4; i++) {
        line(_src, vertice_lights[i], vertice_lights[(i + 1) % 4], CV_RGB(0, 255, 255),2,cv::LINE_8);
    }
    std::string information = std::to_string(enemy_armor.id) + ":" + std::to_string(enemy_armor.confidence*100) + "%";
    //        putText(final_armors_src,ff,finalArmors[i].center,FONT_HERSHEY_COMPLEX, 1.0, Scalar(12, 23, 200), 1, 8);
    putText(_src, information,enemy_armor.armor_pt4[3],cv::FONT_HERSHEY_COMPLEX,2,cv::Scalar(255,0,255),1,3);

    imshow("final_result",_src);
}

/**
 * @brief 维护反陀螺状态map
 * @return 自然衰减状态分数，将分数低于阈值的去除，高于最大阈值的限制在最大值
 */
bool ArmorTracker::updateSpinScore()
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

/**
 * @brief 反陀螺状态检测
 */
void ArmorTracker::spin_detect() {
    int armor_id = enemy_armor.id;
    new_armors_cnt_map.clear();
    auto predictors_with_same_key = trackers_map.count(armor_id);  // 已存在类型的预测器数量
    if (predictors_with_same_key == 1) {
        auto candidate = trackers_map.find(armor_id);  // 原有的同ID装甲板
        auto delta_dist = (enemy_armor.world_position, (*candidate).second.last_armor.world_position).norm();  // 距离
        if (delta_dist < new_old_threshold) {
            (*candidate).second.update_tracker(enemy_armor, t);  // 更新装甲板
        }
        else {
            SpinTracker spinTracker(enemy_armor, t); // 同类型不同位置创建
            trackers_map.insert(std::make_pair(armor_id, spinTracker));
            new_armors_cnt_map[armor_id]++;
        }
    } else {
        // 1e9无实际意义，仅用于非零初始化
        double min_delta_dist = 1e9;

        bool is_best_candidate_exist = false;
        std::multimap<int, SpinTracker>::iterator best_candidate;
        auto candiadates = trackers_map.equal_range(armor_id);
        for (auto iter = candiadates.first; iter != candiadates.second; ++iter) {
            auto delta_t = t - (*iter).second.last_timestamp;
            auto delta_dist = (enemy_armor.world_position - (*iter).second.last_armor.world_position).norm();

            // 在同一位置存在过装甲板且时间最接近设为最高优先级，
            if (delta_dist <= new_old_threshold && delta_dist <= min_delta_dist)  // 距离需要调试
            {
                min_delta_dist = delta_dist;
                best_candidate = iter;
                is_best_candidate_exist = true;
            }
        }
        if (is_best_candidate_exist)
        {
            (*best_candidate).second.update_tracker(enemy_armor, t);
        }
        else{ // first time
            SpinTracker spinTracker(enemy_armor, t); // 同类型不同位置创建
            trackers_map.insert(std::make_pair(static_cast<int&&>(armor_id), static_cast<SpinTracker&&>(spinTracker)));
        }
    }

    if (!trackers_map.empty()) {
        //维护预测器Map，删除过久之前的装甲板
        for (auto iter = trackers_map.begin(); iter != trackers_map.end();) {
            //删除元素后迭代器会失效，需先行获取下一元素
            auto next = iter;
            // cout<<(*iter).second.last_timestamp<<"  "<<src.timestamp<<endl;
            if ((t - (*iter).second.last_timestamp).count() > max_delta_t)  //TODO：时间需要测试
                next = trackers_map.erase(iter);
            else
                ++next;
            iter = next;
        }
    }

    if (new_armors_cnt_map[armor_id] == 1) {
        auto same_armors_cnt = trackers_map.count(armor_id);  // 相同的装甲板数量
        if (same_armors_cnt == 2) {
//            cout << "spin detect" << endl;
            //遍历所有同Key预测器，确定左右侧的Tracker
            SpinTracker *new_tracker = nullptr;
            SpinTracker *last_tracker = nullptr;
            double last_armor_center;
            chrono_time last_armor_timestamp;
            double new_armor_center;
            chrono_time new_armor_timestamp;
//            int best_prev_timestamp = 0;    //候选ArmorTracker的最近时间戳
            auto candiadates = trackers_map.equal_range(armor_id);  // 获取同类型的装甲板
            for (auto iter = candiadates.first; iter != candiadates.second; ++iter) {
                //若未完成初始化则视为新增tracker
                if (!(*iter).second.is_initialized) {
                    new_tracker = &(*iter).second;
                } else {
                    last_tracker = &(*iter).second;
                }

            }
            if (new_tracker != nullptr && last_tracker != nullptr) {
                new_armor_center = new_tracker->last_armor.center.x;
                new_armor_timestamp = new_tracker->last_timestamp;
                last_armor_center = last_tracker->last_armor.center.x;
                last_armor_timestamp = last_tracker->last_timestamp;
                auto spin_movement = new_armor_center - last_armor_center;  // 中心x坐标： 新 - 旧

                if (abs(spin_movement) > 10 && new_armor_timestamp == t && last_armor_timestamp == t) {

                    // 若无该元素则插入新元素
                    if (spin_score_map.count(armor_id) == 0) {
                        // 第一次进入反陀螺状态将第一帧存起来
                        spin_T = 0;
                        jump_tracker.jump_armor = enemy_armor;
                        jump_tracker.jump_time = t;
                        jump_trackers.push_back(jump_tracker);
                        spin_score_map[armor_id] = 1000 * spin_movement / abs(spin_movement);
                    }
                        //  若已有该元素且目前旋转方向与记录不同,则对目前分数进行减半惩罚
                    else if (spin_movement * spin_score_map[armor_id] < 0) {
                        spin_score_map[armor_id] *= 0.5;
                    }
                        // 若已有该元素则更新元素
                    else {
                        spin_score_map[armor_id] = anti_spin_max_r_multiple * spin_score_map[armor_id];
                    }
                }
            }
        }
    }
    updateSpinScore();
}
//}

Circle ArmorTracker::fitCircle(const cv::Point2f &x1, const cv::Point2f &x2, const cv::Point2f &x3) {
    std::vector<double> x_data{x1.x, x2.x, x3.x};
    std::vector<double> y_data{x1.y, x2.y, x3.y};
    //获取数据点个数
    int n = x_data.size();

    //创建矩阵A,B,X
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd B(n);
    Eigen::VectorXd X(3);

    //根据数据点填充矩阵A和B
    for (int i = 0; i < n; i++)
    {
        A(i, 0) = x_data[i];
        A(i, 1) = y_data[i];
        A(i, 2) = 1;
        B(i) = -(x_data[i] * x_data[i] + y_data[i] * y_data[i]);
    }

    //求解线性方程组Ax=B，得到X=[a,b,c]
    X = A.colPivHouseholderQr().solve(B);
    Circle circle;
    //根据X计算圆心和半径
    circle.x = -X(0) / 2;
    circle.y = -X(1) / 2;
    circle.r = sqrt((X(0) * X(0) + X(1) * X(1)) / 4 - X(2));

    return circle;
}