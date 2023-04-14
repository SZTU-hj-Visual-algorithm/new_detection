#include "armor_track.h"
// #include <opencv2/calib3d.hpp>
// #include <opencv2/core/eigen.hpp>

 #define ANTI_SPIN


    ArmorTracker::ArmorTracker() {

        cv::FileStorage fs("/home/hanjiang/sentry_ros/src/robot_detection/vision_data/track_data.yaml", cv::FileStorage::READ);

        KF.initial_KF();

        locate_target = false;
        wait_start = true;
        enemy_armor = Armor();

        is_aim_virtual_armor = false;

        tracker_state = MISSING;
        tracking_id = 0;

        find_aim_cnt = 0;
        find_threshold = (int)fs["find_threshold"];

        lost_aim_cnt = 0;
        lost_threshold = (int)fs["lost_threshold"];

        new_old_threshold = (double)fs["new_old_threshold"];

//        switch_enemy_cnt = 0;
//        switch_enemy_threshold = (int)fs["switch_enemy_threshold"];
//        max_effective_distance = (double)fs["max_effective_distance"];

        anti_spin_max_r_multiple = (double)fs["anti_spin_max_r_multiple"];
        anti_spin_judge_low_thres = (int)fs["anti_spin_judge_low_thres"];
        anti_spin_judge_high_thres = (int)fs["anti_spin_judge_high_thres"];
        max_delta_t = (int)fs["max_delta_t"];

        fs.release();
    }

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
//        switch_enemy_cnt = 0;
    }

    // 初始化，选择最优装甲板，设置卡尔曼的F和x_1，当没有目标时返回false，选一个最优目标返回true
    bool ArmorTracker::initial(std::vector<Armor> find_armors)
    {
        if(find_armors.empty())
        {
            fmt::print("no track enemy!");
            return false;
        }

        // select enemy : for sentry to find nearest aim
        double distance = DBL_MAX;
        for (auto & armor : find_armors)
        {
            if(armor.id == 1)
            {
                enemy_armor = armor;
                break;
            }
            double dis_tmp = armor.world_position.norm();
            if (dis_tmp < distance)
            {
                enemy_armor = armor;
                distance = dis_tmp;
            }
        }
        tracker_state = DETECTING;
        tracking_id = enemy_armor.id;

        // initial KF --- x_post
        KF.initial_KF();
        enemy_armor.world_position = AS.pixel2imu(enemy_armor);
        KF.setXPost(enemy_armor.world_position);
        Singer.setXpos({enemy_armor.world_position(0,0),enemy_armor.world_position(1,0)});
//        fmt::print("{}      {}",enemy_armor.camera_position.norm(), enemy_armor.world_position.norm());
//        fmt::print("enemy_armor.camera_position:  {}, {}, {}",
//                   enemy_armor.camera_position.transpose()[0], enemy_armor.camera_position.transpose()[1],
//                   enemy_armor.camera_position.transpose()[2]);
//        fmt::print("{}       {}       {}",AS.ab_roll, AS.ab_pitch, AS.ab_yaw);
//        fmt::print("enemy_armor.world_position:  {}, {}, {}",enemy_armor.world_position.transpose()[0], enemy_armor.world_position.transpose()[1],
//                   enemy_armor.world_position.transpose()[2]);
        return true;
    }

    bool ArmorTracker::switchEnemy(std::vector<Armor> find_armors)
    {
//        Armor tmp;
//        // genju juli shezhi fadanmoshi
//        if(enemy_armor.camera_position.norm() > max_effective_distance)
//        {
//            reset();
//            return false;
//        }
//        else
//        {
//            // TODO: 如果另一台车比跟踪的进应该怎么选？
//            if(!find_armors.empty())
//            {
//                for(auto armor : find_armors)
//                {
//                    if(armor.id == 1 && tracking_id != 1)
//                    {
//                        tmp = armor;
//                        switch_enemy_cnt++;
//                    }
//                    else
//                    {
//                        switch_enemy_cnt = 0;
//                    }
//                }
//            }
//            else
//            {
//                switch_enemy_cnt = 0;
//            }
//
//            return false;
//        }
//
//        if(switch_enemy_cnt >= switch_enemy_threshold)
//        {
//            enemy_armor = tmp;
//
//            tracker_state = DETECTING;
//            tracking_id = enemy_armor.id;
//            // initial KF --- x_post
//            KF.initial_KF();
//            enemy_armor.world_position = AS.pixel2imu(enemy_armor);
//            KF.setXPost(enemy_armor.world_position);
//            Singer.setXpos({enemy_armor.world_position(0,0),enemy_armor.world_position(1,0)});
//            return true;
//        }
    }

    // dt是两帧之间时间间隔, 跟得住目标  用预测来做匹配，确定跟踪器状态
    bool ArmorTracker::selectEnemy(std::vector<Armor> find_armors, double dt)
    {
        KF.setF(dt);
        predicted_enemy = KF.predict();

        Armor matched_armor;
        bool matched = false;

#ifdef ANTI_SPIN
        SpinHeading spin_status;
        if (!find_armors.empty()) {
            spin_status = spin_status_map[tracking_id];
    //        std::cout<<"[spin_status]: " << spin_status <<std::endl;
            if (spin_status == UNKNOWN) {
    //            if (spin_status == UNKNOWN) {
    //            std::cout << "unkown" << std::endl;
                double min_position_diff = DBL_MAX;
                for (auto &armor: find_armors) {
                    armor.world_position = AS.pixel2imu(armor);
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
    //                    std::cout << "[delta_dist2]: " << dis_tmp << std::endl;
                        if (armor.id == tracking_id && dis_tmp < new_old_threshold)
                        {
                            matched = true;
                            //TODO:封装
                            KF.initial_KF();
                            Eigen::Matrix<double, 6, 1> pos_speed;
                            pos_speed << armor.world_position, predicted_enemy.tail(3);
                            KF.setPosAndSpeed(armor.world_position, predicted_enemy.tail(3));
                            Singer.Reset({armor.world_position[0],armor.world_position[1]});
                            predicted_enemy = pos_speed;

                            same_armor_distance = dis_tmp;
                            matched_armor = armor;
                            break;
                        }
                    }
                }
                is_vir_armor = false;
            }
            else  // spin_status != UNKNOWN
            {
    //            std::cout << "spin" << std::endl;
                jump_tracker = {};
                auto ID_candiadates = trackers_map.equal_range(tracking_id);
                auto backward_with_same_ID = trackers_map.count(tracking_id);  // 已存在类型的预测器数量

                std::vector<Armor> final_armors;
                for (auto &armor: find_armors) {
                    if (armor.id == tracking_id)  // 同一帧图像的装甲板
                    {
                        armor.world_position = AS.pixel2imu(armor);
                        final_armors.push_back(armor);
                        // 储存装甲板
                        if (history_armors.size() <= max_history_len)
                        {
                            history_armors.push_back(armor);
                        } else
                        {
                            history_armors.pop_front();
                            history_armors.push_back(armor);
                        }
                        matched = true;
    //                    std::cout << "match" << std::endl;
                    } else {
                        continue;
                    }
                }

                //若存在一块装甲板
                if (final_armors.size() == 1)
                {
                    int kf_state = 0; // 1: 1->1(跳变); 2: 1->1/2->1(无跳变); 3: 虚拟装甲板
                    matched_armor = final_armors.at(0);
                    if (last_final_armors_size == 1 &&  // 单装甲板跳变；同时发生装甲板出现和消失
                        backward_with_same_ID == 1 &&
                        (matched_armor.world_position - trackers_map.find(tracking_id)->second.last_armor.world_position).norm()>new_old_threshold)
                    {
                        fmt::print("---1->1(跳变)---");
                        if (milliseconds_duration (t - jump_trackers.at(0).jump_time).count()*2 > spin_T){ //TODO: why to do *2?
                            if (spin_T == 0)
                                spin_T = milliseconds_duration (t - jump_trackers.at(0).jump_time).count();
                            else
                                spin_T = (milliseconds_duration (t - jump_trackers.at(0).jump_time).count() + spin_T) / 2;

                        }
                        jump_tracker.jump_armor = matched_armor;
                        jump_tracker.jump_time = t;
                        jump_trackers.push_back(jump_tracker);
                        disappear_tracker.disappear_armor = jump_trackers.at(0).jump_armor;

                        jump_trackers.erase(jump_trackers.begin());

                        if(is_vir_armor)
                        {
                            kf_state = 2;
                            is_aim_virtual_armor = false;
                        }
                        else
                            kf_state = 1;
                    }
                    else  // 二/一 到一，追踪；无变化
                    {
                        if(last_final_armors_size == 2)
                        {
                            fmt::print("---2->1(无跳变)---");

                            if (milliseconds_duration (t - jump_trackers.at(0).jump_time).count()*2 > spin_T){
                                if (spin_T == 0)
                                    spin_T = milliseconds_duration (t - jump_trackers.at(0).jump_time).count();
                                else
                                    spin_T = (milliseconds_duration (t - jump_trackers.at(0).jump_time).count() + spin_T) / 2;

                            }
                            double min_delta_dist = 0;
                            double min_delta_t = 1e9;

                            std::multimap<int, SpinTracker>::iterator best_candidate;
                            auto candiadates = trackers_map.equal_range(tracking_id);
                            for (auto iter = candiadates.first; iter != candiadates.second; ++iter) {
                                auto delta_t = milliseconds_duration (t - (*iter).second.last_timestamp).count();  // TODO: 这里的last_timestamp怎么付的值,这个时间戳是出现时记录还是每一帧更新，如果出现时记录，时间最近和距离最近会矛盾
                                auto delta_dist = (matched_armor.world_position - (*iter).second.last_armor.world_position).norm();
                                // 在同一位置存在过装甲板且时间最接近设为最高优先级，  //TODO: 同一位置存在这样设条件？ 二选一怎么选，大于一个装甲板宽度的阈值,时间最接近,,距离最近是旧的不一定是最新的，
                                if (delta_dist >= new_old_threshold && delta_dist >= min_delta_dist && delta_t < min_delta_t)  // TODO: 距离需要调试
                                {
                                    min_delta_dist = delta_dist;
                                    min_delta_t = delta_t;
                                    disappear_tracker.disappear_armor = (*iter).second.last_armor;
                                }
                            }
                        }
                        kf_state = 2;
                    }


                    // 如果超过限制时间，构造虚拟装甲板。
                    double delay_time = AS.getFlyTime(matched_armor.world_position) * 1000 + 20; // TODO：改为子弹飞行时间+系统时延+the time of move to disappear
                    fmt::print("delay_time: {}", AS.getFlyTime(matched_armor.world_position) * 1000);

    //                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
    //                auto ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(jump_trackers.front().jump_time.time_since_epoch());
    //                std::cout << "t: " << ms.count() << std::endl;
    //                std::cout << "jump_time: " << ms2.count() << std::endl;
    //                std::cout << "jump_tracker_size: " << jump_trackers.size() << std::endl;
                    fmt::print("duration: {}", milliseconds_duration (t - jump_trackers.back().jump_time).count());
                    fmt::print("T: {}", spin_T);

                    if(spin_T > 0 &&
                       milliseconds_duration (t - jump_trackers.back().jump_time).count() >
                       milliseconds_duration (spin_T - delay_time).count())
                    {
                        fmt::print("---generate virtual armor---");
                        real_armor = matched_armor;
    //                    std::cout << "---save tracker--- " << std::endl;
                        /// 根据当前装甲板、当前装甲板出现的第一帧装甲板、以及上一个消失的装甲板坐标用最小二乘法拟合圆，计算圆心、半径
                        std::vector<cv::Point2f> pts;
                        for (const auto& history_armor : history_armors)
                        {
                            pts.push_back(AS.Vector3d2point2f(history_armor.world_position));  // 将Eigen::Vector3d 转为 cv::point2f
                        }

                        pts.push_back(AS.Vector3d2point2f(jump_trackers.back().jump_armor.world_position));
                        pts.push_back(AS.Vector3d2point2f(disappear_tracker.disappear_armor.world_position));
    //                    cv::Mat image = cv::Mat::ones(300, 300, CV_8UC3);
    //                    cv::Scalar color1(255, 0, 0); // 蓝色
                        Circle circle;
                        AS.circleLeastFit(pts, circle.x, circle.y, circle.r);
                        circle.r = (circle.r + last_r) / 2;
    //                    std::cout << "[r]: " << circle.r << std::endl;
                        if (circle.r > 0.35){
                            circle.r = last_r;
                            matched_armor.world_position = last_position; //TODO:优化
                        }
                        else
                        {/// 转化世界坐标
                            double new_world_position_x;
                            double new_world_position_y;
                            // 该点与圆心连线和水平方向的夹角θ
                            double theta = atan2(matched_armor.world_position[1] - circle.y,
                                                 matched_armor.world_position[0] - circle.x);
                            // 逆时针-，顺时针+
                            if (spin_status == COUNTER_CLOCKWISE) {
                                double alpha = theta - M_PI / 2;
                                new_world_position_x = circle.r * cos(alpha) + circle.x;
                                new_world_position_y = circle.r * sin(alpha) + circle.y;
                            } else {
                                double beta = theta + M_PI / 2;
                                new_world_position_x = circle.r * cos(beta) + circle.x;
                                new_world_position_y = circle.r * sin(beta) + circle.y;
                            }
                            matched_armor.world_position.x() = new_world_position_x;
                            matched_armor.world_position.y() = new_world_position_y;
                        }
                        kf_state = 3;
                        vir_num++;
                        if (vir_num > vir_max)
                        {
                            kf_state = 2;
                            spin_T = 0;
                        }
                    }
                    /// 初始化/更新KF参数
                    if(kf_state == 2)
                    {
                        predicted_enemy = KF.update(matched_armor.world_position);
                        is_vir_armor = false;
                    }
                    else if(kf_state == 3 && is_vir_armor)
                    {
                        predicted_enemy = KF.update(matched_armor.world_position);
                    }
                    else
                    {
                        KF.initial_KF();
                        Eigen::Matrix<double, 6, 1> pos_speed;
                        pos_speed << matched_armor.world_position, predicted_enemy.tail(3);
                        KF.setPosAndSpeed(matched_armor.world_position, predicted_enemy.tail(3));
                        predicted_enemy = pos_speed;
                        if(kf_state == 3 && !is_vir_armor)
                        {
                            is_aim_virtual_armor = true; // 出现虚拟装甲板后转过去
                            is_vir_armor = true;
                        }
                        if(kf_state == 1)is_vir_armor = false;
                    }

                    if (jump_trackers.size() == 2)  // 维护jump_trackers
                    {
                        jump_trackers.erase(jump_trackers.begin());
                    }
    //                std::cout << "---generate over--- " << std::endl;

                }
                    //// 若存在两块装甲板
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
                    if (last_final_armors_size == 1)
                    {
                        fmt::print("---1->2(跳变)---");

                        jump_tracker.jump_armor = matched_armor;
                        jump_tracker.jump_time = t;
                        jump_trackers.push_back(jump_tracker);
                        if(is_vir_armor)
                        {
                            predicted_enemy = KF.update(matched_armor.world_position);
                        }
                        else
                        {
                            KF.initial_KF();
                            Eigen::Matrix<double, 6, 1> pos_speed;
                            pos_speed << matched_armor.world_position, predicted_enemy.tail(3);
                            KF.setPosAndSpeed(matched_armor.world_position, predicted_enemy.tail(3));
                            predicted_enemy = pos_speed;
                        }
                    }
                    else  // 二到二，追踪；无变化
                    {
                        fmt::print("---2->2(无跳变)---");

                        predicted_enemy = KF.update(matched_armor.world_position);
                    }
                    if (is_vir_armor)
                    {
                        is_aim_virtual_armor = false;
                        is_vir_armor = false;
                    }
                }
                last_final_armors_size = final_armors.size();
                last_position = matched_armor.world_position;
            }
        }

#else  // NOT_ANTI_SPIN
        if(!find_armors.empty())
        {
            double min_position_diff = DBL_MAX;
            for(auto & armor : find_armors)
            {
                armor.world_position = AS.pixel2imu(armor);
                Eigen::Vector3d pre = predicted_enemy.head(3);
                double position_diff = (pre - armor.world_position).norm();

                if (position_diff < min_position_diff)
                {
                    min_position_diff = position_diff;
                    matched_armor = armor;
                }
            }
            // std::cout<<"min_position_diff(m):    "<<min_position_diff<<std::endl;
            // std::cout<<"match_id: "<<matched_armor.id<<std::endl;
            // 这个相同id对遮挡情况似乎不好
            if (min_position_diff < new_old_threshold && matched_armor.id == tracking_id)
            {
                // std::cout<<"yes"<<std::endl;
                matched = true;
                predicted_enemy = KF.update(matched_armor.world_position);
            }
            else
            {
                // std::cout<<"no"<<std::endl;
                // 本帧内是否有相同ID
                double same_armor_distance = DBL_MAX;
                for (auto & armor : find_armors)
                {
                    double dis_tmp = (enemy_armor.world_position - armor.world_position).norm();
                    if (armor.id == tracking_id
                        && dis_tmp < same_armor_distance
                            )
                    {
                        matched = true;
                        KF.initial_KF();
                        Eigen::VectorXd position_speed(6);
                        position_speed << armor.world_position, predicted_enemy.tail(3);
                        KF.setPosAndSpeed(armor.world_position, predicted_enemy.tail(3));

                        Singer.setXpos({armor.world_position[0],armor.world_position[1]});

                        predicted_enemy = position_speed;
                        matched_armor = armor;

                        same_armor_distance = dis_tmp;
                        std::cout<<"track_sameID_initial!!"<<std::endl;
                    }
                }
            }

        }

#endif // ANTI_SPIN

        if (matched)
        {
            // std::cout<<"matched"<<std::endl;
            enemy_armor = matched_armor;
        }
        // predicted_position = predicted_enemy.head(3);
        // predicted_speed = predicted_enemy.tail(3);

        if (tracker_state == DETECTING)
        {
            // DETECTING
            if (matched)
            {
                find_aim_cnt++;
                if (find_aim_cnt > find_threshold)
                {
                    find_aim_cnt = 0;
                    tracker_state = TRACKING;
                }
            }
            else
            {
                find_aim_cnt = 0;
                tracker_state = MISSING;
            }
        }
        else if (tracker_state == TRACKING)
        {
            if (!matched)
            {
                tracker_state = LOSING;
                lost_aim_cnt++;
            }
        }
        else if (tracker_state == LOSING)
        {
            if (!matched)
            {
                lost_aim_cnt++;
                if (lost_aim_cnt > lost_threshold)
                {
                    lost_aim_cnt = 0;
                    tracker_state = MISSING;
                    history_armors.clear(); // 目标丢失后重置历史装甲板
                }
            }
            else
            {
                tracker_state = TRACKING;
                lost_aim_cnt = 0;
            }
        }

        if (tracker_state == MISSING)
        {
            reset();
            return false;
        }

        KF.setPosAndSpeed(enemy_armor.world_position,predicted_enemy.tail(3));
        if(tracker_state == LOSING)
        {
            enemy_armor.world_position = predicted_enemy.head(3);
            KF.setPosAndSpeed(enemy_armor.world_position,predicted_enemy.tail(3));

        }

        return true;
    }

    // 对处于跟踪和正在丢失状态时做 预测，引入各种时间
    bool ArmorTracker::estimateEnemy(double dt)
    {
#ifdef ANTI_SPIN
        // 如果有虚拟装甲板的那一刻就重置Singer，然后把虚拟点接入Singer重新开始预测
        if(is_aim_virtual_armor)
        {
            Singer.setXpos({enemy_armor.world_position[0],enemy_armor.world_position[1]});
        }
#endif // ANTI_SPIN

        // 对跟踪状态和正在丢失状态时做预测
        if(tracker_state == TRACKING || tracker_state == LOSING)
        {
            double fly_time = AS.getFlyTime(enemy_armor.world_position);
            ////////////////Singer predictor//////////////////////////////
            if(!Singer.SingerPrediction(dt,fly_time,
                                        enemy_armor.world_position,
                                        predicted_position))
            {
                Singer.Reset({enemy_armor.world_position(0,0),enemy_armor.world_position(1,0)});
                //            Singer.setXpos({enemy_armor.world_position(0,0),enemy_armor.world_position(1,0)});
                std::cerr<<"[predict value illegal!!! Fix in origin value]"<<std::endl;
                return false;
            }
            ////////////////Singer predictor//////////////////////////////
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
                // Singer.Reset({enemy_armor.world_position(0,0),enemy_armor.world_position(1,0)});
                //            Singer.setXpos({enemy_armor.world_position(0,0),enemy_armor.world_position(1,0)});
                // std::cerr<<"[predict value illegal!!! Fix in origin value]"<<std::endl;
                return false;
            }
            ////////////////Singer predictor//////////////////////////////

            // 检测状态还给false就会一直进入initial函数，历史代码是这么写的，没问题
            // locate_target = false;
            return false;
        }
        else
        {
            // reset();  // 前一个函数变成MISSING会之间返回false，这个函数里这个判断无效
            // locate_target = false;
            return false;
        }

    }

    // 返回false保持现状，返回true开始控制    ddt --- chrono
    bool ArmorTracker::locateEnemy(cv::Mat src, std::vector<Armor> armors, chrono_time time)
    {
        src.copyTo(_src);

        if(!locate_target)
        {
            if(initial(armors))
            {
                locate_target = true;
                fmt::print("00");
            }
            else
            {
                locate_target = false;
                fmt::print("0");
            }
            return false;
        }
        else
        {
            if (wait_start)
            {
                t = time;
                wait_start = false;
                std::cout<<"wait_start"<<std::endl;
                return false;
            }
            double dt = seconds_duration (time - t).count();
            t = time;

            // if(switchEnemy(armors)) { return false; }

            if(!selectEnemy(armors,dt))  { std::cout<<"1"<<std::endl; return false; }

#ifdef ANTI_SPIN
            if (tracker_state == TRACKING) { spin_detect(); std::cout<<"2"<<std::endl;}
#endif

            if(!estimateEnemy(dt)) { fmt::print("3"); return false; }

            //
            if(is_aim_virtual_armor)
            {
                Eigen::Vector3d rpy = AS.getAngle(enemy_armor.world_position);
                pitch = rpy[1];
                yaw   = rpy[2];
                pitch = round(pitch * 100)/100;
                fmt::print("4");
                return false;  // TODO: fire or fail
            }
            else
            {
                Eigen::Vector3d rpy = AS.getAngle(predicted_position);
                pitch = rpy[1];
                yaw   = rpy[2];
                pitch = round(pitch * 100)/100;
                std::cout<<"5"<<std::endl;
            }

            Eigen::Vector3d rpy = AS.getAngle(enemy_armor.world_position);
            pitch = rpy[1];
            yaw   = rpy[2];

            return true;
        }

    }

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

    /**
     * @brief 维护反陀螺状态map
     * @return 自然衰减状态分数，将分数低于阈值的去除，高于最大阈值的限制在最大值
     */
    bool ArmorTracker::updateSpinScore()
    {
        for (auto score = spin_score_map.begin(); score != spin_score_map.end();)
        {
            SpinHeading spin_status;
            //        std::cout << "score1: " << (*score).second << std::endl;

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
                is_anti = false;
                continue;
            }

            if (spin_status != UNKNOWN)
                (*score).second = 0.933 * (*score).second;
            else
                (*score).second = 0.987 * (*score).second;
            // 当小于该值时移除该元素
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
                if(!is_anti)
                {
                    spin_T = 0;
                    last_final_armors_size = 0;
                    jump_trackers.clear();
                    jump_tracker.jump_armor = enemy_armor;
                    jump_tracker.jump_time = t;
                    jump_trackers.push_back(jump_tracker);
                    is_anti = true;
                }
            }
            //        std::cout << "score2: " << (*score).second << std::endl;

            ++score;
        }

        // cout<<"++++++++++++++++++++++++++"<<endl;
        // for (auto status : spin_status_map)
        // {
        //     cout<<status.first<<" : "<<status.second<<endl;
        // }
        //    std::cout << "score over" << std::endl;

        return true;
    }

    /**
     * @brief 反陀螺状态检测
     */
    void ArmorTracker::spin_detect() {
        Armor detect_armor;
        if(!is_vir_armor)
        {
            detect_armor = enemy_armor;
            //        std::cout << "---enemy_armor--- " << std::endl;
        }
        else
        {
            detect_armor = real_armor;
            //        std::cout << "---real_armor--- " << std::endl;
        }
        int armor_id = detect_armor.id;
        new_armors_cnt_map.clear();
        auto predictors_with_same_key = trackers_map.count(armor_id);  // 已存在类型的预测器数量
        if (predictors_with_same_key == 1)
        {
            auto candidate = trackers_map.find(armor_id);  // 原有的同ID装甲板
            auto delta_dist = (detect_armor.world_position - (*candidate).second.last_armor.world_position).norm();  // 距离
            //        std::cout << "[delta_dist1]: " << delta_dist << std::endl;
            if (delta_dist < new_old_threshold) {
                (*candidate).second.update_tracker(detect_armor, t);  // 更新装甲板
            }
            else {
                SpinTracker spinTracker(detect_armor, t); // 同类型不同位置创建
                trackers_map.insert(std::make_pair(armor_id, spinTracker));
                new_armors_cnt_map[armor_id]++;
                //            std::cout << "-----new_map-----" << std::endl;
                //            for (auto & it : trackers_map)
                //            {
                //                std::cout << it.first << " -> " << it.second.is_initialized << "\n";
                //            }
            }
        }
        else
        {
            // 1e9无实际意义，仅用于非零初始化
            double min_delta_dist = 1e9;
            double min_delta_t = 1e9;

            bool is_best_candidate_exist = false;
            std::multimap<int, SpinTracker>::iterator best_candidate;
            auto candiadates = trackers_map.equal_range(armor_id);
            for (auto iter = candiadates.first; iter != candiadates.second; ++iter) {
                auto delta_t = milliseconds_duration (t - (*iter).second.last_timestamp).count();
                auto delta_dist = (detect_armor.world_position - (*iter).second.last_armor.world_position).norm();

                // 在同一位置存在过装甲板且时间最接近设为最高优先级，
                if (delta_dist <= new_old_threshold && delta_dist <= min_delta_dist && delta_t < min_delta_t)  // 距离需要调试
                {
                    min_delta_dist = delta_dist;
                    min_delta_t = delta_t;
                    best_candidate = iter;
                    is_best_candidate_exist = true;
                    //                std::cout << "2->1" << std::endl;
                }
            }
            if (is_best_candidate_exist)
            {
                (*best_candidate).second.update_tracker(detect_armor, t);
            }
            else
            {   // first time or 2->1
                SpinTracker spinTracker(detect_armor, t);
                trackers_map.insert(std::make_pair(static_cast<int&&>(armor_id), static_cast<SpinTracker&&>(spinTracker)));
            }
        }

        //    std::cout << "[MAP_size]: "<< trackers_map.size() << std::endl;
        //
        if (!trackers_map.empty()) {
            //维护预测器Map，删除过久之前的装甲板
            for (auto iter = trackers_map.begin(); iter != trackers_map.end();) {
                //删除元素后迭代器会失效，需先行获取下一元素
                auto next = iter;
                if (milliseconds_duration(t - (*iter).second.last_timestamp).count() > max_delta_t)  //TODO：时间需要测试
                {
                    //                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
                    //                std::cout<<"timestamp : "<< ms.count() << std::endl;
                    //                auto ms2 = std::chrono::duration_cast<std::chrono::milliseconds>((*iter).second.last_timestamp.time_since_epoch());
                    //                std::cout<<"last_timestamp : "<< ms2.count() << std::endl;
                    //                std::cout<<"duration_cast : "<< milliseconds_duration (t - (*iter).second.last_timestamp).count() << std::endl;
                    //                std::cout << "[MAP_size]: "<< trackers_map.size() << std::endl;
                    next = trackers_map.erase(iter);
                }
                else
                    ++next;
                iter = next;
            }
        }
        //    std::cout << "[MAP_size_2]: "<< trackers_map.size() << std::endl;

        if (new_armors_cnt_map[armor_id] == 1) {
            //        std::cout << "-----spin detect-----" << std::endl;
            auto same_armors_cnt = trackers_map.count(armor_id);  // 相同的装甲板数量
            //        std::cout << "[same_armors_cnt] : "<< same_armors_cnt << std::endl;

            if (same_armors_cnt == 2) {
                //遍历所有同Key预测器，确定左右侧的Tracker
                SpinTracker *new_tracker = nullptr;
                SpinTracker *last_tracker = nullptr;
                double last_armor_center;
                //            chrono_time last_armor_timestamp;
                double new_armor_center;
                //            chrono_time new_armor_tisubscribermestamp;
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
                    //                new_armor_timestamp = new_tracker->last_timestamp;
                    last_armor_center = last_tracker->last_armor.center.x;
                    //                last_armor_timestamp = last_tracker->last_timestamp;
                    auto spin_movement = new_armor_center - last_armor_center;  // 中心x坐标： 新 - 旧
                    //                std::cout << "[spin_movement]: " << spin_movement << std::endl;

                    if (abs(spin_movement) > 10) {


                        // 若无该元素则插入新元素
                        if (spin_score_map.count(armor_id) == 0) {
                            spin_score_map[armor_id] = 1000 * spin_movement / abs(spin_movement);
                            //                        std::cout << "create new score" << std::endl;
                        }
                            //  若已有该元素且目前旋转方向与记录不同,则对目前分数进行减半惩罚
                        else if (spin_movement * spin_score_map[armor_id] < 0) {
                            spin_score_map[armor_id] *= 0.3;
                            //                        std::cout << "-----cut score-----" << std::endl;

                        }
                            // 若已有该元素则更新元素
                        else {
                            //                        std::cout << "update score" << std::endl;
                            spin_score_map[armor_id] = anti_spin_max_r_multiple * spin_score_map[armor_id];
                        }
                        //                    for (auto & it : spin_score_map)
                        //                    {
                        //                        std::cout << it.first << " -> " << it.second << "\n";
                        //                    }
                    }
                }
            }
        }
        updateSpinScore();
        //    for (auto & it : spin_score_map)
        //    {
        //        std::cout << it.first << " -> " << it.second << "\n";
        //    }
    }

void ArmorTracker::show()
{
    // show track state on img's ru
    switch (tracker_state)
    {
        case 0: // MISSING
            cv::putText(_src,"MISSING   ", cv::Point2f(1280 - 200,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
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

    // show receive data on img's ru
    cv::putText(_src,"p : "+std::to_string(AS.ab_pitch),cv::Point2f(1280 - 200,90),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(_src,"y : "+std::to_string(AS.ab_yaw),cv::Point2f(1280 - 200,120),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(_src,"r : "+std::to_string(AS.ab_roll),cv::Point2f(1280 - 200,150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);

    // show send data on img's lu
    cv::putText(_src,"PITCH    : "+std::to_string(pitch),cv::Point2f(0,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(_src,"YAW      : "+std::to_string(yaw),cv::Point2f(0,90),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(_src,"ROLL     : "+std::to_string(AS.ab_roll),cv::Point2f(0,120),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
    cv::putText(_src,"DISTANCE : "+std::to_string(enemy_armor.camera_position.norm())+"m",cv::Point2f(0,150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);

    // 展示选择到的装甲板
    cv::Point2f vertice_enemy[4];
    enemy_armor.points(vertice_enemy);
    for (int i = 0; i < 4; i++) {
        line(_src, vertice_enemy[i], vertice_enemy[(i + 1) % 4], CV_RGB(255, 0, 255),2,cv::LINE_8);
    }
    std::string information = std::to_string(enemy_armor.id) + ":" + std::to_string(enemy_armor.confidence*100) + "%";
    putText(_src, information,enemy_armor.armor_pt4[3],cv::FONT_HERSHEY_SIMPLEX,2,CV_RGB(255,0,255),1,3);

    // 展示要匹配的预测装甲板
    cv::Point2f armor_match_center = AS.imu2pixel(predicted_enemy.head(3));
    // cv::circle(src,armor_match_center,5,cv::Scalar(255,0,0),-1);
    cv::RotatedRect armor_match = cv::RotatedRect(armor_match_center,
                                                  cv::Size2f(enemy_armor.size.width,enemy_armor.size.height),
                                                  enemy_armor.angle);

    cv::Point2f vertice_armor_match[4];
    enemy_armor.points(vertice_armor_match);
    for (int i = 0; i < 4; ++i) {
        line(_src, vertice_armor_match[i], vertice_armor_match[(i + 1) % 4], CV_RGB(0, 255, 0),2,cv::LINE_8);
    }

    cv::putText(_src,"camera_position",cv::Point2f(0,1024 - 150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(_src,"x  : "+std::to_string(enemy_armor.camera_position[0]),cv::Point2f(0,1024 - 120),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(_src,"y  : "+std::to_string(enemy_armor.camera_position[1]),cv::Point2f(0,1024 - 90),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(_src,"z  : "+std::to_string(enemy_armor.camera_position[2]),cv::Point2f(0,1024 - 60),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);

    cv::putText(_src,"world_position",cv::Point2f(1280 - 200,1024 - 150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(_src,"x  : "+std::to_string(enemy_armor.world_position[0]),cv::Point2f(1280 - 200,1024 - 120),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(_src,"y  : "+std::to_string(enemy_armor.world_position[1]),cv::Point2f(1280 - 200,1024 - 90),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
    cv::putText(_src,"z  : "+std::to_string(enemy_armor.world_position[2]),cv::Point2f(1280 - 200,1024 - 60),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);


    // 用预测位置为中心点，选择的装甲板画框
    cv::Point2f armor_singer_center = AS.imu2pixel(predicted_position);
    // armor_singer_center.y = armor_match_center.y;
    cv::RotatedRect armor_singer = cv::RotatedRect(armor_singer_center,
                                                   cv::Size2f(enemy_armor.size.width,enemy_armor.size.height),
                                                   enemy_armor.angle);

    cv::Point2f vertice_armor_singer[4];
    armor_singer.points(vertice_armor_singer);
    for (int m = 0; m < 4; ++m)
    {
        line(_src, vertice_armor_singer[m], vertice_armor_singer[(m + 1) % 4], CV_RGB(255, 255, 0),2,cv::LINE_8);
    }

//    cv::putText(_src,"FPS      : "+std::to_string(1/delta_tt),cv::Point2f(0,30),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);

    cv::circle(_src,cv::Point(640,512),5,cv::Scalar(150,100,255),-1);


    imshow("final_result",_src);

}
