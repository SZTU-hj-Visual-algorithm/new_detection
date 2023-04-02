#include "armor_track.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// #define DRAW_CENTER_CIRCLE
// #define DRAW_LOCATE_MATCH_ARMOR
// #define DRAW_BULLET_POINT

#define SHOOT_DELAY 0.1

//namespace robot_detection {

    ArmorTracker::ArmorTracker() {

        cv::FileStorage fs("../other/track_data.yaml", cv::FileStorage::READ);

        KF.initial_KF();

        locate_target = false;
        enemy_armor = Armor();

        tracker_state = MISSING;
        tracking_id = 0;

        find_aim_cnt = 0;
        find_threshold = (int)fs["find_threshold"];

        lost_aim_cnt = 0;
        lost_threshold = (int)fs["lost_threshold"];

        new_old_threshold = (double)fs["new_old_threshold"];

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
    }

    // 初始化，选择最优装甲板，设置卡尔曼的F和x_1，当没有目标时返回false，选一个最优目标返回true
    bool ArmorTracker::initial(std::vector<Armor> find_armors)
    {
        if(find_armors.empty())
        {
            return false;
        }


        sort(find_armors.begin(),find_armors.end(),
            [](Armor &armor1,Armor &armor2){return armor1.grade > armor2.grade;});

        // select enemy
        enemy_armor = find_armors[0];
        tracker_state = DETECTING;
        tracking_id = enemy_armor.id;

        // initial KF --- x_post
        KF.initial_KF();
        enemy_armor.world_position = AS.pixel2imu(enemy_armor);
        KF.setXPost(enemy_armor.world_position);
        // TODO: 这里用的是x和y是水平面的,顺序和数据是否正确
        Singer.setXpos({enemy_armor.world_position(0,0),enemy_armor.world_position(1,0)});
        // std::cout<<enemy_armor.camera_position.norm()<<"     "<<enemy_armor.world_position.norm()<<std::endl;
        // std::cout<<"enemy_armor.camera_position:  "<<enemy_armor.camera_position.transpose()<<std::endl;
        // std::cout<<AS.ab_roll<<"     "<<AS.ab_pitch<<"     "<<AS.ab_yaw<<std::endl;
        // std::cout<<"enemy_armor.world_position:  "<<enemy_armor.world_position.transpose()<<std::endl;
        return true;
    }

    // dt是两帧之间时间间隔, 跟得住目标  用预测来做匹配，确定跟踪器状态
    bool ArmorTracker::selectEnemy(std::vector<Armor> find_armors, double dt)
    {
        KF.setF(dt);
        predicted_enemy = KF.predict();

#ifdef DRAW_LOCATE_MATCH_ARMOR
        cv::Mat pre_armor_rrt;
        _src.copyTo(pre_armor_rrt);
        // after update 上一帧预测得出本帧的位置
        cv::Point2f pre_armor_center = AS.imu2pixel(predicted_enemy.head(3));
        cv::RotatedRect armor_rrect = cv::RotatedRect(pre_armor_center,
                                                    cv::Size2f(enemy_armor.size.width,enemy_armor.size.height),
                                                    enemy_armor.angle);
        cv::Point2f vertice_armors[4];
        armor_rrect.points(vertice_armors);
        for (int m = 0; m < 4; ++m)
        {
            line(pre_armor_rrt, vertice_armors[m], vertice_armors[(m + 1) % 4], CV_RGB(0, 255, 0),2,cv::LINE_8);
        } 
        cv::imshow("DRAW_LOCATE_MATCH_ARMOR",pre_armor_rrt);
#endif //DRAW_LOCATE_MATCH_ARMOR

        Armor matched_armor;
        bool matched = false;

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
                    if (armor.id == tracking_id && dis_tmp < same_armor_distance)
                    {
                        matched = true;
                        KF.initial_KF();
                        Eigen::VectorXd position_speed(6);
                        position_speed << armor.world_position, predicted_enemy.tail(3);
                        KF.setPosAndSpeed(armor.world_position, predicted_enemy.tail(3));

                        predicted_enemy = position_speed;
                        matched_armor = armor;

                        same_armor_distance = dis_tmp;
                        // std::cout<<"track_sameID_initial!!"<<std::endl;
                    }
                }
            }

        }

        if (matched)
        {

#ifdef DRAW_MATCH_ARMOR
            // matched armor center  青色
            cv::circle(m_a,matched_armor.center,matched_armor.size.width/15,cv::Scalar(255,255,0),-1);
            // after update  黄色
            cv::Point2f p = AS.imu2pixel(predicted_enemy.head(3));
            cv::circle(m_a,p,matched_armor.size.width/20,cv::Scalar(0,255,255),-1);
            cv::Point2f vertice_lights[4];
            matched_armor.points(vertice_lights);
            for (int i = 0; i < 4; i++) {
                line(m_a, vertice_lights[i], vertice_lights[(i + 1) % 4], CV_RGB(0, 255, 255),2,cv::LINE_8);
            }
            cv::putText(m_a,std::to_string(matched_armor.camera_position.norm())+"m",matched_armor.armor_pt4[3],cv::FONT_HERSHEY_COMPLEX,2.0,cv::Scalar(0,255,255),2,8);

            cv::imshow("DRAW_MATCH_ARMOR",m_a);
#endif
            // std::cout<<"enemy_armor_cam: "<<matched_armor.camera_position.transpose()<<std::endl;
            // std::cout<<"enemy_armor_imu: "<<matched_armor.world_position.transpose()<<std::endl;
            // std::cout<<"predicted_enemy: "<<predicted_enemy.transpose()<<std::endl;
            // std::cout<<"P: \n"<<KF.P<<std::endl;
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
        // 对跟踪状态和正在丢失状态时做预测
        if(tracker_state == TRACKING || tracker_state == LOSING)
        {
            double fly_time = AS.getFlyTime(enemy_armor.world_position);
            ////////////////Singer predictor//////////////////////////////
            if(!Singer.SingerPrediction(dt,fly_time,
                                        enemy_armor.world_position,
                                        predicted_position))
            {
                reset();
                std::cerr<<"[predict value illegal!!! Fix in origin value]"<<std::endl;
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
                reset();
                std::cerr<<"[predict value illegal!!! Fix in origin value]"<<std::endl;
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
    bool ArmorTracker::locateEnemy(cv::Mat src, std::vector<Armor> armors, const chrono_time &time)
    {
        _src=src;
        AS.RotationMatrix_imu = AS.quaternionToRotationMatrix();

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
            t = time;

            if(!selectEnemy(armors,dt))
            {
                return false;
            }

            if(!estimateEnemy(dt))
            {
                return false;
            }

            Eigen::Vector3d rpy = AS.getAngle(predicted_position);
            pitch = rpy[1];
            yaw   = rpy[2];
//            std::cout<<pitch<<"---------"<<yaw<<std::endl;
            return true;
        }

    }


    void ArmorTracker::show()
    {
        cv::putText(_src,"PITCH    : "+std::to_string(pitch),cv::Point2f(0,60),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(_src,"YAW      : "+std::to_string(yaw),cv::Point2f(0,90),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        cv::putText(_src,"DISTANCE : "+std::to_string(enemy_armor.camera_position.norm())+"m",cv::Point2f(0,30),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(_src,"PREDICT_X:"+std::to_string(predicted_position(0,0)),cv::Point2f(0,_src.rows-90),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(_src,"PREDICT_Y:"+std::to_string(predicted_position(1,0)),cv::Point2f(0,_src.rows-60),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::putText(_src,"PREDICT_Z:"+std::to_string(predicted_position(2,0)),cv::Point2f(0,_src.rows-30),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,0),1,3);
        cv::Point2f vertice_lights[4];
        enemy_armor.points(vertice_lights);
        for (int i = 0; i < 4; i++) {
            line(_src, vertice_lights[i], vertice_lights[(i + 1) % 4], CV_RGB(0, 255, 255),2,cv::LINE_8);
        }
        std::string information = std::to_string(enemy_armor.id) + ":" + std::to_string(enemy_armor.confidence*100) + "%";
        //        putText(final_armors_src,ff,finalArmors[i].center,FONT_HERSHEY_COMPLEX, 1.0, Scalar(12, 23, 200), 1, 8);
        putText(_src, information,enemy_armor.armor_pt4[3],cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(255,0,255),1,3);

        // 用预测位置为中心点，选择的装甲板画框
        cv::Point2f armor_singer_center = AS.imu2pixel(predicted_position);
        cv::RotatedRect armor_singer = cv::RotatedRect(armor_singer_center,
                                                       cv::Size2f(enemy_armor.size.width,enemy_armor.size.height),
                                                       enemy_armor.angle);

        cv::Point2f vertice_armor_singer[4];
        armor_singer.points(vertice_armor_singer);
        for (int m = 0; m < 4; ++m)
        {
            line(_src, vertice_armor_singer[m], vertice_armor_singer[(m + 1) % 4], CV_RGB(255, 255, 0),2,cv::LINE_8);
        }

        imshow("final_result",_src);

    }

//}
