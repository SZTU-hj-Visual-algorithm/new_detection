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

    isChangeSameID = false;
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

// 初始化，选择最优装甲板，设置卡尔曼的F和x_1
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

// dt是两帧之间时间间隔, 跟得住目标
bool ArmorTracker::selectEnemy2(std::vector<Armor> &find_armors, double dt)
{

    KF.setF(dt);
    predicted_enemy = KF.predict();
#ifdef SHOW_TRACK_PREDICT
//    cv::Mat PointTrack = AS._src.clone();
    Eigen::Vector3d predictedE =  Eigen::Vector3d(predicted_enemy.head(3));
    cv::Point2f preTrack = AS.imu2pixel(predictedE);
    cv::circle(_src,preTrack,5,cv::Scalar(0,0,255),-1);
//    cv::imshow("_src",AS._src);
#endif

    Armor matched_armor;
    bool matched = false;

    if(!find_armors.empty())
    {
        double min_position_diff = DBL_MAX;
        for(auto & armor : find_armors)
        {
            armor.world_position= AS.pixel2imu(armor,1);
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
        }
        else
        {
            // 本帧内是否有相同ID
            for (auto & armor : find_armors) {
                if (armor.id == tracking_id) {
                    matched = true;
                    KF.initial_KF();
                    Eigen::Matrix<double,6,1> pos_speed;
                    pos_speed << armor.world_position, predicted_enemy.tail(3);
                    KF.setPosAndSpeed(armor.world_position, predicted_enemy.tail(3));
                    predicted_enemy = pos_speed;
                    matched_armor = armor;
                    break;
                }
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
    KF.setPosAndSpeed(matched_armor.world_position, predicted_enemy.tail(3));

    if(tracker_state == LOSING)
    {
        enemy_armor.world_position = predicted_enemy.head(3);
        KF.setPosAndSpeed(enemy_armor.world_position, predicted_enemy.tail(3));
    }

    return true;
}

// 对处于跟踪和正在丢失状态时做 预测，引入各种时间
bool ArmorTracker::estimateEnemy(double dt)
{
    cv::putText(_src, std::to_string(tracker_state),cv::Point(50,100),1,5,cv::Scalar(0,0,255),3);
    if(tracker_state == TRACKING || tracker_state == LOSING)
    {
        // enemy_armor get real information to predicted by singer
        enemy_armor.world_position = AS.pixel2imu(enemy_armor,1);
        std::cout << "[target_distance]:  " << enemy_armor.world_position(0, 0) << std::endl;

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
//        cv::imshow("_src",AS._src);
#endif
		return true;
    }
    else if (tracker_state == DETECTING)
    {
        enemy_armor.world_position = AS.pixel2imu(enemy_armor,1);
        std::cout << "[target_distance]:  " << enemy_armor.world_position(0, 0) << std::endl;
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
//        cv::imshow("_src",AS._src);
#endif
        locate_target = false;
        return false;
    }
    else
    {
        locate_target = false;
        return false;
    }
}

bool ArmorTracker::locateEnemy(const cv::Mat& src, std::vector<Armor> &armors, const chrono_time &time)
{
	src.copyTo(_src);
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
	
//		bullet_point = AS.airResistanceSolve(enemy_armor.world_position);

//        pitch = -atan2(bullet_point[1],bullet_point[2])/CV_PI*180 + AS.ab_pitch;
//        yaw   = -atan2(bullet_point[0],bullet_point[2])/CV_PI*180 + AS.ab_yaw;
		pitch = AS.ab_pitch;
        yaw = AS.ab_yaw;
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
//        std::cout<<"[dt:]"<<dt<<std::endl;
        t = time;
        if(!selectEnemy2(armors,dt))
        {
            return false;
        }

        if(!estimateEnemy(dt))
        {
            return false;
        }

//        Eigen::Vector3d gun_offset = {0,0,0};   // m
//        gun_offset +=enemy_armor.camera_position;
//        bullet_point = AS.airResistanceSolve(gun_offset);
	
//		bullet_point = AS.airResistanceSolve(enemy_armor.world_position);
		
		bullet_point = AS.imu2cam(bullet_point);
		pitch = -atan2(bullet_point[1],bullet_point[2])/CV_PI*180 + AS.ab_pitch;
		yaw   = -atan2(bullet_point[0],bullet_point[2])/CV_PI*180 + AS.ab_yaw;

        return true;
    }
}

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
    putText(_src,info_grade,enemy_armor.armor_pt4[0],cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0,255,255),1,8);

    // 用预测位置为中心点，选择的装甲板画框
    cv::Point2f pre_armor_center = AS.imu2pixel(predicted_position);
    cv::RotatedRect armor_rrect = cv::RotatedRect(pre_armor_center,
                                                  cv::Size2f(enemy_armor.size.width,enemy_armor.size.height),
                                                  enemy_armor.angle);
    cv::Point2f vertice_armors[4]; // 预测装甲板绿色
    armor_rrect.points(vertice_armors);
    cv::Point2f vertice_lights[4]; // 目标装甲板紫色
    enemy_armor.points(vertice_lights);
    for (int m = 0; m < 4; ++m)
    {
        line(_src, vertice_lights[m], vertice_lights[(m + 1) % 4], CV_RGB(0, 255, 255),2,cv::LINE_8);
        line(_src, vertice_armors[m], vertice_armors[(m + 1) % 4], CV_RGB(0, 255, 0),2,cv::LINE_8);
    }

    std::string information = std::to_string(enemy_armor.id) + ":" + std::to_string(enemy_armor.confidence*100) + "%";
//            putText(final_armors_src,ff,finalArmors[i].center,FONT_HERSHEY_SIMPLEX, 1.0, Scalar(12, 23, 200), 1, 8);
    putText(_src, information,enemy_armor.armor_pt4[3],cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(255,0,255),1,3);

    imshow("final_result",_src);

}

//}
