#include "ArmorTracker.h"
#include "AngleSolve.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>


#define DRAW_CENTER_CIRCLE

ArmorTracker::ArmorTracker() {
    tracking_id = 0;

    lost_aim_cnt = 0;
    lost_threshold = 5;

    new_old_threshold = 10; //cm

    change_aim_cnt = 0;
    change_aim_threshold = 20;
    isChangeSameID = false;

    tracker_state = DETECTING;
}

void ArmorTracker::setMyAngle(cv::Mat src, float pitch, float yaw, float roll, float SPEED)
{
    _src = src.clone();
    AS.ab_pitch = pitch;
    AS.ab_yaw = yaw;
    AS.ab_roll = roll;
    AS.SPEED = SPEED;

}

// 选择enemy_armors，限制为同ID
void ArmorTracker::selectEnemy(vector<Armor> find_armors)
{

    if(find_armors.empty())
    {

        if(tracker_state == DETECTING)      // 空的&&检测状态，不参与函数
        {
            enemy_armors = Armor();
            tracking_id = 0;
            lost_aim_cnt = 0;
            change_aim_cnt = 0;
            isChangeSameID = false;
        }
        else if(find_armors.empty() && tracker_state == TRACKING)       // 空的&&跟踪状态，丢失++
        {
            lost_aim_cnt++;
            tracker_state = LOSING;
            if(lost_aim_cnt > lost_threshold)
            {
                tracker_state = DETECTING;
                tracking_id = 0;
                lost_aim_cnt = 0;
                change_aim_cnt = 0;
                isChangeSameID = false;
                enemy_armors = Armor();
            }
        }
        else if(tracker_state == LOSING)        // 空的&&丢失状态，丢失++
        {
            lost_aim_cnt++;
            if(lost_aim_cnt > lost_threshold)
            {
                tracker_state = DETECTING;
                tracking_id = 0;
                lost_aim_cnt = 0;
                change_aim_cnt = 0;
                isChangeSameID = false;
                enemy_armors = Armor();
            }
        }

        cout<<"Tracker State in no enemy:   "<< tracker_state<<endl;
        return ;
    }





    // -----------------------单目标
    if(find_armors.size() == 1)
    {

        if(tracker_state == DETECTING)      // 一个目标&&检测状态&&上一帧没有检测到ID，第一跟踪目标放入 enemy 并记录ID，计算真实坐标
        {
            // 计算真实的坐标
            find_armors[0].current_position = getRealPosition(find_armors[0]);

            // 初始化x_k1
////            KF.initial(find_armors[0].current_position);

            enemy_armors = find_armors[0];
            tracking_id = find_armors[0].id;
            tracker_state = TRACKING;
        }
        else if(tracker_state == TRACKING || tracker_state == LOSING)      // 一个目标&&跟踪状态&&与上一帧ID相同&&距离符合阈值（预测阈值和识别阈值），退出函数
        {
            // 计算真实的坐标
            find_armors[0].current_position = getRealPosition(find_armors[0]);

            // 本帧的真实坐标和上一帧的真实坐标，如果上一帧处在丢失状态呢？新旧坐标的保存以及时间的准确；记得清空上一帧的enemy再emplace_back-----------------------------------------
            double new_old_distance = (find_armors[0].current_position - enemy_armors.current_position).norm();

            // 不是检测状态，代表有enemy历史记录，相同ID且符合阈值，即可继续跟踪，否则为丢失
            if(find_armors[0].id == tracking_id || new_old_distance > new_old_threshold)
            {
                // 这一帧的放进去，转到下一帧这个就是上一帧的旧位置
                enemy_armors = find_armors[0];

                tracker_state = TRACKING;
            }
            else
            {
                lost_aim_cnt++;
                if(lost_aim_cnt > lost_threshold)
                {
                    tracker_state = DETECTING;
                    tracking_id = 0;
                    lost_aim_cnt = 0;
                    enemy_armors = Armor();
                }
                else
                {
                    tracker_state = LOSING;
                }
            }
        }

        cout<<"Tracker State in 1 enemy:   "<< tracker_state<<endl;
        return ;
    }

/*
    // 多目标

    if(find_armors.size() > 1)
    {
        if(tracker_state == DETECTING)      // 多个目标&&检测状态&&跟踪ID为0，寻找相同ID装甲板（分数高为跟踪ID），最多只有两个
        {
            // ******
            int maxGrade = find_armors[0].grade;
            tracking_id = find_armors[0].id;
            size_t index;

            for(size_t i = 1; i < find_armors.size(); ++i)
            {
                if(maxGrade < find_armors[i].grade)
                {
                    maxGrade = find_armors[i].grade;
                    tracking_id = find_armors[i].id;
                    index = i;
                }
            }

            find_armors[index].current_position = getRealPosition(find_armors[index]);
            enemy_armors = find_armors[index];

            // 初始化x_k1
////            KF.initial(find_armors[0].current_position);

            tracking_id = find_armors[0].id;
            tracker_state = TRACKING;
            return ;
        }
        else if(tracker_state == TRACKING || tracker_state == LOSING)      // 多目标&&跟踪状态，先筛选同ID的装甲板，再寻找与上一帧阈值范围内的装甲板作为这一帧需要跟踪的装甲板
        {
            bool isFind = false;
            for(size_t i = 0; i < find_armors.size(); ++i)
            {
                //寻找相同ID的装甲板，检验他是否符合阈值
                if(find_armors[i].id == tracking_id)
                {
                    //把装甲板的中心点转换到陀螺仪坐标系下
                    find_armors[i].current_position = getRealPosition(find_armors[i]);

                    double new_old_distance = (find_armors[i].current_position - enemy_armors.current_position).norm();
                    if(new_old_distance < new_old_threshold)
                    {
                        enemy_armors = find_armors[i];
                        isFind = true;
                        break;
                    }
                    else
                    {
                        change_aim_cnt++;
                        if(change_aim_cnt == change_aim_threshold)
                        {
                            // 初始化x_k1
////                            KF.initial(find_armors[0].current_position, predicted_speed);
                        }
                        if(change_aim_cnt > change_aim_threshold)
                        {
                            enemy_armors = find_armors[i];
                            isFind = true;
                            isChangeSameID =true;
                            break;
                        }
                    }
                }
            }

            if(isFind)
            {
                tracker_state = TRACKING;
            }
            else
            {
                lost_aim_cnt++;
                if(lost_aim_cnt > lost_threshold)
                {
                    tracker_state = DETECTING;
                    tracking_id = 0;
                    lost_aim_cnt = 0;
                    enemy_armors = Armor();
                }
                else
                {
                    tracker_state = LOSING;
                }
            }

        }
    }
*/
}

void ArmorTracker::getPredictedPositionAndSpeed(clock_t start_time)
{


    if(tracker_state == TRACKING)
    {
        //时间计算（数据收发延时+弹道时间+程序运行时间）,ms
        double delay = 0.01;
        double fly = AS.getFlyTime();
        clock_t finish = clock();
        double run = (double)(finish - start_time);
        double all_time = delay + fly + run;

        //initial F
////        KF.setF(all_time);
        if(isChangeSameID)
        {
////            KF.setP(KF.P);
        }
////        Eigen::VectorXd pre_position_speed = KF.update(enemy_armors.current_position);

////        predicted_position<<pre_position_speed[0],pre_position_speed[1],pre_position_speed[2];
////        predicted_speed<<pre_position_speed[3],pre_position_speed[4],pre_position_speed[5];

        AS.getAngle(predicted_position);

    }
    else
    {

        return ;
    }
}

// 计算真实坐标
Eigen::Vector3d ArmorTracker::getRealPosition(Armor armor)
{
    Eigen::Vector3d Tvec = AS.pnpSolve(armor.armor_pt4,armor.type, 1);

#ifdef DRAW_CENTER_CIRCLE
    //Pos(1,0) = -1 * Pos(1,0);
    Eigen::Matrix3d F;
    cv::cv2eigen(AS.F_MAT,F);
    Eigen::Vector3d pc = Tvec;
    Eigen::Vector3d pu = F * pc / pc(2, 0);
    cv::circle(_src, {int(pu(0, 0)), int(pu(1, 0))}, 5, cv::Scalar(255,255,0), -1);
    cout<<"center:  ("<<pu(0, 0)<<", "<<pu(1, 0)<<", "<<pu(2,0)<<")"<<endl;

    //Pos(1,0) = -1 * Pos(1,0);
#endif

    Eigen::Vector3d aimInWorld = AS.transformPos2_World(Tvec);
    return aimInWorld;
}


headAngle ArmorTracker::finalResult(cv::Mat src, vector<Armor> find_armors,clock_t start_time)
{
    _src = src;
    selectEnemy(find_armors);
    getPredictedPositionAndSpeed(start_time);


    //printf("-----------------%d------idid------------",enemy_armors.id);

    return AS.send;
}

