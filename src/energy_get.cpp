//这个文件是用来将所有大符所需的函数集合起来的文件
#include "energy_predict.h"
#include <opencv2/opencv.hpp>
#include <iostream>

//#define BEIZHU

using namespace cv;
using namespace std;

bool energy_pre::energy_detect(Mat &image, int color) {

//	Rect r(210,200,840,824);
	src = image/*(r)*/.clone();
	this->enemy_color = color;
	//目标大符装甲板检测
	target = detect_aim();
	Aim_armor = target.re_aim;
	center_R = target.c_rect;
	//cout<<real_xy[1]<<endl;
	//cout<<real_xy[0]<<endl;
	/*double height;
	double del_ta = pow(SPEED, 4) + 2 * 9.8 * real_xy[1] * SPEED * SPEED - 9.8 * 9.8 * depth*depth;
	double t_2 = (9.8 * real_xy[1] + SPEED * SPEED - sqrt(del_ta)) / (0.5 * 9.8 * 9.8);
	height = 0.5 * 9.8 * t_2;
	E_pitch = atan2(real_xy[1] - height, depth)/CV_PI*180.0;
	E_yaw = atan2(real_xy[0],depth)/CV_PI*180.0;*/
	if ((target.re_aim.x == 0)&&(target.re_aim.y == 0))
	{
//		lose_aim = true;
		return false;
	}
	return true;
}


bool energy_pre::energy_predict_aim(long int now_time, bool small_energy) {
	if (start_time == -1)
	{
		start_time = now_time;//大符开始计时的时刻
		last_time = now_time;//上一时刻
	}
	else
	{
		t = ((double)(now_time - start_time)) / getTickFrequency();
//		cout<<t<<endl;
		if (change_aim)
		{
		    if (change_aim == 1)
		    {
		        hit_reset();
		    }
		    else if (change_aim == 2)
		    {
		        reset();
		        return false;
		    }
		    else if (change_aim == 3)
		    {
                reset();
                direct = -1;
                dir_count = 0;
                distances.clear();
                return false;
		    }
		}
//		std::cout<<"总时间："<<t<<endl;
		dt = ((double) (now_time - last_time)) / getTickFrequency();
//		std::cout<<"时间："<<dt<<std::endl;
		last_time = now_time;
	}



	if (start_p.x == -1 && start_p.y == -1)
	{
		start_p = Aim_armor;
		start_c = center_R.center;
	}

	if (dir_count == 0) {
		last_p = Aim_armor;
		dir_count++;
	} else if (dir_count == 10) {
		get_direct(Aim_armor);
		dir_count++;
	} else if (dir_count< 10)
	{
		dir_count++;
	}
//	cout << "direct：" << direct << endl;
//	cout<<"dir_count:"<<dir_count<<endl;
	double angle;

	//上一时刻的点设置好
    if ((center_R.center.x!=0)&&(center_R.center.y!=0)&&(depth>3)&&(depth<13))
    {
        if (direct!=-1)
        {
            angle = measured(Aim_armor);
            //			cout<<"观测值："<<angle<<endl;

            double predict_angle;
            predict(t, dt, true, small_energy);//更新步前必要的更新参数用
            double cor = correct(angle);//更新步
            //std::cout<<"更新角度："<<cor<<std::endl;
            cv::Point2f pre_aim;
            double p_t =sqrt(real_xy[0]*real_xy[0]+real_xy[1]*real_xy[1]+depth*depth) / 28;
            predict_angle = predict(t , (p_t + shoot_delay)*1.3385, false,small_energy);
            //cout<<predict_angle<<endl;
            double pred_ang = predict_angle - cor;
            //std::cout<<"预测角度："<<pred_ang<<std::endl;
            pre_aim = angle2_xy(Aim_armor,pred_ang);
            //			std::cout<<"predict_aim:"<<pre_aim<<std::endl;


            cv::Point2f mubiao = gravity_finish(pre_aim);
            //cout<<mubiao.x<<"\t"<<mubiao.y<<endl;
            circle(src, mubiao, 8, Scalar(255, 255, 0), -1);
            //imshow("image", src);
            return true;
        }
        else
        {

            return false;
        }

    }
    else
    {

        return false;
    }
}


