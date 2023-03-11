#include "energy_predict.h"

void energy_pre::reset()//没打中大符，大符时间重置故所有参数都要重置
{
	H = 1;
	Q = 10;
	R = 0.6;
	sigma = Q;
	start_p = { -1,-1 };
	start_c = {-1,-1};
	measure_angle = 0.10;
	angle_k_1 = 0.10;
	angle_k = 0.10;
	//start_time = -1;
	last_time = 0;
	depth = 0;
	t=0;
	dt=0;
	flip_angle = false;
//	count = 0;
}

void energy_pre::hit_reset()//打中大符后所需的重置函数
{
	H = 1;
	Q = 10;
	R = 0.6;
	sigma = Q;
	start_c = {-1,-1};
	start_p = {-1,-1};
	measure_angle = 0.10;
	angle_k_1 = 0.10;
	angle_k = 0.10;
	last_time = 0;
	flip_angle = false;
}



cv::Point2f energy_pre::angle2_xy(cv::Point2f &now_xy, double pred_angle)
{
    cv::Point2f R_center = center_R.center;
    double radius = sqrt((R_center.x-Aim_armor.x)*(R_center.x-Aim_armor.x)+(R_center.y-Aim_armor.y)*(R_center.y-Aim_armor.y));
	double angle = atan2(now_xy.y - R_center.y, now_xy.x - R_center.x);
	double pre_x, pre_angle, pre_y;
	if (direct == 1)
	{
		//1为顺时针
		pre_angle = keep_pi(angle + pred_angle);
		
	}
	else
	{
		//0为逆时针
		pre_angle = keep_pi(angle - pred_angle);
	}
	pre_x = radius*cos(pre_angle) + R_center.x;
	pre_y = radius*sin(pre_angle) + R_center.y;
	
	
	
	return cv::Point2f(pre_x, pre_y);
}


energy_pre::energy_pre()
{
	H = 1;
	Q = 10;
	R = 0.6;
	sigma = Q;
	F_MAT=(cv::Mat_<double>(3, 3) << 1572.95566, 0.000000000000, 631.34618, 0.000000000000, 1572.71538, 523.05524, 0.000000000000, 0.000000000000, 1.000000000000);
	C_MAT=(cv::Mat_<double>(1, 5) << -0.08780, 0.21354, -0.00000, 0.00006, 0.00000);
	
	cv::cv2eigen(F_MAT,F_EGN);
	cv::cv2eigen(C_MAT,C_EGN);
	//this->enermy_color = RED;
}




double energy_pre::predict(double t, double dt, bool pre_not, bool samll_energy)
{
	if (samll_energy)
	{
//		printf("small energy!!!\n");
		if (pre_not)
		{
			F = 1 + dt/t;
			angle_k = F*angle_k;
			angle_k_1 = angle_k-start_angle;
			return angle_k_1;
		}
		else
		{
			t = (t+t+dt)/2;
			F = 1 + dt/t;
			return F*angle_k - start_angle;
		}
		
	}
	else
	{
//		printf("big energy!!!\n");
		if (pre_not)
		{
			double F1 = 1.977 * sin(2.000 * t) + 1.345;
			double F2 = 1.177 * t - 913.0000 * cos(0.971 * t) * cos(0.971 * t) / 971.000 + 913.000 / 971.000;
			//if (sin(1.942*t))
			F = 1 + F1 * dt / F2;
			/*if (sin(1.942*t) > 0.5)
			{
				shoot_delay = 0.69511;
			}
			else
			{
				shoot_delay = 0.35911;
			}*/
			angle_k = F*angle_k;
			angle_k_1 = angle_k - start_angle;
//		angle_k_1 = F*angle_k_1;
			return angle_k_1;
		}
		else
		{
			t = t + dt;
			double F1 = 1.977 * sin(2.000 * t) + 1.345;
			double F2 = 1.177 * t - ((913.00000 * cos(0.971 * t) * cos(0.971 * t)) / 971.0000)+ (913.00000 / 971.00000);
			//std::cout << "时间" << t << std::endl;
			//std::cout << "F2" << F2 << std::endl;
			F = 1 + F1 * dt / F2;
			//std::cout << "F" << F << std::endl;
			//让预测不影响卡尔曼的迭代
			return F * angle_k - start_angle;
		}
	}
	
}

double energy_pre::correct(double measure)//这里先不管了，以后再想，摆了
{
	sigma = F * sigma * F + Q;
	
	K = sigma * H / (H * sigma * H + R);
	
	angle_k_1 = angle_k_1 + K * (measure - H * angle_k_1);
	
	sigma = (1 - K * H) * sigma;
	
	//得出绝对角度的后验值
	angle_k = angle_k_1 + start_angle;
	
	return angle_k_1;
}



//double energy_pre::keep_pi_2(double angle)
//{
//	if (angle > 90)
//	{
//		angle = 90.0 - (angle - 90.0);
//		return angle;
//	}
//	else if (angle < 0)
//	{
//		angle = -angle;
//		return angle;
//	}
//	return angle;
//}

double energy_pre::keep_pi(double angle)
{
	if (angle > CV_PI)
	{
		return -(CV_PI + CV_PI - angle);
	}
	else if (angle < -CV_PI)
	{
		return CV_PI + CV_PI + angle;
	}
	else
	{
		return angle;
	}
}



cv::Point2f energy_pre::gravity_finish(cv::Point2f& pps)
{
	double height;
	
	//----------用到目标点而不是预测点是因为要获取目标的距离------------
	//double depth = ap(2,0);
	printf("depth:!!!%lf\n",depth);
//	std::cout<<depth<<std::endl;
	//------------------------------------------------------------------
	
	cv::Point2f r_pps(pps.x/*+210*/,pps.y/*+200*/);
	Eigen::Vector3d p_pre = {(double)r_pps.x,(double)r_pps.y,1.0};
	Eigen::Vector3d ap_pre = pu_to_pc(p_pre,depth);
	
	double del_ta = pow(28, 4) + 2 * 9.8 * ap_pre(1, 0) * 28 * 28 - 9.8 * 9.8 * depth*depth;
	double t_2 = (9.8 * ap_pre(1, 0) + 28 * 28 - sqrt(del_ta)) / (0.5 * 9.8 * 9.8);
	height = 0.5 * 9.8 * t_2;
	//std::cout<<"抬枪补偿:"<<height<<std::endl;
	Eigen::Vector3d ap_g = {ap_pre(0,0),ap_pre(1,0) - height,depth};
	E_pitch = atan2(ap_pre(1,0) + 0.05 - height*1., depth)/CV_PI*180.0;
	E_yaw = atan2(ap_pre(0,0) - 0.04, depth)/CV_PI*180.0;
	
	Eigen::Vector3d ap_pu = pc_to_pu(ap_g,depth);//ap_g(2,0)是距离
	
	return cv::Point2f((int)ap_pu(0,0)/*-210*/,(int)ap_pu(1,0)/*-200*/);
}

double energy_pre::measured(cv::Point2f& xy) //重写
{
    cv::Point2f R_center = center_R.center;
    double radius = sqrt((R_center.x-Aim_armor.x)*(R_center.x-Aim_armor.x)+(R_center.y-Aim_armor.y)*(R_center.y-Aim_armor.y));
	int offset_x = R_center.x - start_c.x;
	int offset_y = R_center.y - start_c.y;
	cv::Point2f _start_p = {start_p.x+offset_x,start_p.y+offset_y};
	double dis_st = sqrt((xy.x - _start_p.x) * (xy.x - _start_p.x) + (xy.y - _start_p.y) * (xy.y - _start_p.y)) / 2;
	//防止出现无穷大数据，因为asin函数如果入参大于1就会出现无穷大数据
	if (dis_st > radius)
	{
		dis_st = radius;
	}
	
	double angle = 2 * asin(dis_st/radius);
	
	if (angle > 2.98)
	{
		flip_angle = true;
	}
	if (flip_angle)
	{
		angle = 2 * CV_PI - angle;
	}
	measure_angle = angle;
	//std::cout <<"观测值"<< measure_angle << std::endl;
	if (measure_angle < 0)
	{
		measure_angle = 0.10;
	}
	
	return measure_angle;
}

//bool energy_pre::cal_dela_angle()
//{
//    double dela_dis = sqrt((Aim_armor.x-last_dt_p.x)*(Aim_armor.x-last_dt_p.x)+(Aim_armor.y-last_dt_p.y)*(Aim_armor.y-last_dt_p.y));
//    double angle = (2*asin((dela_dis/2.0)/radius))/CV_PI*180.0;
//    if (angle >= 69)
//    {
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}

void energy_pre::get_direct(cv::Point2f &now)
{
    cv::Point2f R_center = center_R.center;
    double symbol = (now.y - R_center.y) * (last_p.y - R_center.y);
    double now_ang = atan2(now.y - R_center.y, now.x - R_center.x);
    double five_ang = atan2(last_p.y - R_center.y, last_p.x - R_center.x);
    if (symbol < 0)//是否在x轴的交界处
        {
        if ((now_ang - five_ang) < 0)
        {
            direct = abs(now_ang) > CV_PI / 2 && abs(five_ang) > CV_PI / 2 ? 1 : 0;
        }
        else
        {
            direct = abs(now_ang) > CV_PI / 2 && abs(five_ang) > CV_PI / 2 ? 0 : 1;
        }
        }
    else
    {
        if ((now_ang - five_ang) < 0)
        {
            direct = 0;
        }
        else
        {
            direct = 1;
        }
    }


}
