#pragma once
#include "camera.h"
#include "ArmorDetector.hpp"
#include "ArmorTracker.h"
#include <opencv2/core/cvstd.hpp>
#include "CRC_Check.h"
#include "serialport.h"
#include <thread>
#include <mutex>
#include <string>

extern pthread_mutex_t mutex_new;
extern pthread_cond_t cond_new;
extern pthread_mutex_t mutex_ka;
extern pthread_cond_t cond_ka;

extern bool is_ka;
extern bool is_start;
extern cv::Mat src;

extern cv::Mat  quan_src;
extern float quan_ab_pitch;
extern float quan_ab_yaw;
extern float quan_ab_roll;
extern float quan_speed;

void* Build_Src(void* PARAM);
void* Armor_Kal(void* PARAM);
void* Kal_predict(void* PARAM);
