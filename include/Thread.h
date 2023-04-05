#include "camera.h"
#include "armor_track.h"
#include <opencv2/core/cvstd.hpp>
#include "serialport.h"
// #include "energy_predict.h"
#include "serial_main.h"
#include <thread>
#include <mutex>
#include <string>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <chrono>

extern pthread_mutex_t mutex_new; 
extern pthread_cond_t cond_new; 
extern pthread_mutex_t mutex_ka; 
extern pthread_cond_t cond_ka;

extern bool is_ka;
extern bool is_start;
extern bool is_continue;

void* Build_Src(void* PARAM);
void* Armor_Kal(void* PARAM);
void* Kal_predict(void* PARAM);

typedef struct form
{
	int mode;
	int dat_is_get;
	float data[3];
	float quat[4];
	std::vector<Armor> armors;
    chrono_time tim;
}form;//线程之间的数据交换结构体
