#include "camera.h"
#include "armor_track.h"
#include <opencv2/core/cvstd.hpp>
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
	vector<Armor> armors;
    chrono_time tim;
}form;
