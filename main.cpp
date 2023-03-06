#include "armor_detection.h"
#include "gimbal_control.h"
#include <opencv2/core/cvstd.hpp>
#include "CRC_Check.h"
#include"serialport.h"
#include<X11/Xlib.h>
#include"Thread.h"

//#define DETECT
#define PREDICT

using namespace cv;

pthread_t thread1;
pthread_t thread2;
pthread_t thread3;

pthread_mutex_t mutex_new;
pthread_cond_t cond_new;
pthread_mutex_t mutex_ka;
pthread_cond_t cond_ka;

bool is_ka = false;
bool is_start = false;
bool is_continue = true;

int main(void)
{
    XInitThreads();
    pthread_mutex_init(&mutex_new, NULL);
    pthread_cond_init(&cond_new, NULL);
    pthread_create(&thread1, NULL, Build_Src, NULL);
    pthread_create(&thread2, NULL, Armor_Kal, NULL);
    pthread_create(&thread3, NULL, Kal_predict, NULL);
    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);
    pthread_join(thread3, NULL);
    pthread_mutex_destroy(&mutex_new);
    return 0;
}
