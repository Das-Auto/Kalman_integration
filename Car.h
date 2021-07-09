#include "ukf.h"
#include "FrameRead.h"
#include "Sensor_data.h"

typedef struct Vec_Radar
{
    float rho;
    float psi;
    float rho_dot;
    int flag;
} Vec_Radar;

class Car
{
    public:
    UKF ukf;
    
	///float acceleration;
    struct carobjlidar lmarker;
    Vec_Radar rmarker;

    Car(struct carobjlidar lmarker, Vec_Radar rmarker, UKF ukf);
    Car(struct carobjlidar lmarker);
    Car(Vec_Radar rmarker);
 
    
};