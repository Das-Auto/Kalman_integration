#include "Car.h"

Car::Car(carobjlidar lmarker, Vec_Radar rmarker, UKF ukf)
{
    this->ukf = ukf;
    this->lmarker = lmarker;
    this->rmarker = rmarker;
}
Car::Car(carobjlidar lmarker)
{
    this->lmarker = lmarker;
}


Car::Car(Vec_Radar rmarker)
{
    this->rmarker = rmarker;
}
    