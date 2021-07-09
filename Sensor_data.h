#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include "Eigen/Dense"

typedef struct carobjlidar
{
    int isNewCar;
    float x;
    float y;

}carobjlidar;

typedef struct UKF_results
{
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
}UKF_results;

#endif