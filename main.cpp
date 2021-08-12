#include <iostream>
#include "FrameRead.h"
#include "cardetector.h"
#include "Eigen/Dense"
using namespace std;

CarDetector * cd = nullptr;

void initilize_car_detector()
{
    if(cd == nullptr)
    {
        cd = new CarDetector();
    }
}

vector<UKF_results> spin_once(vector<vector<float>> coordinates_lidar)
{
    initilize_car_detector();
    vector<UKF_results> results;
    
    ///Takes frame data and transforms it to carobjlidar 
    ///data type in order to use it in caretector object

    for(int i=0; i<coordinates_lidar.size(); i++)
    {
        carobjlidar temp;
        temp.x = coordinates_lidar[i][0];
        temp.y = coordinates_lidar[i][1];
        temp.isNewCar = 1;
        cd->ldata.push_back(temp);
    }

    cd->detectlidar();

    for(int i = 0; i < cd->traffic.size(); i++)
    {
        float x = cd->traffic[i].lmarker.x;
        float y = cd->traffic[i].lmarker.y;
        Eigen::VectorXd v = Eigen::VectorXd(2);
        v << x,
             y;
        MeasurementPackage meas_package;
        meas_package.sensor_type_ = meas_package.LASER;
        meas_package.raw_measurements_ = v;
        meas_package.timestamp_ = 1;
        cd->traffic[i].ukf.ProcessMeasurement(meas_package);
    }
    
    Eigen::VectorXd x_; 
    Eigen::MatrixXd P_;
    for(int i = 0; i < cd->traffic.size(); i++)
    {

        x_ = cd->traffic[i].ukf.x_;
        P_ = cd->traffic[i].ukf.P_;

        UKF_results res; 

        res.x_ = x_;
        res.P_ = P_;

        cout << x_ << endl;
        cout << P_ << endl;

        results.push_back(res);
    }
    return results;
}

int main (int argc, char *argv[])
{
    FILE * pf = fopen("l.txt", "r");

    std::vector<std::vector<float>> coordinates_lidar;
    std::vector<UKF_results> results;

    while (!feof(pf))
    {
        std::cout << "hello2"<< std::endl;

        coordinates_lidar = getNextFrame(pf);
        std::cout << "hello3"<< std::endl;

        results = spin_once(coordinates_lidar);
        std::cout << "hello4"<< std::endl;

    }
    return 0;
}

