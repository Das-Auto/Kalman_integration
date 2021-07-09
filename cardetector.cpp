#include "cardetector.h"

CarDetector::CarDetector(){
  
}
///1.for loop through car array
    ///2.check if the covariance of the car matches the new lidar readings
    ///3.if a match is found update and set the flag to 0
    ///4.if not remove car object from array
    ///5.for loop again through car array
    ///6.if a one is found create a new object and append it to traffic
    ///

CarDetector::~CarDetector(){}

void CarDetector::readLidarData()
{
    std::vector<struct carobjlidar> data = std::vector<struct carobjlidar>();
    
}

void CarDetector::recieveLIDARData(carobjlidar data)
{
    ldata.push_back(data);
}

void CarDetector::recieveRADARData(Vec_Radar data)
{
    rdata.push_back(data);
}


void CarDetector::detectlidar()
{
    ///1.for loop through car array
    ///2.check if the covariance of the car matches the new lidar readings
    ///3.if a match is found update and set the flag to 0
    ///4.if not remove car object from array
    ///5.for loop again through car array
    ///6.if a one is found create a new object and append it to traffic
    ///
    float sigma1, sigma2, x, y;

    for(int i = 0; i < traffic.size(); i++)
    {
        /// loop through lidar data
        /// if the x and y of lidar data are within the cov range
        /// update position and flip flag to zero
        sigma1 = std::sqrt(traffic[i].ukf.P_(0, 0));
        sigma2 = std::sqrt(traffic[i].ukf.P_(1, 1));
        x = traffic[i].ukf.x_(0);
        y = traffic[i].ukf.x_(1);
        int detected = 0;
        for(int j = 0; j < ldata.size();  j++)
        {
            //check if x and y is within cov range
            if(ldata[j].x < x + sigma1 && ldata[j].x > x - sigma1 && ldata[j].isNewCar == 1)
            {
                if(ldata[j].y < y + sigma2 && ldata[j].y > y - sigma2 && ldata[j].isNewCar == 1)
                {
                    ///when x and y are in cov range, change the x position and y position of the car object
                    traffic[i].lmarker.x = ldata[j].x;
                    traffic[i].lmarker.y = ldata[j].y;
                    ldata[j].isNewCar = 0;
                    detected = 1;
                    break;
                }
                
            }

        }
        
        if(detected == 0)
        {
            traffic.erase(traffic.begin() + i);
        }
        
    }

    for(int i = 0; i < ldata.size(); i++)
    {
        if(ldata[i].isNewCar == 1)
        {
            Car car =  Car(ldata[i]);
            traffic.push_back(car);
        }
    }
}

void CarDetector::detectradar()
{
    ///1.for loop through car array
    ///2.check if the covariance of the car matches the new lidar readings
    ///3.if a match is found update and set the flag to 0
    ///4.if not remove car object from array
    ///5.for loop again through car array
    ///6.if a one is found create a new object and append it to traffic
    ///
    float sigma1, sigma2, sigma3, rho, psi, rho_dot;
    for(int i = 0; i < traffic.size(); i++)
    {
        /// loop through lidar data
        /// if the x and y of lidar data are within the cov range
        /// update position and flip flag to zero
        sigma1 = std::sqrt(traffic[i].ukf.P_(0, 0));
        sigma2 = std::sqrt(traffic[i].ukf.P_(1, 1));
        sigma3 = std::sqrt(traffic[i].ukf.P_(2, 2));

        rho = traffic[i].ukf.x_(0);
        psi = traffic[i].ukf.x_(1);
        rho_dot = traffic[i].ukf.x_(2);
        for(int j = 0; j < rdata.size();  j++)
        {
         
            //check if x and y is within cov range
            if(rdata[j].rho < rho + sigma1 && rdata[j].rho > rho - sigma1 && rdata[j].flag == 1)
            {
                if(rdata[j].psi < psi + sigma2 && rdata[j].psi > psi - sigma2 && rdata[j].flag == 1)
                {
                    if(rdata[j].rho_dot < rho_dot + sigma3 && rdata[j].rho_dot > rho_dot - sigma3 && rdata[j].flag == 1){
                        traffic[i].rmarker.rho = rdata[j].rho;
                        traffic[i].rmarker.psi = rdata[j].psi;
                        traffic[i].rmarker.rho_dot = rdata[j].rho_dot;
                        rdata[j].flag = 0;
                        break;
                    }
                }
                
            }else
            {
                traffic.erase(traffic.begin() + i);
            }

        }
    }

    for(int i = 0; i < rdata.size(); i++)
    {
        if(rdata[i].flag == 1)
        {
            Car car = Car(rdata[i]);
            traffic.push_back(car);
        }
    }
}