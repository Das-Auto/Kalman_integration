#include "Car.h"
#include <vector>
#include <cmath>
#include "FrameRead.h"
#include "Sensor_data.h"
class CarDetector
{

    public:
    

    std::vector<struct carobjlidar> ldata; ///data from lidar
    std::vector<Vec_Radar> rdata; ///data from radar
    std::vector<Car> traffic;
    
    CarDetector();

    virtual ~CarDetector();

    void detectlidar();
    void detectradar();
    void recieveLIDARData(struct carobjlidar data);
    void recieveRADARData(Vec_Radar data);
    void readLidarData();
    
};