#include "FrameRead.h"
// #include <fstream>

std::vector<std::vector<float>> getNextFrame(FILE * pf)
{
    float xmin, ymin, zmin;
    float xmax, ymax, zmax;
    float xavg, yavg;
    char end_of_frame_char = '$';
    std::vector<std::vector<float>> coordinates_lidar;

    while(true)
    {
        int frame_length = 0;
        fscanf(pf, "%f %f %f %f %f %f", &xmin, &ymin, &zmin, &xmax, &ymax, &zmax);
        xavg = (xmax + xmin)/2;
        yavg = (ymax + ymin)/2;


        std::vector<float> vec;
        vec.push_back(xavg);
        vec.push_back(yavg);
        coordinates_lidar.push_back(vec);

 
        if (fgetc(pf) == end_of_frame_char)
        {
            fgetc(pf);
        
            return coordinates_lidar;
        
        }
    }
    
    
}


