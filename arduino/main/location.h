#ifndef LOCATION_H
#define LOCATION_H

#include "MPU9250.h"

#define Sample_Rate 5

#define origin_latitude = -91000000
#define orgigin_longitude = 41000000
#define RADIUS_OF_EARTH 6378137

class Location {
    long last_gps_measurement;
    int32_t gps_latitude;
    int32_t gps_longitude;
    int32_t gps_altitude;
    int32_t gps_heading;
    int32_t gps_speed;
    uint32_t gps_haccuracy;
    uint32_t gps_accuracy;

    long last_imu_measurement;

    int32_t computed_latitude;
    int32_t computed_longitude;
    int32_t computed_altitude;
    uint32_t computed_haccuracy;
    uint32_t computed_accuracy;

    int32_t destination_latitude;
    int32_t destination_longitude;
    int32_t count = 0 ;
    public:
        float Get_Distance();
        float Get_Bearing();
        void Update_Position();
        void Set_Destination(int32_t latitude, int32_t longitude);
        int32_t Get_Latitude();
        int32_t Get_Longitude();
        int32_t Get_Altitude();
        uint32_t Get_Horizontal_Accuracy();
        uint32_t Get_3D_Accuracy();
};
#endif