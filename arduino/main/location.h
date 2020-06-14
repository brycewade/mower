#ifndef LOCATION_H
#define LOCATION_H

#include <EEPROM.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include "SparkFun_BNO080_Arduino_Library.h"
#include "Kalman.h"

#define GPS_I2C_Address 0x42
#define GPS_Sample_Rate 5
#define CLOSE_ENOUGH 0.1
#define ORIGIN_LONGITUDE -916135565
#define ORIGIN_LATITUDE 417712430
#define ORIGIN_ALTITUDE 232
#define INITIAL_TIME_STEP 0
#define HEADING_READINGS 20
#define MIN_TURN_POWER 40

// #define origin_latitude -91000000
// #define orgigin_longitude 41000000
#define RADIUS_OF_EARTH 6378137

class Location {
    // GPS measurements
    long last_gps_measurement;
    int32_t gps_latitude;
    int8_t gps_latitudeHP;
    int32_t gps_longitude;
    int8_t gps_longitudeHP;
    int32_t gps_altitude;
    int32_t gps_heading;
    int32_t gps_velN;
    int32_t gps_velE;
    int32_t gps_velD;
    uint32_t gps_haccuracy;
    uint32_t gps_accuracy;
    // Conversion to meters
    int32_t origin_longitude;
    int32_t origin_latitude;
    int32_t origin_altitude;
    float longitude_to_meters;
    float latitude_to_meters;
    // IMU measurements
    long last_imu_measurement;
    // Computed position
    long last_update;
    BLA::Matrix<3> Ex;
    BLA::Matrix<3> Ey;
    BLA::Matrix<3> Ez;
    bool self_drive=false;
    float heading;
    float yaw;
    float pitch;
    float roll;
    float maintain_heading=-1.0;

    uint32_t computed_haccuracy;
    uint32_t computed_accuracy;
    uint16_t time_step;

    float destination_y;
    float destination_x;
    int32_t count = 0 ;
    public:
        float Get_Global_Distance(int32_t long1, int32_t lat1, int32_t alt1, int32_t long2, int32_t lat2, int32_t alt2);
        float Get_Bearing();
        void Update_Position();
        void Set_Destination(float x, float y);
        float Get_X();
        float Get_Y();
        float Get_Z();
        uint32_t Get_Horizontal_Accuracy();
        uint32_t Get_3D_Accuracy();
        void setOriginLongitude(int32_t longitude);
        void setOriginLatitude(int32_t latitude);
        void setOriginAltitude(int32_t altitude);
        void computeScaling();
        void Navigate();
        void Set_Self_Drive(bool value);
        float Get_Local_Distance();
        float Get_Compass_Reading();
        void Update_IMU();
        void Set_Time_Step(uint16_t step);
        void Calibrate_Compass();
        void Maintain_Heading(float bearing);
        void Set_Heading(float heading);
        void Turn_To_Bearing(float bearing);
};
#endif