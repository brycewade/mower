#include <Wire.h>
#include "motors.h"
#include "location.h"

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
#include "MPU9250.h"

//Pin Setup for the wheel Motor Bridge Controller
//Right Wheel motor
#define ENAPin 7                // EN Pins need a digital pin with PWM
#define IN1Pin 6                // IN Pins dont need digital PWM 
#define IN2Pin 5
//Left Wheel motor
#define ENBPin 2                // EN Pins need a digital pin with PWM
#define IN3Pin 4                // IN Pins dont need digital PWM
#define IN4Pin 3
//Left Blade motor
#define LPWM 8
#define LR_EN 25
#define LL_EN 27
//Right Blade motor
#define RPWM 9
#define RR_EN 29
#define RL_EN 31
//Left Blade motor
#define TPWM 10
#define TR_EN 33
#define TL_EN 35

#define Relay_Motors 24 

SFE_UBLOX_GPS myGPS;
Wheels wheels;
Blades blades;
Location location;

void setup(){
    Serial.begin(115200);
    Serial.println(F("Initializing..."));
    Setup_Relays();
    Setup_Wheels();
    Setup_Blades();
    Turn_On_Relay();
    serial_init();
    Wire.begin();
    Setup_Filters();
    Setup_GPS();
    Setup_Scaling();
    location.Set_Time_Step(INITIAL_TIME_STEP);
    Setup_Compass();
    myGPS.setAutoPVT(true); //Tell the GPS to "send" each solution
    blades.enable_blades();
}

void loop(){
    process_serial_input();
    location.Update_Position();
    location.Update_IMU();
    location.Navigate();
}
