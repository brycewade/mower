#include "location.h"
#include "SparkFun_Ublox_Arduino_Library.h"
MPU9250 IMU(Wire,0x68);

void Setup_Compass() {
    int status;
    // start communication with IMU 
    status = IMU.begin();
    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while(1) {}
    }
    Serial.println(F("Setting Calibration of compass."));
    IMU.setMagCalX(29.55, 1.0009310987);
    IMU.setMagCalY(61.835, 1.00315345754);
    IMU.setMagCalZ(-32.61, 0.995942751262);
}

void Setup_GPS(){
    Serial.println(F("Initialziging GPS"));
    if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
    {
        Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }

    myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGPS.setNavigationFrequency(GPS_Sample_Rate); //Produce two solutions per second
    // myGPS.enableDebugging(Serial);

  // If we are going to change the dynamic platform model, let's do it here.
  // Possible values are:
  // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE

    if (myGPS.setDynamicModel(DYN_MODEL_PEDESTRIAN) == false) // Set the dynamic model to PORTABLE
    {
        Serial.println("***!!! Warning: setDynamicModel failed !!!***");
    } else {
        Serial.println("Dynamic platform model changed successfully!");
    }
    //Send high precision data over I2C with every fix
    if (myGPS.enableMessage(UBX_CLASS_NAV, UBX_NAV_HPPOSLLH, COM_PORT_I2C, 1) == false)
    {
        Serial.println("***!!! Warning: set UBX_NAV_HPPOSLLH rate failed !!!***");
    } else {
        Serial.println("UBX_NAV_HPPOSLLH rate set successfully!");
    }
    myGPS.setAutoPVT(true); //Tell the GPS to "send" each solution
    myGPS.saveConfiguration(); //Save the current settings to flash and BBR

    byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
    Serial.print("Current update rate:");
    Serial.println(rate);
}

// This is overkill, but might as well be accurate
float Location::Get_Distance(){
    float phi1, phi2, delta_phi, delta_lambda;
    float sin_delta_phi_over_2;
    float sin_delta_lambda_over_2;
    float a, c, d;

    phi1 = computed_latitude*PI/1800000000;
    phi2 = destination_latitude*PI/1800000000;
    delta_phi = (destination_latitude-computed_latitude)*PI/1800000000;
    delta_lambda = (destination_longitude-computed_longitude)*PI/1800000000;

    sin_delta_phi_over_2=sin(delta_phi/2);
    sin_delta_lambda_over_2=sin(delta_lambda/2);

    a = sin_delta_phi_over_2*sin_delta_phi_over_2 + cos(phi1)*cos(phi2)*sin_delta_lambda_over_2*sin_delta_lambda_over_2;
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    d = c * RADIUS_OF_EARTH;

    return d;
}

float Location::Get_Bearing(){
    float phi1, phi2, delta_lambda;
    float x, y, bearing;

    phi1 = computed_latitude*PI/1800000000;
    phi2 = destination_latitude*PI/1800000000;
    delta_lambda = (destination_longitude-computed_longitude)*PI/1800000000;

    y = sin(delta_lambda)*cos(phi2);
    x = cos(phi1)*sin(phi2)-sin(phi1)*cos(phi2)*cos(delta_lambda);

    bearing = atan2(y,x) * 180 / PI;
    if (bearing < 0) {
        bearing += 360;
    }
    return bearing;
}

void Location::Update_Position(){
    if (millis()-last_gps_measurement > 100)
    {
        if (myGPS.getPVT())
        {
            Serial.print(F("loops since last update: "));
            Serial.println(count);
            Serial.print(F("millis since last update: "));
            Serial.println(millis() - last_gps_measurement);
            last_gps_measurement = millis();  //Update the time stamp for the GPS reading
            gps_latitude = myGPS.getHighResLatitude();
            gps_longitude = myGPS.getHighResLongitude();
            gps_altitude = myGPS.getAltitude();
            gps_heading = myGPS.getHeading();
            gps_velN = myGPS.getVelN();
            gps_velE = myGPS.getVelE();
            gps_velD = myGPS.getVelD();
            gps_haccuracy = myGPS.getHorizontalAccuracy();
            gps_accuracy = myGPS.getPositionAccuracy();

            computed_latitude = gps_latitude;
            computed_longitude = gps_longitude;
            computed_altitude = gps_altitude;
            computed_haccuracy = gps_haccuracy;
            computed_accuracy = gps_accuracy;
            count=0;
        } else {
            
            count++;
            // delay(50);
        }
    } else {
        count++;
        // delay(50);
    }
}

void Location::Set_Destination(int32_t latitude, int32_t longitude){
    destination_latitude = latitude;
    destination_longitude = longitude;
}

int32_t Location::Get_Latitude(){
    return computed_latitude;
}

int32_t Location::Get_Longitude(){
    return computed_longitude;
}

int32_t Location::Get_Altitude(){
    return computed_altitude;
}

uint32_t Location::Get_Horizontal_Accuracy(){
    return computed_haccuracy;
}

uint32_t Location::Get_3D_Accuracy(){
    return computed_accuracy;
}