#include "location.h"
#include "SparkFun_Ublox_Arduino_Library.h"
MPU9250 IMU(Wire,0x68);

void Setup_Scaling(){
    location.setOriginLongitude(ORIGIN_LONGITUDE);
    location.setOriginLatitude(ORIGIN_LATITUDE);
    location.setOriginAltitude(ORIGIN_ALTITUDE);
    location.computeScaling();
}

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

void Scan_and_reset_I2C(){
    byte error, address;
    int nDevices;

    for(address = 1; address < 127; address++ ) 
    {
        // Skip 0x68 we know what is there
        if(address != 104)
        {
            // The i2c_scanner uses the return value of
            // the Write.endTransmisstion to see if
            // a device did acknowledge to the address.
            Wire.beginTransmission(address);
            error = Wire.endTransmission();

            if (error == 0)
            {
                Serial.print("I2C device found at address 0x");
                if (address<16) 
                    Serial.print("0");
                Serial.print(address,HEX);
                Serial.println("  !");
                if (myGPS.begin(Wire, address) == true)
                {
                    myGPS.setI2CAddress(GPS_I2C_Address);
                    Serial.print("Using I2C address ");
                    if (address<16) 
                        Serial.print("0");
                    Serial.print(address,HEX);
                    Serial.println(" to issue hard reset!");
                    // myGPS.hardReset();
                    delay(5000);
                    return;
                }
            }
        }
    }
    Serial.println("GPS Not found\n");
    delay(5000);
}

void Setup_GPS(){
    Serial.println(F("Initialziging GPS"));
    if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
    {
        Serial.println(F("Ublox GPS not detected at default I2C address. Scanning."));
        Scan_and_reset_I2C();
    }

    byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
    Serial.print("Current update rate:");
    Serial.println(rate);

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

    rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
    Serial.print("Current update rate:");
    Serial.println(rate);
}

// This is overkill, but might as well be accurate
float Location::Get_Global_Distance(int32_t long1, int32_t lat1, int32_t alt1, int32_t long2, int32_t lat2, int32_t alt2){
    float phi1, phi2, delta_phi, delta_lambda;
    float sin_delta_phi_over_2;
    float sin_delta_lambda_over_2;
    float a, c, d;

    phi1 = lat1*PI/1800000000;
    phi2 = lat2*PI/1800000000;
    delta_phi = (lat2-lat1)*PI/1800000000;
    delta_lambda = (long2-long1)*PI/1800000000;

    sin_delta_phi_over_2=sin(delta_phi/2);
    sin_delta_lambda_over_2=sin(delta_lambda/2);

    a = sin_delta_phi_over_2*sin_delta_phi_over_2 + cos(phi1)*cos(phi2)*sin_delta_lambda_over_2*sin_delta_lambda_over_2;
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    d = c * (RADIUS_OF_EARTH+(alt1+alt2)/2);

    return d;
}

float Location::Get_Local_Distance(){
    float distance;
    float x;
    float y;
    x=destination_x-computed_x;
    y=destination_y-computed_y;
    distance=sqrt(x*x+y*y);
    return(distance);
}

float Location::Get_Bearing(){
    float x, y, bearing;

    y = destination_y - computed_y;
    x = destination_x - computed_x;
    Serial.print("X Distance: ");
    Serial.print(x);
    Serial.print(" Y Distance: ");
    Serial.println(y);
    bearing = atan2(y,x) * 180 / PI;
    bearing=90-bearing;
    if (bearing < 0) {
        bearing += 360;
    }
    if (bearing >= 360) {
        bearing -= 360;
    }
    Serial.print("Bearing: ");
    Serial.println(bearing);
    return bearing;
}

float Location::Get_Compass_Reading() {
    float heading, magx, magy;
    int reading;
    magx=0;
    magy=0;
    for(reading=0; reading<HEADING_READINGS; reading++){
        IMU.readSensor();
        magx+=IMU.getMagY_uT();
        magy+=IMU.getMagX_uT();
        delay(5);
    }
    magx /= HEADING_READINGS;
    magy /= HEADING_READINGS;
    heading = atan2(magy,magx) * 180 / PI - 55/60;
    heading=-heading;
    if (heading<0){
        heading += 360;
    }
    if (heading>=360){
        heading -= 360;
    }
    return heading;
}

void Location::Update_Position(){
    float gps_x;
    float gps_y;
    float gpz_z;
    float predicted_x;
    float predicted_y;
    float predicted_z;
    long current_time;
    char buffer[255];

    current_time=millis();
    // Serial.print("Computed: ");
    // Serial.print(computed_x);
    // Serial.print(", ");
    // Serial.println(computed_y);
    predicted_x = computed_x + gps_velE * (current_time - last_update) / 1000;
    predicted_y = computed_y + gps_velN * (current_time - last_update) / 1000;
    predicted_z = computed_z - gps_velD * (current_time - last_update) / 1000;
    last_update = current_time;
    // Serial.print("Predicted: ");
    // Serial.print(predicted_x);
    // Serial.print(", ");
    // Serial.println(predicted_y);
    if (millis()-last_gps_measurement > 100)
    {
        if (myGPS.getPVT())
        {
            // Serial.print(F("loops since last update: "));
            // Serial.println(count);
            // Serial.print(F("millis since last update: "));
            // Serial.println(millis() - last_gps_measurement);
            last_gps_measurement = millis();  //Update the time stamp for the GPS reading
            gps_latitude = myGPS.getLatitude();
            gps_latitudeHP = myGPS.getLatHp();
            gps_longitude = myGPS.getLongitude();
            gps_longitudeHP = myGPS.getLongHp();
            gps_altitude = myGPS.getAltitude();
            gps_heading = myGPS.getHeading();
            gps_velN = myGPS.getVelN();
            gps_velE = myGPS.getVelE();
            gps_velD = myGPS.getVelD();
            gps_haccuracy = myGPS.getHorizontalAccuracy();
            // gps_accuracy = myGPS.getPositionAccuracy();
            // Convert position to meters from origin point
            // Serial.print("Computing X: ");
            // Serial.print(gps_longitude);
            // Serial.print(" ");
            // Serial.print(origin_longitude);
            // Serial.print(" ");
            gps_x = float(gps_longitude-origin_longitude);
            // Serial.print(gps_x);
            // Serial.print(" ");
            gps_x += gps_longitudeHP/100;
            // Serial.print(gps_x);
            // Serial.print(" ");
            gps_x *= longitude_to_meters;
            // Serial.println(gps_x);

            // Serial.print("Long: ");
            // Serial.println(gps_x);
            // sprintf(buffer, "Lon: (%ld - %ld + %d /100) * %4f = %4f\0", gps_longitude, origin_longitude, gps_longitudeHP, longitude_to_meters, gps_x);
            // Serial.println(buffer);
            // Serial.print("Computing Y: ");
            // Serial.print(gps_latitude);
            // Serial.print(" ");
            // Serial.print(origin_latitude);
            // Serial.print(" ");
            gps_y = float(gps_latitude-origin_latitude);
            // Serial.print(gps_y);
            // Serial.print(" ");
            gps_y += float(gps_latitudeHP)/100;
            // Serial.print(gps_y);
            // Serial.print(" ");
            gps_y *= latitude_to_meters;
            // Serial.println(gps_y);
            // Serial.print("Lat: ");
            // Serial.println(gps_y);
            // sprintf(buffer, "Lat: (%ld - %ld + %d /100 * %4f) = %4f\0", gps_latitude, origin_latitude, gps_latitudeHP, latitude_to_meters, gps_y);
            // Serial.println(buffer);

            computed_x = gps_x;
            computed_y = gps_y;
            computed_z = gps_altitude/1000;
            computed_haccuracy = gps_haccuracy;
            computed_accuracy = gps_accuracy;
            count=0;
            // sprintf(buffer, "Measured: %ld.%d, %ld.%d  ", gps_longitude, gps_longitudeHP, gps_latitude, gps_latitudeHP);
            // Serial.print(buffer);
            // Serial.print(gps_x);
            // Serial.print(", ");
            // Serial.println(gps_y);
            return;
        }
    }
    computed_x = predicted_x;
    computed_y = predicted_y;
    computed_z = predicted_z;
    count++;
}

void Location::Set_Destination(float x, float y){
    destination_x = x;
    destination_y = y;
}

float Location::Get_X(){
    return computed_x;
}

float Location::Get_Y(){
    return computed_y;
}

float Location::Get_Z(){
    return computed_z;
}

uint32_t Location::Get_Horizontal_Accuracy(){
    return computed_haccuracy;
}

uint32_t Location::Get_3D_Accuracy(){
    return computed_accuracy;
}

void Location::setOriginLongitude(int32_t longitude){
    origin_longitude = longitude;
}

void Location::setOriginLatitude(int32_t latitude){
    origin_latitude = latitude;
}

void Location::setOriginAltitude(int32_t altitude){
    origin_altitude = altitude;
}

void Location::computeScaling(){
    longitude_to_meters = Get_Global_Distance(origin_longitude-500, origin_latitude, origin_altitude, origin_longitude+500, origin_latitude, origin_altitude)/1000;
    latitude_to_meters = Get_Global_Distance(origin_longitude, origin_latitude-500, origin_altitude, origin_longitude, origin_latitude+500, origin_altitude)/1000;
}

void Location::Set_Self_Drive(bool value){
    self_drive=value;
}

void Location::Navigate(){
    float drift, bearing, heading;
    short left, right;
    if(self_drive)
    {
        Serial.print("Location: ");
        Serial.print(computed_x);
        Serial.print(", ");
        Serial.println(computed_y);
        Serial.print("Destination: ");
        Serial.print(destination_x);
        Serial.print(", ");
        Serial.println(destination_y);
        Serial.print("Distance: ");
        Serial.print(Get_Local_Distance());
        Serial.println(" m");
        if(Get_Local_Distance()<CLOSE_ENOUGH)
        {
            self_drive=false;
            wheels.set_speeds(0,0);
            send_ok();
            return;
        }
        bearing=Get_Bearing();
        heading=Get_Compass_Reading();
        drift=bearing-heading;
        if(drift>180)
            drift-=360;
        if(drift<=-180)
            drift+=360;
        Serial.print("BHD: ");
        Serial.print(bearing);
        Serial.print(" ");
        Serial.print(heading);
        Serial.print(" ");
        Serial.println(drift);
        // left=255;
        // if(drift<-3)
        //     left=0;
        // if(drift<-45)
        //     left=-255;
        // right=255;
        // if(drift>3)
        //     right=0;
        // if(drift>45)
        //     right=-255;
        left=max(min(255,255+int(drift*255/90+0.5)),-255);
        right=max(min(255,255-int(drift*255/90+0.5)),-255);
        wheels.set_speeds(left,right);
        delay(time_step);
    }
}

void Location::Set_Time_Step(uint16_t step){
    time_step = step;
}