#include "location.h"
#include "Kalman.h"
using namespace BLA;

BNO080 myIMU;


//------------------------------------
/****  MODELIZATION PARAMETERS  ****/
//------------------------------------

#define Nstate 3 // position, velocity, acceleration
#define Nobs 3   // position, velocity, acceleration

// measurement std
#define n_p 0.3
#define n_v 0.1
#define n_a 5.0
// model std (1/inertia)
#define m_p 0.1
#define m_v 0.1
#define m_a 0.8

#define IDENTITY_MATRIX {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}


KALMAN<Nstate,Nobs> Kx; // your Kalman filter
KALMAN<Nstate,Nobs> Ky; // your Kalman filter
KALMAN<Nstate,Nobs> Kz; // your Kalman filter

void Setup_Filters(){
    Kx.F = IDENTITY_MATRIX;
    Kx.H = IDENTITY_MATRIX;
    Kx.R = {n_p*n_p, 0.0,     0.0,
            0.0,     n_v*n_v, 0.0,
            0.0,     0.0,     n_a*n_a};
    Kx.Q = {m_p*m_p, 0.0,     0.0,
            0.0,     m_v*m_v, 0.0,
			0.0,     0.0,     m_a*m_a};
    Ky.F = IDENTITY_MATRIX;
    Ky.H = IDENTITY_MATRIX;
    Ky.R = {n_p*n_p, 0.0,     0.0,
            0.0,     n_v*n_v, 0.0,
            0.0,     0.0,     n_a*n_a};
    Ky.Q = {m_p*m_p, 0.0,     0.0,
            0.0,     m_v*m_v, 0.0,
			0.0,     0.0,     m_a*m_a};
    Kz.F = IDENTITY_MATRIX;
    Kz.H = IDENTITY_MATRIX;
    Kz.R = {n_p*n_p, 0.0,     0.0,
            0.0,     n_v*n_v, 0.0,
            0.0,     0.0,     n_a*n_a};
    Kz.Q = {m_p*m_p, 0.0,     0.0,
            0.0,     m_v*m_v, 0.0,
			0.0,     0.0,     m_a*m_a};
}

void Setup_Scaling(){
    location.setOriginLongitude(ORIGIN_LONGITUDE);
    location.setOriginLatitude(ORIGIN_LATITUDE);
    location.setOriginAltitude(ORIGIN_ALTITUDE);
    location.computeScaling();
}

void Setup_Compass() {
    int status;
    float offset;
    float scale;
    int address=0;
    // start communication with IMU 
    status = myIMU.begin();
    if (status == false) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while(1) {}
    }
    myIMU.calibrateAll();

    myIMU.enableRotationVector(50); //Send data update every 50ms
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form roll, pitch, yaw"));

    myIMU.enableAccelerometer(50); //Send data update every 50ms
    Serial.println(F("Accelerometer enabled"));
    Serial.println(F("Output in form x, y, z, in m/s^2"));
}

void Location::Calibrate_Compass() {
    /*
    Serial.print("Calibrating Accelerometer, spinning around for a bit");
    myIMU.calibrateAccelerometer();
    while(1) {
        if(myIMU.dataAvailable() == true) {

        }
    }
    delay(3000);
    */
    Serial.print("Calibrating Gyroscope, staying static for 3 seconds");
    myIMU.calibrateMagnetometer();
    wheels.set_speeds(-255, 255);
    delay(5000);
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
    x=destination_x-Ex(0);
    y=destination_y-Ey(0);
    distance=sqrt(x*x+y*y);
    return(distance);
}

float Location::Get_Bearing(){
    float x, y, bearing;

    y = destination_y - Ey(0);
    x = destination_x - Ex(0);
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
    // Serial.print("Bearing: ");
    // Serial.println(bearing);
    return bearing;
}

float Location::Get_Compass_Reading(){
    return yaw;
}

void Location::Update_IMU(){
    if (myIMU.dataAvailable() == true)
    {
        roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
        pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
        yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
        yaw += 90;
        if (yaw < 0) {
            yaw += 360;
        }
        if (yaw >= 360) {
            yaw -= 360;
        }
        Serial.print(roll, 1);
        Serial.print(F(","));
        Serial.print(pitch, 1);
        Serial.print(F(","));
        Serial.print(yaw, 1);

        Serial.println();
    }
}
void Location::Update_Position(){
    float gps_x;
    float gps_y;
    float gps_z;
    long current_time;
    long dt;
    char buffer[255];
    BLA::Matrix<Nstate,Nobs> F;
    BLA::Matrix<Nobs> obsx; // observation vector
    BLA::Matrix<Nobs> obsy; // observation vector
    BLA::Matrix<Nobs> obsz; // observation vector

    current_time=millis();
    dt=current_time-last_update;
    F = {1.0, dt,  dt*dt/2,
         0.0, 1.0, dt,
         0.0, 0.0, 1.0};
    if (current_time-last_gps_measurement > 100)
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
            Serial.print(F("Updating position from GPS: "));
            Serial.print(gps_x);
            Serial.print(" ");
            Serial.print(gps_y);
            Serial.print(" ");
            Serial.print(gps_altitude/1000);
            Serial.print(" ");
            Serial.print(gps_velE);
            Serial.print(" ");
            Serial.print(gps_velN);
            Serial.print(" ");
            Serial.println(gps_velD);

            obsx = {gps_x, gps_velE, 0.0};
            obsy = {gps_y, gps_velN, 0.0};
            obsz = {gps_z, -gps_velD/100, 0.0};
            Kx.F = Ky.F = Kz.F = F;
            Kx.update(obsx);
            Ky.update(obsy);
            Kz.update(obsz);
            Ex = Kx.x;
            Ey = Ky.x;
            Ez = Kz.x;
            // sprintf(buffer, "Measured: %ld.%d, %ld.%d  ", gps_longitude, gps_longitudeHP, gps_latitude, gps_latitudeHP);
            // Serial.print(buffer);
            // Serial.print(gps_x);
            // Serial.print(", ");
            // Serial.println(gps_y);
            return;
        }
    }
    // Ok, no GPS updates, so just update our estimate of where we are
    Ex = F * Ex;
    Ey = F * Ey;
    Ez = F * Ez;

    count++;
}

void Location::Set_Destination(float x, float y){
    destination_x = x;
    destination_y = y;
}

float Location::Get_X(){
    return Ex(0);
}

float Location::Get_Y(){
    return Ey(0);
}

float Location::Get_Z(){
    return Ez(0);
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
    float drift, bearing;

    heading=Get_Compass_Reading();
    if(self_drive)
    {
        Serial.print("Time: ");
        Serial.println(millis());
        Serial.print("Location: ");
        Serial.print(Ex(0));
        Serial.print(", ");
        Serial.println(Ey(0));
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
            Serial.println(F("OK"));
            return;
        }
        bearing=Get_Bearing();
        Maintain_Heading(bearing);
    } else if(maintain_heading>=0){
        Maintain_Heading(maintain_heading);
    }
}

void Location::Maintain_Heading(float bearing){
    float drift;
    short left, right;

    drift=bearing-heading;
    if(drift>180)
        drift-=360;
    if(drift<=-180)
        drift+=360;
    if((drift>-10)&&(drift<0))
        drift=-10;
    if((drift<10)&&(drift>0))
        drift=10;
    Serial.print("BHD: ");
    Serial.print(bearing);
    Serial.print(" ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.println(drift);
    left=max(min(255,255+int(drift*255/90+0.5)),-255);
    right=max(min(255,255-int(drift*255/90+0.5)),-255);
    wheels.set_speeds(left,right);
    delay(time_step);
}

void Location::Set_Heading(float new_heading){
    maintain_heading = new_heading;
}

void Location::Turn_To_Bearing(float bearing){
    float drift=180.0;
    float power;
    short left, right, i;

    Serial.print("Turning to heading: ");
    Serial.println(bearing);
    Serial.print("Letting compass reading steady:");
    for(i=0; i<255; i++)
        heading=Get_Compass_Reading();
    while((drift>3)||(drift<-3)){
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
        power=drift*255/180;
        left=int(power+0.5);
        if((left>0)&&(left<MIN_TURN_POWER))
            left=MIN_TURN_POWER;
        if((left<0)&&(left>-MIN_TURN_POWER))
            left=-MIN_TURN_POWER;
        right=-left;
        wheels.set_speeds(left,right);
    }
    wheels.set_speeds(0,0);
    Serial.print("Letting compass reading steady:");
    for(i=0; i<255; i++)
        heading=Get_Compass_Reading();
    Serial.print("Done turning to heading: ");
    Serial.println(bearing);
}

void Location::Set_Time_Step(uint16_t step){
    time_step = step;
}