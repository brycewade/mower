#include "location.h"
#include "Kalman.h"
using namespace BLA;

BNO080 myIMU;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


//------------------------------------
/****  MODELIZATION PARAMETERS  ****/
//------------------------------------

#define Nstate 3 // position, velocity, acceleration
#define Nobs 3   // position, velocity, acceleration

// measurement std
#define n_p 0.3
#define n_v 0.5
#define n_a 5.0
// model std (1/inertia)
#define m_p 0.1
#define m_v 0.1
#define m_a 0.8

#define IDENTITY_MATRIX {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}


KALMAN<Nstate,Nobs> Kx; // your Kalman filter
KALMAN<Nstate,Nobs> Ky; // your Kalman filter
// KALMAN<Nstate,Nobs> Kz; // your Kalman filter

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
    // Kz.F = IDENTITY_MATRIX;
    // Kz.H = IDENTITY_MATRIX;
    // Kz.R = {n_p*n_p, 0.0,     0.0,
    //         0.0,     n_v*n_v, 0.0,
    //         0.0,     0.0,     n_a*n_a};
    // Kz.Q = {m_p*m_p, 0.0,     0.0,
    //         0.0,     m_v*m_v, 0.0,
	// 		0.0,     0.0,     m_a*m_a};
}

void Setup_Scaling(){
    location.setOriginLongitude(ORIGIN_LONGITUDE);
    location.setOriginLatitude(ORIGIN_LATITUDE);
    location.setOriginAltitude(ORIGIN_ALTITUDE);
    location.computeScaling();
}

void Setup_Compass() {
    int status;
    float offset = 0.00f;
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
    // Read the offset from EEPROM
    EEPROM.get(address, offset);
    Serial.print(F("Setting compass offset to "));
    Serial.println(offset);
    location.Set_Yaw_Offset(offset);
}

void Location::Set_Yaw_Offset(float offset){
    yaw_offset = offset;
}

void Scan_and_reset_I2C(){
    byte error, address;
    int nDevices;

    for(address = 1; address < 127; address++ ) 
    {
        // Skip 0x68 and 0x3c we know what is there
        if((address != 104) && (address!=60))
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
    // Serial.print("X Distance: ");
    // Serial.print(x);
    // Serial.print(" Y Distance: ");
    // Serial.println(y);
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

void Location::Calibrate_Compass() {
    float raw_yaw;

    Serial.println("Setting compass offset");
    //Set the offset to 0 and read the compass
    yaw_offset=0.0;
    Serial.println(yaw_offset);
    raw_yaw = (myIMU.getYaw()) * 180.0 / PI;
    yaw_offset = Adjust_Yaw(raw_yaw);
    Serial.println(yaw_offset);
    EEPROM.put(0, yaw_offset);
    Serial.print("Compass offset set to ");
    Serial.println(yaw_offset);
}

float Location::Get_Compass_Reading(){
    Update_IMU();
    return yaw;
}

float Location::Adjust_Yaw(float yaw){
    yaw = -yaw;
    yaw -= yaw_offset;
    if (yaw < 0) {
        yaw += 360;
    }
    if (yaw >= 360) {
        yaw -= 360;
    }
    return(yaw);
}

void Location::Update_IMU(){
    float raw_yaw;
    if (myIMU.dataAvailable() == true)
    {
        roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
        pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
        raw_yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
        yaw=Adjust_Yaw(raw_yaw);
    }
}

void Location::Update_Position(){
    float gps_x;
    float gps_y;
    float gps_z;
    long current_time;
    float dt;
    // char buffer[255];
    BLA::Matrix<Nstate,Nobs> F;
    BLA::Matrix<Nobs> obsx; // observation vector
    BLA::Matrix<Nobs> obsy; // observation vector
    // BLA::Matrix<Nobs> obsz; // observation vector

    current_time=millis();
    dt=(current_time-last_update)/1000;
    last_update = current_time;
    if (current_time-last_gps_measurement > 100)
    {
        if (myGPS.getPVT())
        {
            dt = (current_time-last_gps_measurement)/1000;
            F = {1.0, dt,  dt*dt/2,
                0.0, 1.0, dt,
                0.0, 0.0, 1.0};
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
            gps_x = float(gps_longitude-origin_longitude);
            gps_x += gps_longitudeHP/100;
            gps_x *= longitude_to_meters;
            gps_y = float(gps_latitude-origin_latitude);
            gps_y += float(gps_latitudeHP)/100;
            gps_y *= latitude_to_meters;
            fixType = myGPS.getCarrierSolutionType();

            obsx = {gps_x, gps_velE/100, 0.0};
            obsy = {gps_y, gps_velN/100, 0.0};
            // obsz = {gps_z, -gps_velD/100, 0.0};
            Kx.F = Ky.F = F;
            Kx.update(obsx);
            Ky.update(obsy);
            // Kz.update(obsz);
            Ex = Kx.x;
            Ey = Ky.x;

            return;
        }
    }
    // Ok, no GPS updates, so just update our estimate of where we are
    F = {1.0, dt,  dt*dt/2,
         0.0, 1.0, dt,
         0.0, 0.0, 1.0};
    Ex = F * Ex;
    Ey = F * Ey;

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

    drift=bearing-yaw;
    if(drift>180)
        drift-=360;
    if(drift<=-180)
        drift+=360;
    if((drift>-10)&&(drift<0))
        drift*=3;
    if((drift<10)&&(drift>0))
        drift*=3;
    //If off by more than 15 degrees just stop and turn
    if((drift>15)||(drift<-15)){
        drift*=6;
    }
    // Serial.print("BHD: ");
    // Serial.print(bearing);
    // Serial.print(" ");
    // Serial.print(yaw);
    // Serial.print(" ");
    // Serial.println(drift);
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
    while((drift>3)||(drift<-3)){
        heading=Get_Compass_Reading();
        drift=bearing-heading;
        if(drift>180)
            drift-=360;
        if(drift<=-180)
            drift+=360;
        // Serial.print("BHD: ");
        // Serial.print(bearing);
        // Serial.print(" ");
        // Serial.print(heading);
        // Serial.print(" ");
        // Serial.println(drift);
        power=drift*255/180;
        left=int(power+0.5);
        if((left>0)&&(left<MIN_TURN_POWER))
            left=MIN_TURN_POWER;
        if((left<0)&&(left>-MIN_TURN_POWER))
            left=-MIN_TURN_POWER;
        right=-left;
        wheels.set_speeds(left,right);
        Update_Display();
    }
    wheels.set_speeds(0,0);
    Serial.print("Done turning to heading: ");
    Serial.println(bearing);
}

void Location::Set_Time_Step(uint16_t step){
    time_step = step;
}

void Location::Setup_Display(){
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
}

void Location::Update_Display(){
    char buffer[17];
    char buffer2[10];
    char buffer3[10];
    Serial.write(27);       // ESC command
    Serial.print("[2J");    // clear screen command
    Serial.write(27);
    Serial.print("[H");     // cursor to home command
    display.clearDisplay();
    display.setCursor(0,0);
    dtostrf(Get_X(),5,2,buffer2);
    dtostrf(Get_Y(),5,2,buffer3);
    sprintf(buffer, "LOC %s, %s", buffer2, buffer3);
    display.println(buffer);
    Serial.println(buffer);
    dtostrf(destination_x,5,2,buffer2);
    dtostrf(destination_y,5,2,buffer3);
    sprintf(buffer, "DES %s, %s", buffer2, buffer3);
    display.println(buffer);
    Serial.println(buffer);
    dtostrf(Get_Bearing(),6,2,buffer2);
    sprintf(buffer, "BEARING %s", buffer2);
    display.println(buffer);
    Serial.println(buffer);
    dtostrf(yaw,6,2,buffer2);
    sprintf(buffer, "HEADING %s", buffer2);
    display.println(buffer);
    Serial.println(buffer);
    dtostrf(Get_Local_Distance(),5,2,buffer2);
    sprintf(buffer, "DISTANCE %s", buffer2);
    display.println(buffer);
    Serial.println(buffer);
    sprintf(buffer, "WHL L %3d R %3d", wheels.get_left_speed(), wheels.get_right_speed());
    display.println(buffer);
    Serial.println(buffer);
    sprintf(buffer, "Fix %d%d Acc %d", fixType, myIMU.getMagAccuracy(), gps_haccuracy);
    display.println(buffer);
    Serial.println(buffer);
    display.display();
}