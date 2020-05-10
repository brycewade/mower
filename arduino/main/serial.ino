#include <stdio.h>
#include <string.h>
#include "motors.h"
#include "serial.h"

char serial_rx_buffer[RX_BUFFER_SIZE];
char tail=0;

void serial_init(){
    Serial.begin(115200);
    while (!Serial); //Wait for user to open terminal
    Serial.println("Mower Initializing");

}

void process_serial_input(){
    char endMarker = 13;

    while (Serial.available() > 0) {
        char received = Serial.read();
        if (received != endMarker) {
            serial_rx_buffer[tail]=received;
            Serial.print(received);
            tail++;
        } else {
            serial_rx_buffer[tail]='\0';
            Serial.println();
            // Serial.println(F("Newline received"));
            process_line(serial_rx_buffer);
            tail=0;
        }
    }
}

void process_line(char *string){
    char delim[] = " ";
    char letter;
    long value;
    float number;
    // Get the first word of the string:
    char *ptr = strtok(string, delim);
    if(ptr == NULL){
        return;
    }
    letter = ptr[0];
    value = atoi(ptr+1);
    switch(letter) {
        case 'G':
            // Serial.println(F("It's a G code"));
            switch(value){
                case 0:{
                        ptr = strtok(NULL, delim);
                        value=atoi(ptr);
                        location.Set_Time_Step(static_cast<uint16_t>(value));
                        send_ok();
                    }
                    break;
                case 1:{
                        float x = -999999;
                        float y = -999999;
                        ptr = strtok(NULL, delim);
                        while(ptr != NULL) {
                            letter = ptr[0];
                            number = atof(ptr+1);
                            switch(letter) {
                                case 'X':
                                    x = number;
                                    break;
                                case 'Y':
                                    y = number;
                                    break;
                            }
                            ptr = strtok(NULL, delim);
                        }
                        if((x==-999999) || (y==-999999)){
                            // We didn't get a properly formatted line
                            send_err();
                            return;
                        }
                        location.Set_Destination(x, y);
                        location.Set_Self_Drive(true);
                        location.Set_Heading(-1.0);
                    }
                    break;
                case 92: {
                        ptr = strtok(NULL, delim);
                        while(ptr != NULL) {
                            letter = ptr[0];
                            value = atol(ptr+1);
                            switch(letter) {
                                case 'X':
                                    location.setOriginLongitude(static_cast<int32_t>(value));
                                    break;
                                case 'Y':
                                    location.setOriginLatitude(static_cast<int32_t>(value));
                                    break;
                                case 'Z':
                                    location.setOriginAltitude(static_cast<int32_t>(value));
                                    break;
                            }
                            ptr = strtok(NULL, delim);
                        }
                        location.computeScaling();
                        send_ok();
                    }
                    break;
                case 132: {
                    location.Calibrate_Compass();
                    send_ok();
                    break;
                    }
                case 1000: {
                        short int left = -999;
                        short int right = -999;
                        ptr = strtok(NULL, delim);
                        while(ptr != NULL) {
                            letter = ptr[0];
                            value = atoi(ptr+1);
                            switch(letter) {
                                case 'L':
                                    left = value;
                                case 'R':
                                    right = value;
                            }
                            ptr = strtok(NULL, delim);
                        }
                        if((left==-999) || (right==-999)){
                            // We didn't get a properly formatted line
                            send_err();
                            return;
                        }
                        //Sanitize the numbers
                        if(left < -255){
                            left = -255;
                        }
                        if(left > 255){
                            left = 255;
                        }
                        if(right < -255){
                            right = -255;
                        }
                        if(right > 255){
                            right = 255;
                        }
                        location.Set_Self_Drive(false);
                        location.Set_Heading(-1.0);
                        wheels.set_speeds(left, right);
                        send_ok();
                    }
                    break;
                case 1001: {
                        ptr = strtok(NULL, delim);
                        while(ptr != NULL) {
                            letter = ptr[0];
                            value = atoi(ptr+1);
                            switch(letter) {
                                case 'L':
                                    if (value > 255){
                                        value=255;
                                    }
                                    if (value < 0){
                                        value=0;
                                    }
                                    blades.set_left_blade(static_cast<unsigned char>(value));
                                    break;
                                case 'R':
                                    if (value > 255){
                                        value=255;
                                    }
                                    if (value < 0){
                                        value=0;
                                    }
                                    blades.set_right_blade(static_cast<unsigned char>(value));
                                    break;
                                case 'T':
                                    if (value > 255){
                                        value=255;
                                    }
                                    if (value < 0){
                                        value=0;
                                    }
                                    blades.set_trimmer_blade(static_cast<unsigned char>(value));
                                    break;
                            }
                            ptr = strtok(NULL, delim);
                        }
                        send_ok();
                    }
                    break;
                case 1002:{
                        ptr = strtok(NULL, delim);
                        number=-9999;
                        if(ptr != NULL)
                            number = atof(ptr);
                        if((number<0) || (number>=360)){
                            // We didn't get a properly formatted line
                            send_err();
                            return;
                        }
                        location.Set_Self_Drive(false);
                        location.Set_Heading(number);
                    }
                    break;
                case 1003:{
                        number = location.Get_Compass_Reading();
                        Serial.print(F("Holding heading: "));
                        Serial.println(number);
                        location.Set_Self_Drive(false);
                        location.Set_Heading(number);
                    }
                    break;
                default:
                    send_err();
                    break;
            }
            break;
        case 'M':
            // Serial.println(F("It's a M code"));
            switch(value){
                case 114:
                    Serial.print(location.Get_X());
                    Serial.print(F(" "));
                    Serial.print(location.Get_Y());
                    Serial.print(F(" "));
                    Serial.print(location.Get_Z());
                    Serial.print(F(" "));
                    Serial.print(location.Get_Compass_Reading());
                    Serial.print(F(" "));
                    Serial.print(location.Get_Horizontal_Accuracy());
                    Serial.print(F(" "));
                    Serial.print(location.Get_3D_Accuracy());
                    Serial.println();
                    send_ok();
                    break;
                case 1000:
                    Serial.println(location.Get_Compass_Reading());
                    send_ok();
                    break;
                default:
                    send_err();
                    break;
            }
    }
}

void send_err(){
    Serial.println(F("ERROR"));
}

void send_ok(){
    Serial.println(F("OK"));
}

