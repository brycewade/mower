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
    int value;
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
                case 92: {
                        ptr = strtok(NULL, delim);
                        while(ptr != NULL) {
                            letter = ptr[0];
                            value = atoi(ptr+1);
                            switch(letter) {
                                case 'X':
                                    location.setOriginX(static_cast<int32_t>(value));
                                    break;
                                case 'Y':
                                    location.setOriginY(static_cast<int32_t>(value));
                                    break;
                                case 'Z':
                                    location.setOriginZ(static_cast<int32_t>(value));
                                    break;
                            }
                            ptr = strtok(NULL, delim);
                        }
                        send_ok();
                    }
                    break;
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
                default:
                    send_err();
                    break;
            }
            break;
        case 'M':
            // Serial.println(F("It's a M code"));
            switch(value){
                case 114:
                    Serial.print(location.Get_Latitude());
                    Serial.print(F(" "));
                    Serial.print(location.Get_Longitude());
                    Serial.print(F(" "));
                    Serial.print(location.Get_Altitude());
                    Serial.print(F(" "));
                    Serial.print(location.Get_Horizontal_Accuracy());
                    Serial.print(F(" "));
                    Serial.print(location.Get_3D_Accuracy());
                    Serial.println();
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

