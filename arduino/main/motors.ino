#include "motors.h"

void Setup_Wheels() {
    //Left Wheel motor
    pinMode(ENAPin, OUTPUT);
    pinMode(LEFTBRAKE, OUTPUT);
    pinMode(LEFTDIRECTION, OUTPUT);

    //Right Wheel motor
    pinMode(ENBPin, OUTPUT);
    pinMode(RIGHTBRAKE, OUTPUT);
    pinMode(RIGHTDIRECTION, OUTPUT);

    // Make sure the wheels are stopped
    wheels.set_speeds(0, 0);
}


void Setup_Relays() {
    Serial.println(F("Setup Relays"));
    pinMode(Relay_Motors, OUTPUT);
    delay(5);
    Turn_Off_Relay();
    delay(5);
}

void  Turn_On_Relay() {
    Serial.print(F("Relay:ON|"));
    digitalWrite(Relay_Motors, LOW);                         // Turn on the relay for the main battery power
}

void  Turn_Off_Relay() {
    Serial.print(F("Relay:Off|"));
    digitalWrite(Relay_Motors, HIGH);                         // Turn off the relay for the main battery power
}

void Wheels::set_speeds (short left, short right) {
    left_speed=left;
    right_speed=right;
    if(left >= 0) {
        // Set pins to move left forwards
        digitalWrite(LEFTBRAKE, LOW);
        digitalWrite(LEFTDIRECTION, LOW);
    } else {
        // Set pins to move left backwards
        digitalWrite(LEFTBRAKE, LOW);
        digitalWrite(LEFTDIRECTION, HIGH);
    }
    if(right >= 0) {
        // Set pins to move right forwards
        digitalWrite(RIGHTBRAKE, LOW);
        digitalWrite(RIGHTDIRECTION, LOW);
    } else {
        // Set pins to move right backwards
        digitalWrite(RIGHTBRAKE, LOW);
        digitalWrite(RIGHTDIRECTION, HIGH);
    }
    analogWrite(ENAPin, abs(left) );
    analogWrite(ENBPin, abs(right) );
    // Serial.print(F("Set wheels to: "));
    // Serial.print(left);
    // Serial.print(F(", "));
    // Serial.println(right);
}

short Wheels::get_left_speed() {
    return left_speed;
}

short Wheels::get_right_speed() {
    return right_speed;
}

void Setup_Blades() {
    Setup_Mower_Pins();
    blades.disable_blades();
    blades.set_left_blade(0);
    blades.set_right_blade(0);
    blades.set_trimmer_blade(0);
}

void Setup_Mower_Pins() {
    Serial.println(F("Setup Mower Pins"));
    pinMode(LL_EN, OUTPUT);
    pinMode(LR_EN, OUTPUT);
    pinMode(LPWM, OUTPUT);

    pinMode(RL_EN, OUTPUT);
    pinMode(RR_EN, OUTPUT);
    pinMode(RPWM, OUTPUT);

    pinMode(TL_EN, OUTPUT);
    pinMode(TR_EN, OUTPUT);
    pinMode(TPWM, OUTPUT);
}

void Blades::enable_blades(){
    digitalWrite(LR_EN, HIGH);
    digitalWrite(LL_EN, HIGH);

    digitalWrite(RR_EN, HIGH);
    digitalWrite(RL_EN, HIGH);

    digitalWrite(TR_EN, HIGH);
    digitalWrite(TL_EN, HIGH);

    Serial.println(F("Blades:ON_|"));
}

void Blades::disable_blades(){
    digitalWrite(LR_EN, LOW);
    digitalWrite(LL_EN, LOW);

    digitalWrite(RR_EN, LOW);
    digitalWrite(RL_EN, LOW);

    digitalWrite(TR_EN, LOW);
    digitalWrite(TL_EN, LOW);

    Serial.println(F("Blades:OFF_|"));
}

void Blades::set_left_blade(unsigned char left){
    analogWrite(LPWM, left);
}
void Blades::set_right_blade(unsigned char right){
    analogWrite(RPWM, right);
}
void Blades::set_trimmer_blade(unsigned char trimmer){
    analogWrite(TPWM, trimmer);
}

unsigned char Blades::get_left_speed(){
    return left_speed;
}

unsigned char Blades::get_right_speed(){
    return right_speed;
}

unsigned char Blades::get_trimmer_speed(){
    return trimmer_speed;
}