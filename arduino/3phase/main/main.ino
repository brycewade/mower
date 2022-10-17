#define PHASE 5
#define DELAY PHASE * 1000 / 256

#define DIRECTION 2
#define CONTROL 3
#define BRAKE 4
#define LED 13
void setup()
{
  pinMode(CONTROL, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(BRAKE, LOW);
}

void loop()
{
  unsigned char speed;
  // Set for complete stop
  analogWrite(CONTROL, 0);
  // set forward
  digitalWrite(DIRECTION, LOW);
  digitalWrite(LED, LOW);
  // Ramp up speed
  for(speed=0; speed<255; speed++){
    analogWrite(CONTROL, speed);
    delay(DELAY);
  }
  // Ramp down speed
  for(speed=255; speed>0; speed--){
    analogWrite(CONTROL, speed);
    delay(DELAY);
  }
  // Change direction
  digitalWrite(DIRECTION, HIGH);
  digitalWrite(LED, HIGH);
  // Ramp up speed
  for(speed=0; speed<255; speed++){
    analogWrite(CONTROL, speed);
    delay(DELAY);
  }
  // Ramp down speed
  for(speed=255; speed>0; speed--){
    analogWrite(CONTROL, speed);
    delay(DELAY);
  }
}
