#include <arduino-timer.h>
#include <ezButton.h>
#include <Servo.h>

auto timer = timer_create_default();
unsigned long previousMillis = 0;
unsigned long buttonSeqPrevMillis = 0;
unsigned long interval = 1000;

//Motor X (Big one directly on rail)
int PWMX = 11; //Speed control 
int XIN1 = 13; //Direction
int XIN2 = 12; //Direction

//Motor Y (Big one on top of X)
int PWMY = 10; //Speed control 
int YIN1 = 9; //Direction
int YIN2 = 8; //Direction

//Motor Z (Small White)
int PWMZ = 6; //Speed control 
int ZIN1 = 7; //Direction
int ZIN2 = 5; //Direction

//IR sensor
int IR = 4;

//Servo Motor
int ServoPin = 3;
// Distances for the grabbing sequence:
int ServoAngleMax = 150; // Full Grab
int ServoAngleMin = 10; // Full Release
int ServoAngle = ServoAngleMax;
Servo clawServo;

//End Switch Pins
#define ESX0 A2
#define ESX1 2
#define ESY0 A4
#define ESY1 A5

#define VRX_PIN  A0
#define VRY_PIN  A1
#define SW_PIN   0
int ESArr[4] = {0,0,0,0};

ezButton button(SW_PIN);

int xValue = 0; 
int yValue = 0; 
int bValue = 0;
int buttonSequence = 0;

void setup(){
  Serial.begin(9600) ;

  pinMode(ESX0, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(ESY0, INPUT_PULLUP);
  pinMode(ESY1, INPUT_PULLUP);
  pinMode(IR, INPUT);

  clawServo.attach(ServoPin, 600, 2350);
  clawServo.write(ServoAngle);
  button.setDebounceTime(50); // set debounce time to 50 milliseconds
  runMotorTimed(2,0, 255, 500);
  while (digitalRead(IR)) {
    runMotorTimed(2, 1, 255, 100);
  }
}

void loop(){
  timer.tick(); // MUST call this
  button.loop(); // MUST call the loop() function first
  unsigned long currentMillis = millis();

  // read analog X and Y analog values
  xValue = mapper(analogRead(VRX_PIN));
  yValue = mapper(analogRead(VRY_PIN));
  bValue = button.getState();

  // set ESArr values
  setEndSwitchValues();

  if (button.isPressed()) {
    Serial.println("button sequence init");
    buttonSequence = 1;
    moveMotor(0, 0, 0);
    moveMotor(1, 0, 0);
    runButtonSequence();
  }

  if(!buttonSequence){
    /*if (Serial.available() > 1) {
      handleSerialRead();
    }*/
    if(xValue != 0){
      moveMotor(0, 255, xValue); 
    } else {
      moveMotor(0, 0, xValue); 
    }

    if(yValue != 0){
      moveMotor(1, 255, abs(yValue-3)); // This is a hack to reverse the directions of movement. The output spectrum of the joystick is reversed for the y-axis, so we have to do this.
    } else {
      moveMotor(1, 0, yValue); 
    }
  }
  
  if(currentMillis - previousMillis > interval) {
   previousMillis = currentMillis;
   if(buttonSequence){
    //Serial.println("CANNOT MOVE");
   } else {
    //Serial.println("I can move");
   }
  }
}

void moveMotor(int motor, int speed, int direction){
  // Hardcoded motor-switch mapping:
  int sw = -1;
  if (motor == 0 && direction == 1) sw = 0;
  else if (motor == 0 && direction == 2) sw = 1;
  else if (motor == 1 && direction == 1) sw = 2;
  else if (motor == 1 && direction == 2) sw = 3;
  // return if switch corresponding to movement direction is pressed.
  if (sw >= 0 && ESArr[sw]) speed = 0;

  
  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 0){
    digitalWrite(XIN1, inPin1);
    digitalWrite(XIN2, inPin2);
    analogWrite(PWMX, speed);
  } else if (motor == 1) {
    digitalWrite(YIN1, inPin1);
    digitalWrite(YIN2, inPin2);
    analogWrite(PWMY, speed);
  } else if (motor == 2){
    digitalWrite(ZIN1, inPin1);
    digitalWrite(ZIN2, inPin2);
    analogWrite(PWMZ, speed);
  }
}

int mapper(int analogValue){
  int val = 0;
  if(analogValue < 100){
    val = 1;
  } else if (analogValue > 900){
    val = 2;
  }
  return val;
}

void runButtonSequence(){
  runMotorTimed(2, 0, 255, 3500);
  setClawPos(ServoAngleMin);
  runMotorTimed(2, 0, 255, 2100);
  delay(1000);
  setClawPos(ServoAngleMax);
  while (digitalRead(IR))
    runMotorTimed(2, 1, 255, 100);
  delay(1000);  
  moveCarriageToOrigin();
  delay(1000);
  setClawPos(ServoAngleMin);
  delay(1000);
  setClawPos(ServoAngleMax);
  

  buttonSequence = 0;
  
}

void runMotorTimed(int motor, int dir, int speed, int time){
  unsigned long startTime = millis();
  unsigned long endTime = startTime;
  unsigned int timeToRun = time;
  
  while((endTime - startTime) <= timeToRun){
    button.loop();
    moveMotor(motor, speed, dir);
    endTime = millis();
  } 
  moveMotor(motor, 0, 0);
}

void setClawPos(int deg){
  int degDif = deg - ServoAngle;
  int singleStep = degDif / abs(degDif);
  for (degDif = abs(degDif); degDif > 0; degDif -= 1) {
    ServoAngle += singleStep;
    clawServo.write(ServoAngle);
    delay(3);
  }
}

void moveCarriageToOrigin() {
  int esx = !digitalRead(ESX1);
  int esy = !digitalRead(ESY1);
  if (esx) moveMotor(0, 255, 2);
  delay(10);
  if (esy) moveMotor(1, 255, 2);
  while(esx || esy) {
    esx = !digitalRead(ESX1);
    esy = !digitalRead(ESY1);
    moveMotor(0, esx * 255, 2);
    moveMotor(1, esy * 255, 2);
  }
}

void setEndSwitchValues(){
  // set global array of swith values [x0, x1, y0, y1]
  
  ESArr[0] = digitalRead(ESX0);
  ESArr[1] = digitalRead(ESX1);
  ESArr[2] = digitalRead(ESY0);
  ESArr[3] = digitalRead(ESY1);
}


void handleSerialRead() {
  String text = Serial.readStringUntil('\n');
  if (text.startsWith("up")) {
    text = text.substring(3, text.length());
    int value = text.toInt();
    runMotorTimed(2, 1, 255, value);
  }
  else if (text.startsWith("down")) {
    text = text.substring(5, text.length());
    int value = text.toInt();
    runMotorTimed(2, 0, 255, value);
  }
}
