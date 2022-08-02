#include <arduino-timer.h>
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

#include <ezButton.h>

#define VRX_PIN  A0 
#define VRY_PIN  A1 
#define SW_PIN   0 

ezButton button(SW_PIN);

int xValue = 0; 
int yValue = 0; 
int bValue = 0;
int buttonSequence = 0;

void setup(){
  Serial.begin(9600) ;
  
  button.setDebounceTime(50); // set debounce time to 50 milliseconds

}

void loop(){
  timer.tick(); // MUST call this
  button.loop(); // MUST call the loop() function first
  unsigned long currentMillis = millis();

  // read analog X and Y analog values
  xValue = mapper(analogRead(VRX_PIN));
  yValue = mapper(analogRead(VRY_PIN));
  bValue = button.getState();

  if (button.isPressed()) {
    Serial.println("button sequence init");
    buttonSequence = 1;
    runButtonSequence();
  }

  if(!buttonSequence){
    if(xValue != 0){
      moveMotor(0, 255, xValue); 
    } else {
      moveMotor(0, 0, xValue); 
    }
    
    if(yValue != 0){
      moveMotor(1, 255, (yValue != 1)); // invert direction so it matches with controls
    } else {
      moveMotor(1, 0, yValue); 
    }
  } else {
    // Force both axis to stop
    moveMotor(0, 0, xValue); 
    moveMotor(1, 0, yValue); 
  }
  
  if(currentMillis - previousMillis > interval) {
   previousMillis = currentMillis;
   if(buttonSequence){
    Serial.println("CANNOT MOVE");
   } else {
    Serial.println("I can move");
   }
  }
}


void moveMotor(int motor, int speed, int direction){

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
  runMotorTimed(2, 0, 255, 2000);
  
  delay(1000); 
  // servo.run() // make this worky
  
  runMotorTimed(2, 1, 255, 2000);

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
