#include <Servo.h>
#include <SoftwareSerial.h>

Servo myServo;


#define servoPin 62
#define Ultra 66
#define BT_SERIAL_TX 10
#define BT_SERIAL_RX 11

#define Dir1LeftMotor 37
#define Dir1RightMotor 34
#define Dir1GripMotor 58
#define Dir1ArmMotor 43

#define Dir2LeftMotor 36
#define Dir2RightMotor 35
#define Dir2GripMotor 59
#define Dir2ArmMotor 42

#define pwmRight 12
#define pwmLeft 8
#define pwmArm 9
#define pwmGrip 5

#define Encoder1Right 18
#define Encoder1Left 19
#define Encoder1Arm 3

#define Encoder2Right 31
#define Encoder2Left 38
#define Encoder2Arm 49

#define interruptRight 18
#define interruptLeft 19
#define interruptArm 3

//all distance is in cm and angles in degrees
long duration;
int distance;
int FORWARD = 1;
int REVERSE = 0;
volatile  long enc_R = 0;
volatile  long enc_L = 0;
int rightDist;
int centerDist;
int leftDist;
int armCount = 0;

void setup() {
  // put your setup code here, to run once:
  myServo.attach(servoPin);
 
  pinMode(pwmArm, OUTPUT);
  pinMode(Dir1ArmMotor, OUTPUT);
  pinMode(Dir2ArmMotor, OUTPUT);

  pinMode(pwmLeft, OUTPUT);
  pinMode(Dir1LeftMotor, OUTPUT);
  pinMode(Dir2LeftMotor, OUTPUT);
    
  pinMode(pwmRight, OUTPUT);
  pinMode(Dir1RightMotor, OUTPUT);
  pinMode(Dir2RightMotor, OUTPUT);

  pinMode(Encoder1Right, INPUT);
  pinMode(Encoder2Right, INPUT);

  pinMode(Encoder1Left, INPUT);
  pinMode(Encoder2Left, INPUT);
  
  Serial.begin(115200);
  Serial3.begin(115200);
  
  attachInterrupt(5,ReadEnR,CHANGE);
  attachInterrupt(4,ReadEnL,CHANGE);
  attachInterrupt(1, readArmEncoder, HIGH);

    
  //Set PWM to 8kHZ
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

void loop() {
  //Use Switch case for Bluetooth connection
  
  while(Serial3.available() != 0){  
  char  message = Serial3.read();
  
  switch (message){
  case '1':
     moveForward();
     delay(250);
     Stop();
     break;
  case '2':
    moveBack();
    delay(250);
    Stop();
    break;
 case '3':
    turnRight();
    delay(100);
    Stop();
    break;
 case '4':
    turnLeft();
    delay(100);
    Stop();
    break;
 case '5':
    armUp();
    break;
 case '6':
  //start autonomous
  Start();
  break;
 case '7':
 armDown();
 break;
 
 case '8':
 Grip();
 break;

 case '9':
 unGrip();
 break;
 
 default:
    Stop();
    break;    
    }
  }
}

void Start() {
  //Autonomous movement
  while(true){
  Stop();
  ServoSweepCenter();
  delay(100);
  ServoSweepLeft();
  delay(100);
  ServoSweepRight();
  delay(100);
  if (centerDist < 18){
    if (rightDist > leftDist){
      turnRightBig();
      delay(300);
      moveForward();
      delay(200);
    }
    else{
      turnLeftBig();
      delay(300);
      moveForward();
      delay(200);
    }
  }
  else if (rightDist <= 14){
    turnLeft();
  }
  else if (leftDist <= 14){
    turnRight();
  }
  else{
    moveForward();
  }
  delay(500);
}
}

void ServoSweepCenter(){
  //Sensor reads Center distance away from the Rover
  centerDist = 0;

  myServo.write(95);
  delay(500);
  Distance();
  centerDist = distance;
  Serial.print("Center Distance: ");
  Serial.println(centerDist);
  delay(500);
}

void ServoSweepLeft(){
  //Sensor reads Left distance away from the Rover
  leftDist = 0;
   
  myServo.write(190);
  delay(500);
  Distance();
  leftDist = distance;
  Serial.print("Left Distance: ");
  Serial.println(leftDist);
  delay(500);
}
void ServoSweepRight(){ 
  //Sensor reads Right distance away from the Rover
  rightDist =0;
  
  myServo.write(0);
  delay(500);
  Distance();
  rightDist = distance;
  Serial.print("Right Distance: ");
  Serial.println(rightDist);
  delay(500);
}

int Distance(){
  //Sensor calculates distance
  pinMode(Ultra, OUTPUT);
  digitalWrite(Ultra, LOW);
  delayMicroseconds(5);
  digitalWrite(Ultra,HIGH);
  delayMicroseconds(10);
  digitalWrite(Ultra,LOW);
  pinMode(Ultra,INPUT);
  duration = pulseIn(Ultra,HIGH);
  distance = duration / 58;
  delay(50);
  return distance;
}

void moveForward(){
   //Right move forward
  digitalWrite(Dir1RightMotor, HIGH);
  digitalWrite(Dir2RightMotor,LOW);
  analogWrite(pwmRight,200);


  //Left move forward
  digitalWrite(Dir1LeftMotor,LOW);
  digitalWrite(Dir2LeftMotor,HIGH);
  analogWrite(pwmLeft,200);
}

void turnLeft(){
  //Right move forward
  digitalWrite(Dir1RightMotor, HIGH);
  digitalWrite(Dir2RightMotor,LOW);
  analogWrite(pwmRight,100);


  //Left does not move
  digitalWrite(Dir1LeftMotor,HIGH);
  digitalWrite(Dir2LeftMotor,HIGH);
  analogWrite(pwmLeft,100);
}

void turnRight(){
  //Right does not move
  digitalWrite(Dir1RightMotor,HIGH);
  digitalWrite(Dir2RightMotor,HIGH);
  analogWrite(pwmRight,100);


  //Left move forward
  digitalWrite(Dir1LeftMotor,LOW);
  digitalWrite(Dir2LeftMotor,HIGH);
  analogWrite(pwmLeft,100);
}

void turnLeftBig(){
  //Turns left about 90 degrees
  int enc_L1 = enc_L + 350; //746 ticks is 1 wheel rotation; 350 ticks gives about 90 degrees
  while (enc_L <= enc_L1){
    //Right move forward
    digitalWrite(Dir1RightMotor, HIGH);
    digitalWrite(Dir2RightMotor, LOW);
    analogWrite(pwmRight,220);


    //Left move back
    digitalWrite(Dir1LeftMotor,HIGH);
    digitalWrite(Dir2LeftMotor,LOW);
    analogWrite(pwmLeft,220);
    Serial.print("Left Count: ");
    Serial.println(enc_L);
  }
}

void turnRightBig()
{
  //Turns right about 90 degrees
  int enc_R1 = enc_R + 350; //746 ticks is 1 wheel rotation; 350 ticks gives about 90 degrees
  while (enc_R <= enc_R1){
    //Right move back
    digitalWrite(Dir1RightMotor,LOW);
    digitalWrite(Dir2RightMotor,HIGH);
    analogWrite(pwmRight,220);


    //Left move forward
    digitalWrite(Dir1LeftMotor,LOW);
    digitalWrite(Dir2LeftMotor,HIGH);
    analogWrite(pwmLeft,220);
    Serial.print("Right Count: ");
    Serial.println(enc_R);
  }
}

void Stop()
{
  //Right Stop
  digitalWrite(Dir1RightMotor, HIGH);
  digitalWrite(Dir2RightMotor,HIGH);
  analogWrite(pwmRight,0);


  //Left Stop
  digitalWrite(Dir1LeftMotor,HIGH);
  digitalWrite(Dir2LeftMotor,HIGH);
  analogWrite(pwmLeft,0);
}

void ReadEnR()
{
    //Right encoder, reads pin18 and pin31
    if(digitalRead(18)==HIGH && digitalRead(31)==LOW){
    enc_R++;
    }
    else if(digitalRead(18)==LOW && digitalRead(31)==HIGH){
    enc_R++;
    }
    else{
    enc_R--;
    }
}
void ReadEnL()
{
    //Left encoder, reads pin19 and pin38
    if(digitalRead(19)==HIGH && digitalRead(38)==HIGH){
    enc_L++;
    }
    else if(digitalRead(19)==LOW && digitalRead(38)==LOW){
    enc_L++;
    }
    else{
    enc_L--;
    }
}

void Grip(){
  //Tightens Grip
  digitalWrite(Dir1GripMotor, HIGH);
  digitalWrite(Dir2GripMotor, LOW);
  analogWrite(pwmGrip, 128);
  delay(500);
  digitalWrite(Dir1GripMotor, HIGH);
  digitalWrite(Dir2GripMotor, HIGH);   
  return;
}

void unGrip(){
  //Loosen Grip
  digitalWrite(Dir2GripMotor, HIGH);
  digitalWrite(Dir1GripMotor, LOW);
  analogWrite(pwmGrip, 255);  
  delay(500);
  digitalWrite(Dir1GripMotor, HIGH);
  digitalWrite(Dir2GripMotor, HIGH);
  return;
 }

 void armUp(){
  //Moves arm up
    digitalWrite(Dir1ArmMotor, LOW);
    digitalWrite(Dir2ArmMotor,HIGH);
    analogWrite(pwmArm,255);
    delay(300);  
  
  digitalWrite(Dir1ArmMotor,HIGH);
  digitalWrite(Dir2ArmMotor,HIGH);
  delay(1000);
  return;
}
void armDown(){
  //Moves arm down
    digitalWrite(Dir1ArmMotor, HIGH);
    digitalWrite(Dir2ArmMotor,LOW);
    analogWrite(pwmArm,255);
    delay(300);   
  
  digitalWrite(Dir1ArmMotor,HIGH);
  digitalWrite(Dir2ArmMotor,HIGH);
  delay(1000);  
}

void readArmEncoder(){
  //Reads Arm Encoder
  if(digitalRead(Encoder1Arm) == HIGH && digitalRead(Encoder2Arm) == LOW)
  {armCount++;}
  else if(digitalRead(Encoder1Arm) == LOW && digitalRead(Encoder2Arm) == HIGH)
  {armCount++;}
  else
  {armCount--;}
  Serial.print(armCount);
  Serial.println("");
}

void moveBack(){
    //Right move back
  digitalWrite(Dir1RightMotor, LOW);
  digitalWrite(Dir2RightMotor,HIGH);
  analogWrite(pwmRight,128);


  //Left move back
  digitalWrite(Dir1LeftMotor,HIGH);
  digitalWrite(Dir2LeftMotor,LOW);
  analogWrite(pwmLeft,128);
}
