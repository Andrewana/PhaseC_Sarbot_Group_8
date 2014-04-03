#include <Wire.h>
#include <L3G.h>
#include <Servo.h> 

//TROUBLESHOOTING
//IF ROBOT DOESN'T STOP AT 90 DEGREE TURNS, RECALIBRATE COMPASS AT MAZE
//IF ENCODER DOESN'T COUNT PROPERLY, MOVE ENCODER DISK CLOSER
//IF ROBOT MOVES BACKWARDS, MOVE THE FRONT DIGITAL SENSORS CLOSER TO THE CENTER OF BOT


#define minerPosition 0
#define startPosition 1

boolean goal = minerPosition;

Servo servolift;  // create servo object to control a servo 
// a maximum of eight servo objects can be created 

Servo servograb;

volatile long totalCount = 0;

//Define all 2 encoder, 2 Ir sensor
int backLeftIR = 0; //right back
int backRightIR = 0;
int FrontLeftIR = 0; //right front
int FrontRightIR = 0; //left

//Set Power motor level
double refLeftMotor = 0;
double refRightMotor = 0;

//current set motor speed
double leftMotor;
double rightMotor;

//Digital Pin out configuration for motor shield
int E1 = 6;
int M1 = 7; //left motor
int E2 = 5;
int M2 = 4; //right motor

//timer
unsigned long referenceTimer = 0;
unsigned long currentTimer = 0;
unsigned long differenceTimer = 0;

//Wheel Encoder values
int leftEncoderValue = 0; 
int leftEncoderPreviousState = 0;
int leftEncoderCurrentState = 0;
int rightEncoderValue = 0;
int rightEncoderPreviousState = 0;
int rightEncoderCurrentState = 0;
long leftEncoderCounter = 0;
long rightEncoderCounter = 0;
int cellDistance = 6; //should be 14!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
double distanceTravelled = 0;

//Define all four headings of compass; mini
L3G gyro;
int currentHeading = 0;
int setHeading = 0;

//Variables to track location of robot
int x = 1;
int y = 0;
int setDirection = 1;

//Boolean to ignore corridors
boolean ignore = true; //////////////////////////////MUST BE FALSE/////////////////////////////

//Instructions that the robot will follow, enter the following commands in the list
//"stop","straight","left","right","finish"
String instructionList[] = {
  "left",
  "right",
  "right",
  "left"};

int instructionCounter = 0;

int locationCounter = 0;


void setup() {
  Serial.begin(9600);
  Serial.println("sizeof(instructionList");
  Serial.println(sizeof(instructionList)/sizeof(instructionList[0]));

  servolift.attach(11);
  servograb.attach(12); 


  //Motor Pin Modes
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  
  //digitalWrite(2, HIGH);       // turn on pullup resistor
  attachInterrupt(0,updateLeftEncoder, RISING);
  attachInterrupt(1,updateRightEncoder, RISING);

  //Front Left Digital Sensor Pin Mode, it wasn't put on analog ports because there aren't any left
//  pinMode(2,INPUT);
//  pinMode(3,INPUT);
  
  Serial.println("Digital Pins configured.");

  //GYRO SETTINGS: CALIBRATE THE COMPASS IN THE LAB
  Wire.begin();
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  Serial.println("Gyro Loaded.");

  gyro.enableDefault();

  //SERVOLIFT 15 = all the way down, 160 = all the way up
  //SERVOGRAB 55 = all the way closed, 180 = all the way open


  servolift.write(160);
  servograb.write(180);
  delay(3000);

  clearEncoder();
  //test();
}

void updateLeftEncoder(){
  totalCount++;
}

void updateRightEncoder(){
  totalCount++;
}

void test(){
  moveForward();

  long currentCount = totalCount;
  long countDifference = 0;

  while (countDifference <4200){
    countDifference = totalCount-currentCount;
    moveForward();
  }

  tapBrake(75,500);
  shutdownRobot();
}

void goStraight(){
  digitalWrite(M2,HIGH);
  digitalWrite(M1,HIGH);
  analogWrite(E2, 0); //PWM Speed Control
  analogWrite(E1, 45); //PWM Speed Control
}

void loop(){
  
  updateAllSensors();

  if (totalCount>2400){
    ignore = false; 
  }

  if (backLeftIR <= 170 && ignore == false || backRightIR <= 170 && ignore == false) //Sees right hallway if right sees more than 8cm
  {

    if (totalCount<4800){

      long currentCount = totalCount;
      long countDifference = 0;

      driveMotor(-62,-58);

      while (countDifference < 250){
        countDifference = totalCount - currentCount;
      }

    }
    else{
      totalCount = 0;
      moveForward();
      while (totalCount<500){
        moveForward();
      }
    }

    totalCount = 0;
    nextInstruction();
    ignore = true;
    totalCount = 0;
    clearEncoder();
  }
  else//Go straight using compass and PID
  {
    moveForward(); 
  }
  
Serial.println(totalCount);
Serial.print("                ");
Serial.println(analogRead(3));
}

void pickup(){
  //SERVOLIFT 15 = all the way down, 160 = all the way up
  //SERVOGRAB 55 = all the way closed, 180 = all the way open

  for (int i = 170; i>15; i=i-2){
    servolift.write(i); // lower arm all the way down
    delay(10);
  }

  delay(500);

  for (int i = 180; i>55; i=i-3){ // close claw
    servograb.write(i);
    delay(7);
  }

  delay(500);

  for (int i = 15; i<160; i=i+2){
    servolift.write(i); // lower arm all the way down
    delay(40);
  }

  delay(500);
}

void updateAllSensors(){
  backLeftIR = analogRead(0);
  backRightIR = analogRead(1);
  FrontLeftIR = digitalRead(10);
  FrontRightIR = digitalRead(9);
  //  Serial.print("analog0");
  //  Serial.println(backLeftIR);
  //  Serial.print("                                              analog0");
  //  Serial.println(backRightIR);
}

void tapBrake(int a, int b){
  driveMotor(-leftMotor/abs(leftMotor)*80,-rightMotor/abs(rightMotor)*80);
  delay(a);
  driveMotor(0,0);
  delay(b);
}

void brakeTires(){
  driveMotor(leftMotor/abs(leftMotor)*44,rightMotor/abs(rightMotor)*58);
  delay(100);
  driveMotor(0,0);
  delay(250);
}

void moveForward(){
  updateAllSensors();

  long currentCount = totalCount;
  long countDifference = 0;

  if (FrontLeftIR == LOW){ //FRONT LEFT IR DETECTED A WALL. ROTATE RIGHT
    tapBrake(75,300);

    driveMotor(30, 0);

    while (countDifference < 200){
      countDifference = totalCount - currentCount;
    }

    tapBrake(75,300);

  }
  else if (FrontRightIR == LOW){ //FRONT RIGHT IR DETECTED A WALL. ROTATE LEFT
    tapBrake(75,300);

    driveMotor(0, 30);

    while (countDifference < 200){

      countDifference = totalCount - currentCount;
    }

    tapBrake(75,300);
  }
  else{
    driveMotor(62,53);
  }
}


void nextInstruction(){

  if (instructionList[instructionCounter] == "stop"){
    shutdownRobot();
  }
  else if (instructionList[instructionCounter] == "straight"){
    //do nothing
  }
  else if (instructionList[instructionCounter] == "left"){
    Serial.println("Turning to left");
    rotate("left");
    setDirection -= 1;
    if (setDirection<0){
      setDirection = 3;
    }
  }
  else if (instructionList[instructionCounter] == "right"){
    Serial.println("Turning to right");
    rotate("right");
    setDirection += 1;
    if (setDirection>3){
      setDirection = 0;
    }
  }
  else if (instructionList[instructionCounter] == "pickup"){
    Serial.println("Pickup miner");


  }


  instructionCounter ++;

  //this is triggered after the last instruction is triggered
  if (instructionCounter > sizeof(instructionList)/sizeof(instructionList[0]) - 1){
    Serial.println("POSITIONED IN FRONT OF MINER.");
    if (goal == minerPosition){

      moveForward();

      long currentCount = totalCount;
      long countDifference = 0;

      while (countDifference <300){
        countDifference = totalCount-currentCount;
        moveForward();
      }

      tapBrake(75,500);

      pickup();

      currentCount = totalCount;
      countDifference = 0;

      moveForward();

      while (countDifference < 2400){
        countDifference = totalCount-currentCount;
        moveForward();
      }

      tapBrake(75,200);

      rotate("left");
      rotate("left");

      flipInstructions();
      instructionCounter = 0;
      goal = startPosition;
      totalCount = 0;

      for (int i=0; i<2; i++){
        Serial.println(instructionList[i]);
      }

    }
    else if (goal == startPosition){

      long currentCount = totalCount;
      long countDifference = 0;

      moveForward();

      while (countDifference < 8000){
        countDifference = totalCount-currentCount;
        moveForward();
      }

      tapBrake(90,300);
      shutdownRobot();
    } 


  }
}

void flipInstructions(){
  int size = sizeof(instructionList)/sizeof(instructionList[0]);
  for (int i = 0; i<size; i++){
    if (instructionList[i] == "left"){
      instructionList[i] = "right";
    }
    else if (instructionList[i] == "right"){
      instructionList[i] = "left";
    }
  }

  String instructionList2[size];

  for (int i = 0;i<size; i++){
    instructionList2[i] = instructionList[i];
  }

  int j = size;
  for (int i = 0; i<size; i++){
    instructionList[i] = instructionList2[size-1-i];
  }
}

void shutdownRobot(){
  brakeTires();
  while (1){ 
    Serial.println("Shutdown complete. Please reset robot.");
    delay(5000);
  }
}



void driveMotor(int newLeftMotor, int newRightMotor){

  if (newLeftMotor <= 0){
    digitalWrite(M2,LOW);
  }
  else{
    digitalWrite(M2,HIGH);
  }

  if (newRightMotor <= 0){
    digitalWrite(M1,LOW);
  }
  else{
    digitalWrite(M1,HIGH);
  }

  leftMotor = newLeftMotor;
  rightMotor = newRightMotor;

  newLeftMotor = abs(newLeftMotor);
  newRightMotor = abs(newRightMotor);

  analogWrite(E2, newLeftMotor); //PWM Speed Control
  analogWrite(E1, newRightMotor); //PWM Speed Control
}

void rotate(char* rotationDir){ //rotate the robot|    rotationDir: 0 turns cc, 1 turns ccw     toDir: up; right;down;left 

  long total = 0;

  gyro.read();
  tapBrake(95,300);
  gyro.read();
  total += gyro.g.x;
  driveMotor(-37,37);

  if (rotationDir == "left"){
    Serial.println("DIAGNOSING||||||||||||||||Starting Left Rotating");
    while (total>-830000){

      gyro.read();
      Serial.println(total);
      total += gyro.g.y;
      delay(10);
    }
  }
  else if (rotationDir == "right"){
    driveMotor(37,-37);
    while (total<830000){
      gyro.read();
      Serial.println(total);
      total += gyro.g.y;
      delay(10);
    }
  }

  Serial.println("DIAGNOSING||||||||||||||||Finish Rotating");
  Serial.print("LeftMotor:");
  Serial.println(leftMotor);
  Serial.print("RightMotor:");
  Serial.println(rightMotor);

  tapBrake(75,300);
}

int getRightEncoderCounter(){
  int a = rightEncoderCounter;
  return a;
}

int getLeftEncoderCounter(){
  int a = leftEncoderCounter;
  return a;
}

void clearLeftEncoderCounter(){
  leftEncoderCounter = 0;
}

void clearRightEncoderCounter(){
  leftEncoderCounter = 0;
}

void clearEncoder(){
  leftEncoderCounter = 0;
  rightEncoderCounter = 0;
}




















