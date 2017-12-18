/* This code is used for receiving data via HC 12 and control a tank chassys arm servos
 * Author: Rácz László
*/
#include <SoftwareSerial.h>
#include <Servo.h>
/*PIN definition*/
#define R_SER_RX_PIN          2
#define R_SER_TX_PIN          3
#define R_SHOULDER_SERVO_PIN  4
#define R_SHOULDER2_SERVO_PIN 5
#define R_ELBOW_SERVO_PIN     6
#define R_WRIST_SERVO_PIN     7
#define R_WRIST2_SERVO_PIN    8
#define R_HAND_SERVO_PIN      9
/**/
#define RECEIVED_ARRAY_ELEMENTS 16
#define RF2 0
#define RF1 1
#define LF2 2
#define LF1 3
#define BU4 4
#define BU3 5
#define BU2 6
#define BU1 7
#define UP0 8
#define RIG 9
#define LEF 10
#define DOW 11
#define RJX 12
#define RJY 13
#define LJX 14
#define LJY 15

/****** controll modes  **************/
#define DRIVE_ONLY      0
#define R_ARM_ONLY      1
#define L_ARM_ONLY      2
#define R_ARM_AND_DRIVE 3
#define L_ARM_AND_DRIVE 4
#define DEFAULT_MODE    100

/******Timing defines*************/
#define CONN_LOST_TIME    3000  //3 s
#define ARM_UPDATE_TIME   100

/**/
#define SERVO_MAX 175
#define SERVO_MIN 0

SoftwareSerial receiverSerial(R_SER_RX_PIN, R_SER_TX_PIN);
Servo rShoulder, rShoulder2, rElbow, rWrist, rWrist2, rHand;

char incomingByte;
String message, command;
int value, controllMode, controllModePrev, rShoulderPos, rShoulderPos2, rElbowPos, rWristPos, rWristPos2, rHandPos;
bool commandReceived, lostConnection;
String inputDataIDArray[] = {"rF2", "rF1", "lF2", "lF1",
                            "bu4", "bu3", "bu2", "bu1",
                            "up0", "rig", "lef", "dow",
                            "rJX", "rJY", "lJX", "lJY"};
unsigned short inputDataArray[RECEIVED_ARRAY_ELEMENTS];
unsigned long currTime, lastAliveTime, lastArmUpdateTime;

/***********************************************Function definitions***********************************************************/
/*
 * Function:    fillDataArrayWithInput
 * Parameter:   index: Index of input array
 *              value: Input array data to be stored
 * Description: Fill the input array with the received data
*/
void fillDataArrayWithInput(String index, int value){
  for (int i = 0; i < RECEIVED_ARRAY_ELEMENTS; i++){
    if (inputDataIDArray[i] == index){
      Serial.print("Update ");
      Serial.println(index);
      inputDataArray[i] = value;
    }
  }
}

/*
 * Function:    initInputDataArray
 * Description: Set init values as not pressed / not moved joystic
*/
void initInputDataArray(){
  int i;
  for (i = 0; i < 12; i++){
    inputDataArray[i] = 1; // Set 0 to the push buttons
  }
  for (i = 12; i < RECEIVED_ARRAY_ELEMENTS; i++){
    inputDataArray[i] = 512; // Set 512 for joystic 512 = middle
  }
}

/*
 * Function:    calcServoPosAI
 * Description: Calculates the new position of the corresponding servo 
 * Parameter:   inputDataArrayIndex: Index of input array
 *              pos:                 Actual position of he servo
 * Return:      New servo position
*/
int calcServoPosAI(int inputDataArrayIndex, int pos){
  int posMod;
  if (inputDataArray[inputDataArrayIndex] > 100 && inputDataArray[inputDataArrayIndex] < 923){
    posMod = 0;
  }
  if (inputDataArray[inputDataArrayIndex] >= 923){
    posMod = -1;
  }
  if (inputDataArray[inputDataArrayIndex] <= 100){
    posMod = 1;
  }    
  pos += (posMod * 2);
  if (pos >= SERVO_MAX){
    pos = SERVO_MAX; 
  }
  if (pos <= SERVO_MIN){
    pos = SERVO_MIN;
  }
  return pos;
}

/*
 * Function:    calcServoPosAI
 * Description: Calculates the new position of the corresponding servo 
 * Parameter:   inputDataArrayIndex: Index of input array
 *              pos:                 Actual position of he servo
 * Return:      New servo position
*/
int calcServoPosDI(int inputDataArrayIndex, int inputDataArrayIndex2, int pos){
  int posMod;
  if (inputDataArray[inputDataArrayIndex] == 1 && inputDataArray[inputDataArrayIndex2] == 1){
    posMod = 0;
  }
  if (inputDataArray[inputDataArrayIndex] == 0){
    posMod = -1;
  }
  if (inputDataArray[inputDataArrayIndex2] == 0){
    posMod = 1;
  }    
  pos += (posMod * 2);
  if (pos >= SERVO_MAX){
    pos = SERVO_MAX; 
  }
  if (pos <= SERVO_MIN){
    pos = SERVO_MIN;
  }
  return pos;
}

void setup() {
  Serial.begin(9600);
  //receiverSerial.begin(9600); 
  initInputDataArray();
  rShoulderPos = 30;
  rShoulderPos2 = 150;
  rElbowPos = 45;
  rWristPos = SERVO_MIN;
  rWristPos2 = 90;
  rHandPos = SERVO_MAX;
  controllMode = 0;
  controllModePrev = 0;
  commandReceived = false;
  message = "";
  incomingByte = "";
  command = "";
  lostConnection = false;
  currTime = millis();
  lastAliveTime = millis();
  lastArmUpdateTime = millis();
  rShoulder.attach(R_SHOULDER_SERVO_PIN);
  rShoulder2.attach(R_SHOULDER2_SERVO_PIN);
  rElbow.attach(R_ELBOW_SERVO_PIN);
  rWrist.attach(R_WRIST_SERVO_PIN);
  rWrist2.attach(R_WRIST2_SERVO_PIN);
  rHand.attach(R_HAND_SERVO_PIN);
  rShoulder.write(rShoulderPos);
  rShoulder2.write(rShoulderPos2);
  rElbow.write(rElbowPos);
  rWrist.write(rWristPos);
  rWrist2.write(rWristPos2);
  rHand.write(rHandPos);
}

void loop() {
  if(Serial.available() > 0){
    incomingByte = Serial.read();
    //Serial.print("incomingByte :");
    //Serial.println(incomingByte);
    if (isWhitespace(incomingByte)){
      /*Do nothing here*/
    }else{
      //Serial.println(incomingByte);
      if (incomingByte == '@'){//The control sign
        command = message.substring(0,3);
        value = message.substring(3).toInt();
        message = "";
        commandReceived = true;
      }else{
        commandReceived = false;
        message += incomingByte;
      }
    }
  }
  if (commandReceived){
    lastAliveTime = millis();
    if (lostConnection){
      controllMode = controllModePrev;
      lostConnection = false;
    }
    if (command == "Mod"){
      controllMode = value;
      Serial.print("controllMode :");
      Serial.println(controllMode); 
      //receiverSerial.print("OK");
      //receiverSerial.print(" ");
    }
    else if (command == "ALI"){
      /*Do nothing here*/
    }
    else {
      /*Serial.println("Aatfeltoltes--------------------------------------------");
      Serial.print("command ");
      Serial.println(command);
      Serial.print("value ");
      Serial.println(value);*/
      fillDataArrayWithInput(command, value);
    }
    commandReceived = false;
  }

  currTime = millis();
  
  if (currTime - lastAliveTime > CONN_LOST_TIME){
    controllModePrev = controllMode;
    controllMode = DEFAULT_MODE;
    lostConnection = true;
    delay(50);
  }
  //Serial.print("controllMode :");
  //Serial.println(controllMode);    
  switch(controllMode){
    case DRIVE_ONLY:
      /*Do nothing here*/
    break;
    case R_ARM_ONLY:
    case R_ARM_AND_DRIVE:
      if (currTime - lastArmUpdateTime > ARM_UPDATE_TIME){
        rShoulderPos = calcServoPosAI(RJX, rShoulderPos);
        rShoulder.write(rShoulderPos);
        rShoulderPos2 = calcServoPosAI(RJY, rShoulderPos2);
        rShoulder2.write(rShoulderPos2);
        rElbowPos = calcServoPosDI(LF1, LF2, rElbowPos);
        rElbow.write(rElbowPos);
        rWristPos = calcServoPosAI(LJX, rWristPos);
        rWrist.write(rWristPos);
        rWristPos2 = calcServoPosAI(LJY, rWristPos2);
        rWrist2.write(rWristPos2);
        rHandPos = calcServoPosDI(RF1, RF2, rHandPos);
        rHand.write(rHandPos);
        lastArmUpdateTime = millis();
      }  
    break;
    default:
      /*Do nothing here*/
    break;
    
  }
}
