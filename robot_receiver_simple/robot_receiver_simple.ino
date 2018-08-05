/* This code is used for receiving data via HC 12 and control a tank chassys, reansmitt Data to receiver_extension,
 * that is controlling the arm servos. This unit controlls the chain.
 * Author: Rácz László
*/

#include <SoftwareSerial.h>
#include <common_for_robot.h>
/*PIN definition*/
#define RIGHT_CHAIN_PWM   6
#define LEFT_CHAIN_PWM    5
#define RIGHT_CHAIN_DIR1  9
#define RIGHT_CHAIN_DIR2  10
#define LEFT_CHAIN_DIR1   7
#define LEFT_CHAIN_DIR2   8
#define VOLT_MEAS_PIN     A0
#define RED_LED_PIN       A4
#define YELLOW_LED_PIN    A3
#define GREEN_LED1_PIN    A2
#define GREEN_LED2_PIN    A1
#define R_SER_RX_PIN      3
#define R_SER_TX_PIN      2

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

/**/
#define CONN_LOST_TIME    3000  //3 s
#define BATT_REFRESH_TIME 60000 //1 min

/****** controll modes  **************/
#define DRIVE_ONLY      0
#define R_ARM_ONLY      1
#define L_ARM_ONLY      2
#define R_ARM_AND_DRIVE 3
#define L_ARM_AND_DRIVE 4
#define DEFAULT_MODE    100

SoftwareSerial receiverSerial(R_SER_RX_PIN, R_SER_TX_PIN);

char incomingByte;
//char* command;
String message, command;
bool commandReceived, lostConnection;
int value, controllMode, controllModePrev;
/*String inputDataIDArray[] = {"rF2", "rF1", "lF2", "lF1",
                            "bu4", "bu3", "bu2", "bu1",
                            "up0", "rig", "lef", "dow",
                            "rJX", "rJY", "lJX", "lJY"};*/
unsigned short inputDataArray[RECEIVED_ARRAY_ELEMENTS];
unsigned long currTime, lastAliveTime, lastBattUpdateTime;

/***********************************************Function definitions***********************************************************/
/*
 * Function:    ControlChains
 * Parameter:   chainLeft: PWM and direction for left motor
 *              chainRight: PWM and direction for right motor
 * Description: Handlethe right and left chain motors
*/
void ControlChains(int chainLeft, int chainRight){
  bool directionL, directionR;      // direction of motor rotation L298N

  if(chainLeft > 0){
    directionL = LOW;
  }
  else if(chainLeft < 0){
    chainLeft = abs(chainLeft);
    directionL = HIGH;
  }
  if(abs(chainLeft) < 150){
    chainLeft = 0;
  }
 
  if(chainRight > 0){
    directionR = LOW;
  }
  else if(chainRight < 0){
    chainRight = abs(chainRight);
    directionR = HIGH;
  }
  if(abs(chainRight) < 150){
    chainRight = 0;
  }
  /*Serial.print("chainRight ");
  Serial.print(chainRight); 
  Serial.print("chainLeft ");
  Serial.print(chainLeft); */
  analogWrite(RIGHT_CHAIN_PWM, (chainRight));   // set speed for left motor
  analogWrite(LEFT_CHAIN_PWM, (chainLeft));     // set speed for right motor
  digitalWrite(RIGHT_CHAIN_DIR1, directionR);   // set direction of left motor rotation
  digitalWrite(RIGHT_CHAIN_DIR2, !directionR);  // set direction2 of left motor rotation
  digitalWrite(LEFT_CHAIN_DIR1, directionL);    // set direction of right motor rotation
  digitalWrite(LEFT_CHAIN_DIR2, !directionL);   // set direction2 of right motor rotation    
}

/*
 * Function:    fillDataArrayWithInput
 * Parameter:   index: Index of input array
 *              value: Input array data to be stored
 * Description: Fill the input array with the received data
*/
void fillDataArrayWithInput(String index, int value){
  for (int i = 0; i < RECEIVED_ARRAY_ELEMENTS; i++){
    if (inputDataIDArray[i] == index){
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
 * Function:    getBatteryVoltage
 * Description: Reads the measured voltage and calculates the battery voltage
 * Return:      The input battery voltage
*/
float getBatteryVoltage(){
  float vIn = 0.0; 
  float vOut = 0.0; 
  float R1 = 536000.0;
  float R2 = 268000.0;
  int readBatt = 0;
  readBatt = analogRead(VOLT_MEAS_PIN);
  //Serial.print("Measured readBatt: ");
  //Serial.println(readBatt);  
  vOut = (readBatt * 5.0) / 1024.0; 
  vIn = vOut / ( R2 / (R1 + R2)); 
  if (vIn < 0.09) {
    vIn = 0.0;//statement to quash undesired reading !
  }
  return vIn;
}

/*
 * Function:    checkAndSetBatteryStatus
 * Description: Sets the battery status leds and the inner state based on measured battery voltage
*/
void checkAndSetBatteryStatus(){
  float batteryVoltage = getBatteryVoltage();
  if (batteryVoltage > 11.60){
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(YELLOW_LED_PIN, HIGH);
    digitalWrite(GREEN_LED1_PIN, HIGH);
    digitalWrite(GREEN_LED2_PIN, HIGH);    
  }else if (batteryVoltage > 10.80){
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(YELLOW_LED_PIN, HIGH);
    digitalWrite(GREEN_LED1_PIN, HIGH);
    digitalWrite(GREEN_LED2_PIN, LOW);    
  }else if (batteryVoltage > 10.00){
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(YELLOW_LED_PIN, HIGH);
    digitalWrite(GREEN_LED1_PIN, LOW);
    digitalWrite(GREEN_LED2_PIN, LOW);    
  }else if (batteryVoltage <= 10.00){
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GREEN_LED1_PIN, LOW);
    digitalWrite(GREEN_LED2_PIN, LOW);
    controllMode = DEFAULT_MODE;
  }
  lastBattUpdateTime = millis();
}
/*****************************************************SETUP*************************************************************/
void setup() {
  Serial.begin(9600);
  receiverSerial.begin(9600);
  //transmitterSerial.begin(19200);
  pinMode(RIGHT_CHAIN_PWM, OUTPUT);
  pinMode(LEFT_CHAIN_PWM, OUTPUT);
  pinMode(RIGHT_CHAIN_DIR1, OUTPUT);
  pinMode(RIGHT_CHAIN_DIR2, OUTPUT);
  pinMode(LEFT_CHAIN_DIR1, OUTPUT);
  pinMode(LEFT_CHAIN_DIR1, OUTPUT);
  pinMode(VOLT_MEAS_PIN, INPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED1_PIN, OUTPUT);
  pinMode(GREEN_LED2_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(YELLOW_LED_PIN, HIGH);
  digitalWrite(GREEN_LED1_PIN, HIGH);
  digitalWrite(GREEN_LED2_PIN, HIGH);  
  initInputDataArray();
  controllMode = 0;
  controllModePrev = 0;
  commandReceived = false;
  message = "";
  incomingByte = "";
  command = "";
  lostConnection = false;
  currTime = millis();
  lastAliveTime = millis();
  delay(500);
  checkAndSetBatteryStatus();
}

/*****************************************************LOOP*************************************************************/
void loop() {
  /*if(Serial.available() > 0){
    String input = Serial.readString();
    receiverSerial.println(input);    
  }*/
    
  if(receiverSerial.available() > 1){
    incomingByte = receiverSerial.read();
    //transmitterSerial.write(incomingByte);
    Serial.write(incomingByte);
    if (isWhitespace(incomingByte)){
      /*Do nothing here*/
    }else{
      //Serial.println(incomingByte);
      if (incomingByte == '@'){//The control sign
        command = message.substring(0,1);
        value = message.substring(1).toInt();
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
    /*Serial.print("command :");
    Serial.println(command);
    Serial.print("value :");
    Serial.println(value);*/
    if (command == "Y"){
      Serial.print("MOD");
      controllMode = value;
      receiverSerial.print("OK");
      receiverSerial.print(" ");
    }
    else if (command == "X"){
      /*int rightC =  map(inputDataArray[RJX], 0, 1023, -255, 255);
      int leftC =  map(inputDataArray[LJX], 0, 1023, -255, 255);
      Serial.print("Alive Message");*/
      receiverSerial.print("Z");
      receiverSerial.print(controllMode);
      receiverSerial.print("@");
      receiverSerial.print(" ");
      /*Serial.print("right Chain:");
      Serial.print(rightC);
      Serial.print("left Chain:");
      Serial.print(leftC);*/
    }
    else {
      fillDataArrayWithInput(command, value);
      /*for (int i =0; i < RECEIVED_ARRAY_ELEMENTS; i++){
        if (inputDataIDArray[i] == command){
          Serial.print(inputDataIDArray[i]);
          Serial.print(" :");
          Serial.println(inputDataArray[i]);
        }
      }*/
    }
    commandReceived = false;
  }

  /*Serial.print("Measured battery: ");
  Serial.println(getBatteryVoltage());*/
  currTime = millis();
  if (currTime - lastBattUpdateTime > BATT_REFRESH_TIME){
    checkAndSetBatteryStatus();
  }
  
  if (currTime - lastAliveTime > CONN_LOST_TIME){
    controllModePrev = controllMode;
    controllMode = DEFAULT_MODE;
    ControlChains(0, 0);
    lostConnection = true;
    delay(50);
  }
    
  switch(controllMode){
    int rightC, leftC;
    case DRIVE_ONLY:
      if (inputDataArray[RJX] > 462 && inputDataArray[RJX] < 562){
        rightC = 0;
      }else{
        rightC =  map(inputDataArray[RJX], 1023, 0, -255, 255);
      }
      if (inputDataArray[LJX] > 462 && inputDataArray[LJX] < 562){
        leftC = 0;
      }else{      
        leftC =  map(inputDataArray[LJX], 1023, 0, -255, 255);
      }
      /*Serial.print("right Chain:");
      Serial.print(rightC);
      Serial.print("left Chain:");
      Serial.print(leftC);*/
      ControlChains(leftC, rightC);
    break;
    case R_ARM_AND_DRIVE:
      if (inputDataArray[UP0] == 0){
        rightC = leftC = -255;
      }else if (inputDataArray[DOW] == 0){
        rightC = leftC = 255;
      }else if (inputDataArray[LEF] == 0){
        rightC = 215;
        leftC = -215;
      }else if (inputDataArray[RIG] == 0){
        rightC = -215;
        leftC = 215;
      }else{
        rightC = leftC = 0;
      }
      ControlChains(leftC, rightC);
    break;
    case R_ARM_ONLY:
    case L_ARM_ONLY:
    default:
      /*Do nothing here*/
    break;

  }
}
