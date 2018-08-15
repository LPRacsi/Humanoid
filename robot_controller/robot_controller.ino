/* This code is used for sending the converted PS2 data through HC 12
 * Author: Rácz László
*/

#include <SoftwareSerial.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <common_for_robot.h>

#define I2C_ADDR           0x27        //Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN      3
#define En_pin             2
#define Rw_pin             1
#define Rs_pin             0
#define D4_pin             4
#define D5_pin             5
#define D6_pin             6
#define D7_pin             7

LiquidCrystal_I2C      lcd(I2C_ADDR, En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

/*Remote controller modes*/
#define SELECT 1
#define NORMAL 0

#define ALIVE_INDEX 20
#define MOD_INDEX   30

#define MODE_MAX_INDEX      5
#define SENT_ARRAY_ELEMENTS 16
#define ALIVE_SEND_TIME     1500
#define CONN_LOST_TIME      3500
/*Pin selections*/
#define rightJoyXPin  A0
#define rightJoyYPin  A1
#define leftJoyXPin   A2
#define leftJoyYPin   A3
#define up            3
#define right         4
#define down          5
#define left          6
#define button4       7
#define button3       8
#define button2       9
#define button1       10
#define rightFront2   2
#define rightFront1   12
#define leftFront2    A6
#define leftFront1    A7
#define mode          11


short modeCounter, dataSendSelect, remoteMode, modeStringIndex, modeStringIndexPrev, value, prevReceivedValue, resendNum;
bool modeRead, upRead, downRead, upReadPrev, downReadPrev, modeChange, hasNewIndex, connectionLost, devTextChange;
char* modeString[MODE_MAX_INDEX] = {"Drive only!",//0 
                                    "R.arm only!",//1 
                                    "L.arm only!",//2 
                                    "R.arm&drive!",//3 
                                    "L.arm&drive!"};//4
                                    
/*char* inputDataIDArray[] = {"rF2", "rF1", "lF2", "lF1",
                            "bu4", "bu3", "bu2", "bu1",
                            "up0", "rig", "lef", "dow",
                            "rJX", "rJY", "lJX", "lJY",};*/
unsigned short inputDataArray[SENT_ARRAY_ELEMENTS], inputDataArray_old[SENT_ARRAY_ELEMENTS],indexArray[SENT_ARRAY_ELEMENTS];
unsigned long lastSendTime, currTime, lastSelectTime, lastModeTime, prevSentAliveTime, lastReceivedAliveTime;
unsigned long cycleTime = 100;
unsigned long selectCycleTime = 250;

char input;
String message, command;
bool commandReceived;

/***********************************************Function definitions***********************************************************/

/*
 * Function:    readFromInput
 * Description: Reading the input from buttons.
 *              Filling the inputDataArray with the data.
*/
void readFromInput(){
  int lf1, lf2;
  lf2 = analogRead(leftFront2);
  lf1 = analogRead(leftFront1);
  inputDataArray[0] = digitalRead(rightFront2); 
  inputDataArray[1] = digitalRead(rightFront1);
  inputDataArray[2] = (lf2 > 512) ? 1: 0; //convert to bool
  inputDataArray[3] = (lf1 > 512) ? 1: 0; //convert to bool
  inputDataArray[4] = digitalRead(button4);
  inputDataArray[5] = digitalRead(button3); 
  inputDataArray[6] = digitalRead(button2);
  inputDataArray[7] = digitalRead(button1); 
  inputDataArray[8] = digitalRead(up);
  inputDataArray[9] = digitalRead(right); 
  inputDataArray[10] = digitalRead(left); 
  inputDataArray[11] = digitalRead(down); 
  inputDataArray[12] = joysticToRange(analogRead(rightJoyXPin)); 
  inputDataArray[13] = joysticToRange(analogRead(rightJoyYPin)); 
  inputDataArray[14] = joysticToRange(analogRead(leftJoyXPin)); 
  inputDataArray[15] = joysticToRange(analogRead(leftJoyYPin));
}

/*
 * Function:    joysticToRange
 * Parameter:   chord - The input joystic chordinate 0-1023
 * Return value:Returning joystic chordinate in ranged value
 * Description: Converting the input choordinates into fixed range values.
*/
unsigned short joysticToRange(unsigned short chord){
  unsigned short retVal = 0;
  if (0 <= chord && chord <= 204){
    retVal = 0;
  }
  if (205 <= chord && chord <= 409){
    retVal = 307;
  }
  if (410 <= chord && chord <= 614){
    retVal = 512;
  }
  if (615 <= chord && chord <= 819){
    retVal = 717;
  }
  if (820 <= chord && chord <= 1023){
    retVal = 1023;
  }
  return retVal;
}
/*
 * Function:    saveOldInput
 * Description: Saving the previusly readed input data to inputDataArray_old.
*/
void saveOldInput(){
  for(int j = 0; j < SENT_ARRAY_ELEMENTS; j++){
    inputDataArray_old[j] = inputDataArray[j];
  }
}

/*
 * Function:    sendData
 * Parameter:   index: The index of data ID and data
 * Description: Sending the ID of the data and the data itself to the receiver. 
*/
void sendData(int index){
  if (index == ALIVE_INDEX){// nagyobb mint a inputDataIDArray maximuma
    Serial.print("X1");
    Serial.print('@');
    Serial.print(" ");
  }else if (index >= MOD_INDEX && index <= MOD_INDEX+4){// kell egy shift a 0-4 be pl 30 és akkor 
    Serial.print("Y");
    Serial.print(index-MOD_INDEX);
    Serial.print('@');
    Serial.print(" ");  
  }else{ // ha nem mod váltás és nem alive küldés
    Serial.print(inputDataIDArray[index]);
    Serial.print(inputDataArray[index]);
    Serial.print('@');
    Serial.print(" ");
  }
  delay(20);
  prevSentAliveTime = millis();
}

/*
 * Function:    compareChanges
 * Description: Comparing the old button states and the new states, and storing the changes in indexArray
*/
void compareChanges(){
  int k = 0;
  for(int j = 0; j < SENT_ARRAY_ELEMENTS; j++){
    if (j > 11){
      if (5 < abs(inputDataArray_old[j] - inputDataArray[j])){
        indexArray[k] = j;
        k++;
        hasNewIndex = true;
      }
    }else
    if (inputDataArray_old[j] != inputDataArray[j]){
      indexArray[k] = j;
      k++;
      hasNewIndex = true;
    }
  }
}

/*
 * Function:    clearIndexArray
 * Description: Clearing the indexes in indexArray
*/
void clearIndexArray(){
  for(int j = 0; j < SENT_ARRAY_ELEMENTS; j++){
    indexArray[j] = 0;
    hasNewIndex = false;
  }
}


/*
 * Function:    handleLCD
 * Parameter:   row: The row to be modified on LCD
 *              column: The start column of the printed string
 *              message: The massage to be printed
 *              clearRow: Clear the row if requested
 * Description: Handle requests to LCD writes.
*/
void handleLCD(int row, int column, String message, bool clearRow){
  if (clearRow){
    lcd.setCursor(0,row);
    lcd.print("                "); //Clearing the selected row
  }
  lcd.setCursor(column,row);
  lcd.print(message);
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
  for (i = 12; i < SENT_ARRAY_ELEMENTS; i++){
    inputDataArray[i] = 512; // Set 512 for joystic 512 = middle
  }
}

/*
 * Function:    receiveSerialInput
 * Description: Checks the received input from serial.
*/
void receiveSerialInput(){
  if(Serial.available() > 0){              
    input = Serial.read();
    lastReceivedAliveTime = millis();
    if (isWhitespace(input)){
      /*Do nothing here*/
    }else{
      if (input == '@'){//The control sign
        command = message.substring(0,1);
        value = message.substring(1).toInt();
        message = "";
        commandReceived = true;
      }else{
        commandReceived = false;
        message += input;
      }
    }
  }  
}
/*
 * 
 * 
 */
void sendModeChange(int modeStringIndex_){
  bool answerReceived = false;
  unsigned long sentTime;
  handleLCD(0, 2, "Sending mode", true);
  handleLCD(1, 0, "change to device", true);
  modeChange = true;
  delay(750);
  sendData(modeStringIndex_ + MOD_INDEX);
  sentTime = millis();
  currTime = millis();
  while(!answerReceived && (currTime - sentTime) < 5000){
    currTime = millis();
    if(Serial.available() > 0){              
      String input = Serial.readString();
      //Debug print
      //Serial.print("Answer ");
      //Serial.print(input);
      if (input == "OK " || input == " OK " || input == " OK"){
        answerReceived = true;
        break;
      }
    }
  }
  if (answerReceived){
    handleLCD(0, 0, "Answer received:", true);
    handleLCD(1, 7, "OK", true);
    connectionLost = false;
    delay(750);
    lastReceivedAliveTime = millis();            
  }else{
     handleLCD(0, 1, "No answer for", true);
     handleLCD(1, 2, "mode change!", true);
     delay(750);
  } 
}
/*****************************************************SETUP*************************************************************/
void setup() {
  Serial.begin(9600);
  lcd.begin (16,2);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  /*Pin init*/  
  pinMode(rightJoyXPin, INPUT);
  pinMode(rightJoyYPin, INPUT);
  pinMode(leftJoyXPin, INPUT);
  pinMode(leftJoyYPin, INPUT);
  pinMode(up, INPUT_PULLUP);
  pinMode(right, INPUT_PULLUP);
  pinMode(down, INPUT_PULLUP);
  pinMode(left, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button1, INPUT_PULLUP);
  pinMode(rightFront2, INPUT_PULLUP);
  pinMode(rightFront1, INPUT_PULLUP);
  pinMode(leftFront2, INPUT);
  pinMode(leftFront1, INPUT);
  pinMode(mode, INPUT_PULLUP);
  /*Variable init*/
  modeCounter = 0;
  remoteMode = NORMAL; 
  modeRead = HIGH; //Not pressed
  modeStringIndex = 0;
  modeStringIndexPrev = MODE_MAX_INDEX;
  modeChange = true;
  connectionLost = true;
  lastSendTime = 0;
  lastSelectTime = 0;
  lastModeTime = 0;
  commandReceived = false;
  message = "";
  input = "";
  command = "";
  value = 0;
  prevReceivedValue = 100;
  devTextChange = true;
  
  handleLCD(0, 3, "CONTROLLER", true);
  handleLCD(1, 5, "READY!", true);
  initInputDataArray();
  /*readFromInput();
  for(int j = 0; j < SENT_ARRAY_ELEMENTS; j++){
    sendData(j);
    delay(100);
  }*/
  delay(700);
  clearIndexArray();
}
/*****************************************************LOOP*************************************************************/
void loop() {
  switch(remoteMode)
  {
    case NORMAL:
    modeStringIndexPrev = MODE_MAX_INDEX;
      if (modeChange){
        modeChange = false;
        handleLCD(0, 0, "CON:", true);
        handleLCD(0, 4, modeString[modeStringIndex], false);
      }
      saveOldInput();
      readFromInput();
      compareChanges();
      if (hasNewIndex){ // If there is a new index, the first element of indexArray should be sent at least.
        resendNum = 2;
        commandReceived = false;
        while (!commandReceived && resendNum > 0){
          sendData(indexArray[0]);
          receiveSerialInput();
          resendNum--;  
        }
     
        for(int j = 1; j < SENT_ARRAY_ELEMENTS; j++){          
          if (indexArray[j] != 0){
            resendNum = 2;
            commandReceived = false;
            while (!commandReceived && resendNum > 0){ 
              sendData(indexArray[j]);
              receiveSerialInput();
              resendNum--;              
            }
          }
        }
      //}
        clearIndexArray();
      }
      currTime = millis();
      if (currTime - prevSentAliveTime > ALIVE_SEND_TIME){
        prevSentAliveTime = currTime;
        connectionLost = false;
        devTextChange = true;
        sendData(ALIVE_INDEX); //ide kellene a waitforAnswer is
      }      
      currTime = millis();
      if (currTime - lastReceivedAliveTime > CONN_LOST_TIME && !connectionLost){
        handleLCD(1, 0, "Connection lost!", true);
        connectionLost = true;
        devTextChange = false;
        delay(750);
      }      
    break;
    /*********************************************Controller selector mode*****************************************************/
    case SELECT:
      /*currTime = millis();
      if (currTime - prevSentAliveTime > ALIVE_SEND_TIME){
        prevSentAliveTime = currTime;
        connectionLost = false;
        Serial.print("ALI1");
        Serial.print('@');
        Serial.print(" ");
      }*/   
      if (currTime - lastSelectTime > selectCycleTime){
        lastSelectTime = currTime;
        upRead = digitalRead(down);
        downRead = digitalRead(up);  
        if (upRead == LOW && upReadPrev == HIGH){
          modeStringIndex++;
          upReadPrev = upRead;
        } else if (upRead == HIGH){
          upReadPrev = upRead;
        }
        if (downRead == LOW && downReadPrev == HIGH){
          modeStringIndex--;
          downReadPrev = downRead;
        } else if (downRead == HIGH){
          downReadPrev = downRead;
        }
        if (modeStringIndex > MODE_MAX_INDEX-1){
          modeStringIndex = 0;
        }
        if (modeStringIndex < 0){
          modeStringIndex = MODE_MAX_INDEX-1;
        }
        if (modeStringIndexPrev != modeStringIndex){          
          handleLCD(0, 1, "Selector mode!", true);
          handleLCD(1, 2, modeString[modeStringIndex], true);
          modeStringIndexPrev = modeStringIndex;
        }
      }
    break;
  }
  /************************************************Check mode selector press**********************************************/
  currTime = millis();
  if (currTime - lastModeTime > selectCycleTime){
    lastModeTime = currTime;
    modeRead = digitalRead(mode); 
    if (modeRead == LOW){
      modeCounter++;
      if (modeCounter >= 5){
        modeCounter = 0;
        remoteMode = (remoteMode == NORMAL) ? SELECT: NORMAL;        
        /************************************************************************************/
        if (remoteMode == NORMAL){
          sendModeChange(modeStringIndex);
        }  
      }
    }
  }
  /***************************************Check input from receiver*************************************************/
  receiveSerialInput();
    /*Serial.print("command :");
    Serial.println(command);*/
    if (commandReceived && command == "Z"){
      commandReceived = false; //Command already processed
      //debug print
      /*Serial.print("command: ");
      Serial.println(command);
      Serial.print("value: ");
      Serial.println(value);*/
      /*Serial.print("value: ");
      Serial.println(value);
      Serial.print("modeStringIndex: ");
      Serial.println(modeStringIndex);*/      
      if (value == modeStringIndex){
        lastReceivedAliveTime = millis();
        if(prevReceivedValue != value || devTextChange){
          handleLCD(1, 0, "DEV:", true);
          handleLCD(1, 4, modeString[modeStringIndex], false);
          devTextChange = false;
          prevReceivedValue = value;
        }
 
      }else{
        sendModeChange(modeStringIndex);
      }
    }/*else if (commandReceived){
      handleLCD(0, 0, "MSG from device:", true);
      handleLCD(1, 0, command, true);
      delay(750);
    }*/
  //}  
}
