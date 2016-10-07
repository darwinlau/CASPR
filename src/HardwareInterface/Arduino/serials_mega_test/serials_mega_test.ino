/**
   MEGA

   send via TX1
   receive via digital pins

   Not all pins on the Mega and Mega 2560 support change interrupts,
   so only the following can be used for RX:
   10, 11, 12, 13, 14, 15, 50, 51, 52, 53,
   A8 (62), A9 (63), A10 (64), A11 (65),
   A12 (66), A13 (67), A14 (68), A15 (69).
*/

#include <SoftwareSerial.h>

// Defining constants
#define FEEDBACK_FREQUENCY 20// In Hz
#define TIME_STEP 1.0/FEEDBACK_FREQUENCY
#define NUMBER_CONNECTED_NANOS 8
#define BAUD_RATE 74880

#define HEX_DIGITS_LENGTH 4
#define DIGITS_PWM_COMMAND 4
#define DIGITS_PWM_FEEDBACK 3

#define ASCII_MIDDLE_POINT 75 //breakpoint between cw(0-9, A-F) and ccw (P-Y, a-f)
#define ASCII_DIFFERENCE 32 //difference for conversion between cw and ccw

#define RECEIVE_ANGLE 'a'
#define REQUEST_FEEDBACK 'f'
#define REQUEST_TEST 't'
#define COMMAND_LENGTH 'l'

#define SEND_PREFIX_FEEDBACK 'f'
#define SEND_PWM_COMMAND 'p'
#define RECEIVE_PREFIX_START 's'
#define RECEIVE_PREFIX_END 'e'
#define RECEIVE_PREFIX_INITIAL 'i'
#define RECEIVE_PREFIX_LENGTH_CMD 'l'
#define COMM_PREFIX_ACKNOWLEDGE 'a'

#define RADIUS 200 //spool, 20mm in 0.1mm precision

int maximumPWMFeedback[8] = {1501, 1494, 1501, 1493, 1501, 1499, 1501, 1520};
int minimumPWMFeedback[8] = {484, 483, 484, 482, 484, 484, 484, 491};
int middlePWMFeedback[8] = {992, 988, 992, 987, 992, 991, 992, 1005}; // all numbers rounded down
int maximumPWMOutput[8] = {1488, 1485, 1489, 1481, 1488, 1490, 1490, 1509};
int minimumPWMOutput[8] = {469, 469, 471, 473, 469, 471, 474, 481}; //3, 6, 7 increased by 5
int clockwise_max[8] = {2194, 2175, 2185, 2175, 2189, 2188, 2188, 2215};
int clockwise_min[8] = {2094, 2082, 2090, 2079, 2089, 2088, 2088, 2117};
int clockwise_max_speed[8] = {283, 278, 272, 269, 272, 281, 278, 278};
int clockwise_min_speed[8] = {130, 131, 127, 127, 127, 128, 133, 130};
int anticlockwise_max[8] = {1800, 1780, 1785, 1780, 1785, 1786, 1788, 1811};
int anticlockwise_min[8] = {1891, 1880, 1887, 1876, 1885, 1886, 1888, 1910};
int anticlockwise_max_speed[8] = { -281, -278, -273, -269, -270, -279, -273, -279};
int anticlockwise_min_speed[8] = { -133, -130, -132, -129, -124, -129, -128, -131};


unsigned long int t_ref;
String receivedCommand;
String receivedFeedback;
int pwmCommand[NUMBER_CONNECTED_NANOS];
int pwmFeedback[NUMBER_CONNECTED_NANOS];
int pwmLastFeedback[NUMBER_CONNECTED_NANOS];
int pwmFeedbackDiff = 0;
int rangePWMOutput[NUMBER_CONNECTED_NANOS];
int rangePWMFeedback[NUMBER_CONNECTED_NANOS];
unsigned int lastLengthCommand[NUMBER_CONNECTED_NANOS]; //unsigned int has 2 bytes, range 0 - 65535
unsigned int lastLengthFeedback[NUMBER_CONNECTED_NANOS]; //with .1 mm precision, this equals ~6.5m
unsigned int initLength[NUMBER_CONNECTED_NANOS];
double stepPWMFeedback[NUMBER_CONNECTED_NANOS];
double stepPWMOutput[NUMBER_CONNECTED_NANOS];

boolean crossing[NUMBER_CONNECTED_NANOS];

char feedbackNano[DIGITS_PWM_FEEDBACK]; // Array to store the nano feedback
boolean positive = 1; // flag to indicate positive angle change
int angularChangeReceived; // change in angular value that was recieved
unsigned int lengthFeedback; // length value for feedback
char feedbackMega[HEX_DIGITS_LENGTH]; //4 digit hex length from a nano, sent to mega
int lengthChangeCommand;
int angleChange;
char tmpRead[HEX_DIGITS_LENGTH];

char commandNano[DIGITS_PWM_COMMAND - 1];

float lengthToAngle = 720.0 / (M_PI * RADIUS); //1.14591562747955322265625
float angleToLength = (M_PI * RADIUS) / 45.0; // 13.9626340866088867


unsigned int tmpSendLength;
int angularChangeCommand;

boolean systemOn, enableMotors;

int counter = 0;

int radCounter = 0;
float rad = 0;
int sinPWM = 0;
int sinDeg = 0;
int lastSinDeg = 0;
int sinAngleChange = 0;
char sinAngleCmd[2];
boolean sinPositive = 0;
int sendPWM = 0;

int strLength = 0;


SoftwareSerial serialNano[8] = {
  SoftwareSerial (62, 19), // RX, TX - 0
  SoftwareSerial (63, 23), //1 THERE IS AN ISSUE WITH THIS PIN
  SoftwareSerial (64, 24), //2
  SoftwareSerial (65, 25), //3
  SoftwareSerial (66, 26), //4
  SoftwareSerial (67, 27), //5
  SoftwareSerial (68, 28), //6
  SoftwareSerial (69, 29)  //7
};

/* Setup 3 different serial lines.
   Serial for MATLAB
   Serial1. for transmission to Nano devices
   SoftwareSerial for receiving from individual Nanos
*/
void setup() {
  Serial.begin(BAUD_RATE);  //USB
  Serial1.begin(BAUD_RATE); //broadcast
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) { //all the softwareSerials for arduino nano
    serialNano[i].begin(BAUD_RATE);
    initLength[i] = 32768; //middle
    lastLengthFeedback[i] = 32768;
    lastLengthCommand[i] = 32768;
    stepPWMFeedback[i] = 1440.0 / (double)(maximumPWMFeedback[i] - minimumPWMFeedback[i]); //INVERTED FOR SPECIFIC USE IN THIS CODE
    rangePWMOutput[i] = maximumPWMFeedback[i] - minimumPWMFeedback[i];
    stepPWMOutput[i] = (double)(maximumPWMOutput[i] - minimumPWMOutput[i]) / 1440.0; //360 degree in quarter degree precision -> 1440 steps
    rangePWMOutput[i] = maximumPWMOutput[i] - minimumPWMOutput[i];

    /////// TEMPORARY - REVISE LATER AFTER CALIBRATION //////////
    pwmCommand[i] = middlePWMFeedback[i] - 13;


  }
  t_ref = millis();
  //receivedCommand = INITIAL_LENGTH_COMMAND;
}

/* Main loop acts to interface with MATLAB (asynchronously) and nano at 20Hz */
void loop() {
  readSerialUSB();
  if ((millis() - t_ref) > TIME_STEP * 1000) { // Operate at roughly 20Hz time
    Serial.print(millis() - t_ref);
    Serial.print(" total  nanoFB: ");
    t_ref = millis(); // Reset the time (AT A LATER DATE PROTECTION MAY BE NEEDED FOR OVERFLOW
    if (systemOn) {
      requestNanoFeedback();
      sendNanoFeedback();

      Serial.print(" ");
      Serial.print(millis() - t_ref);
      Serial.print(" ");

      radCounter++;
      rad = radCounter * 0.1;
      tmpSendLength = sin(rad) * 1500;
      tmpSendLength += 32768;

      readNanoCommand();

      Serial.print(" after read: ");
      Serial.print(millis() - t_ref);
      Serial.print(" nanoCommand: ");

      if (enableMotors) {
        sendNanoCommand(); // Set up to send command for the nano
      }
    }
    Serial.print(" ");
    Serial.println(millis() - t_ref);
    Serial.flush();
  }
}

void readSerialUSB() {
  if (Serial.available() > 0) {  //MATLAB via USB
    receivedCommand = Serial.readStringUntil('\n');
    if (receivedCommand[0] == COMM_PREFIX_ACKNOWLEDGE && receivedCommand.length() == 1) //a
    {
      systemOn = 0;
      Serial.println(COMM_PREFIX_ACKNOWLEDGE);
    }

    else if (receivedCommand[0] == RECEIVE_PREFIX_START && receivedCommand.length() == 1) //s
    {
      systemOn = 1;
    }
    else if (receivedCommand[0] == RECEIVE_PREFIX_END && receivedCommand.length() == 1) //e
    {
      systemOn = 0;
      enableMotors = 1;
      Serial1.println(" p10001000100010001000100010001000");
    }
    else if (receivedCommand[0] == RECEIVE_PREFIX_INITIAL) //i
    {
      setInitialLengths();
      enableMotors = 1;
    }
    else if (receivedCommand[0] == RECEIVE_PREFIX_LENGTH_CMD) //l
    {
      enableMotors = 1;
    }
  }
}

void setInitialLengths() {
  char tmp[4];
  unsigned long int newInitLength;
  for (int j = 0; j < NUMBER_CONNECTED_NANOS; j++) {
    for (int k = 0; k < HEX_DIGITS_LENGTH; k++) {
      tmp[k] = receivedCommand[j * HEX_DIGITS_LENGTH + k + 1];
    }
    newInitLength = strtol(tmp, 0, 16);
    lastLengthFeedback[j] += (newInitLength - initLength[j]);
    lastLengthCommand[j] += (newInitLength - initLength[j]);
    initLength[j] = newInitLength;
  }
}

void requestNanoFeedback() {
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    serialNano[i].listen();
    Serial1.println('f'); //requests feedback from nano
    Serial1.flush(); //waits for the sending of Serial to be complete before moving on

    counter = 0;
    while ((serialNano[i].available() == 0) && counter < 500) {
      counter++;
    }
    if (serialNano[i].available() > 0) {
      //receivedFeedback = Serial.readStringUntil('\n');
      for (int j = 0; j < DIGITS_PWM_FEEDBACK; j++) {
        feedbackNano[j] = serialNano[i].read();
      }
      feedbackNano[DIGITS_PWM_FEEDBACK] = '\0';

      while (serialNano[i].available() > 0) {
        serialNano[i].read(); //clears the buffer of any other bytes
      }
      pwmFeedback[i] = strtol(feedbackNano, 0, 16);
      if (pwmFeedback[i] > pwmLastFeedback[i]) { //possible crossing CCW (right -> left)
        if ((-rangePWMOutput[i] - pwmLastFeedback[i] + pwmFeedback[i]) > (pwmLastFeedback[i] - pwmFeedback[i])) {
          pwmFeedbackDiff = pwmFeedback[i] - rangePWMOutput[i] - pwmLastFeedback[i];
          crossing[i] = true;
        }
      }
      else if ((rangePWMOutput[i] - pwmLastFeedback[i] + pwmFeedback[i]) < (pwmLastFeedback[i] - pwmFeedback[i])) { //crossing CW (left -> right)
        pwmFeedbackDiff = rangePWMOutput[i] - pwmLastFeedback[i] + pwmFeedback[i];
        crossing[i] = true;
      } else {
        pwmFeedbackDiff = pwmFeedback[i] - pwmLastFeedback[i];
        crossing[i] = false;
      }
      pwmLastFeedback[i] = pwmFeedback[i];

      lastLengthFeedback[i] += (int)(((pwmFeedbackDiff * stepPWMFeedback[i]) * angleToLength) + 0.5); //conversion of pwmDiff to angleChange to lengthChange
    }
  
  Serial.print(" ");
  Serial.print(millis() - t_ref);
  Serial.print(" ");
  }
}

void sendNanoFeedback() {
  Serial.print(SEND_PREFIX_FEEDBACK);
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    itoa(lastLengthFeedback[i], feedbackMega, 16);
    strLength = strlen(feedbackMega);
    for (int j = 0; j < (DIGITS_PWM_FEEDBACK - strLength); j++) {
      Serial.print('0');
    }
    for (int j = 0; j < strLength; j++) { //fills sendFeedback array at right position, no conversion necessary
      Serial.print(feedbackMega[j]);
    }
  }
}


void readNanoCommand() {
  //if (receivedCommand[0] == COMMAND_LENGTH) {
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    /*for (int j = 0; j < HEX_DIGITS_LENGTH; j++) {
        tmpRead[j] = receivedCommand[HEX_DIGITS_LENGTH * i + j + 1]; //HEX_DIGITS_LENGTH*i gives position in array for respective ID, +1 omits command prefix
      }
      tmpRead[HEX_DIGITS_LENGTH] = '\0';
      tmpSendLength = strtol(tmpRead, 0, 16);
    */
    lengthChangeCommand = tmpSendLength - lastLengthCommand[i]; //strtol returns long int, lastLengthCommand is unsigned int (4byte - 2byte), changes will not be >int_max
    lastLengthCommand[i] += lengthChangeCommand; //update lastLengthCommand for next command
    angularChangeCommand = (lengthChangeCommand * lengthToAngle) + 0.5;
    pwmCommand[i] += (int)((angularChangeCommand * stepPWMOutput[i] ) + 0.5);

    // keeping pwmCommand in boundaries, enabling crossing
    if (pwmCommand[i] < minimumPWMOutput[i]) {
      pwmCommand[i] = maximumPWMOutput[i] - fabs(fmod(pwmCommand[i], maximumPWMOutput[i]));
    }
    else if (pwmCommand[i] > maximumPWMOutput[i]) {
      pwmCommand[i] = fmod(pwmCommand[i], maximumPWMOutput[i]);
    }
  }
  //}
  receivedCommand[0] = '\0'; //resets array, so it is not read twice
}

void sendNanoCommand() {
  Serial1.print(SEND_PWM_COMMAND);
  Serial.print(SEND_PWM_COMMAND);
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    itoa(pwmCommand[i], commandNano, 16); 
    //strLength = strlen(commandNano); //not necessary, 2 digit hex is 255 max -> smaller than minimumPWM
    Serial1.print(crossing[i]);
    Serial.print(crossing[i]);
    for (int j = 0; j < (DIGITS_PWM_COMMAND - 1); j++) {
      Serial1.print(commandNano[j]);
      Serial.print(commandNano[j]);
    }
  }
  Serial1.println();
  Serial1.flush();
  Serial.flush();
}


