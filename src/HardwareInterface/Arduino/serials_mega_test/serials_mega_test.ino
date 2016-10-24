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

#define NUMBER_CONNECTED_NANOS 8
#define RADIUS 210 //spool, in average radius in 0.1mm precision  actual radius is 20mm

#define FEEDBACK_FREQUENCY 20// In Hz
#define TIME_STEP 1.0/FEEDBACK_FREQUENCY
#define BAUD_RATE 74880

#define HEX_DIGITS_LENGTH 4
#define DIGITS_PWM_COMMAND 3
#define DIGITS_PWM_FEEDBACK 3
#define CROSSING_ID 1

#define CASPR_FEEDBACK 'f'
#define CASPR_LENGTH_CMD 'l'
#define CASPR_ACKNOWLEDGE 'a'
#define CASPR_START 's'
#define CASPR_END 'e'
#define CASPR_INITIAL 'i'
#define CASPR_RESET 'r'
#define CASPR_HOLD 'h'

#define NANO_PWM_COMMAND 'p'
#define NANO_FEEDBACK 'f'
#define NANO_TEST 't'
#define NANO_TESTDRIVE 'z'

/////////////////////////// SERVO PARAMETERS //////////////

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


int rangePWMOutput[NUMBER_CONNECTED_NANOS];
int rangePWMFeedback[NUMBER_CONNECTED_NANOS];
unsigned int initLength[NUMBER_CONNECTED_NANOS];
double stepPWMFeedback[NUMBER_CONNECTED_NANOS];
double stepPWMOutput[NUMBER_CONNECTED_NANOS];
double pwmMapping[NUMBER_CONNECTED_NANOS];

/////////////////////////// OPERATION ARRAYS //////////////

int pwmFeedback[NUMBER_CONNECTED_NANOS];
int pwmLastFeedback[NUMBER_CONNECTED_NANOS];

boolean crossingFeedback[NUMBER_CONNECTED_NANOS];
int crossingCommand[NUMBER_CONNECTED_NANOS];
unsigned int lengthCommand[NUMBER_CONNECTED_NANOS]; //unsigned int has 2 bytes, range 0 - 65535
unsigned int lengthFeedback[NUMBER_CONNECTED_NANOS]; //with .1 mm precision, this equals ~6.5m

/////////////////////////// TEMPORARY VARIABLES //////////////

int pwmFeedbackDiff = 0;
char feedbackNano[DIGITS_PWM_FEEDBACK]; // Array to store the nano feedback
char commandNano[DIGITS_PWM_COMMAND];
char feedbackMega[HEX_DIGITS_LENGTH]; //4 digit hex length from a nano, sent to mega
int lengthChangeCommand;
char tmpRead[HEX_DIGITS_LENGTH];
unsigned int tmpSendLength;
int angularChangeCommand;
int strLength = 0;
int readCounter = 0;

/////////////////////////// MISC //////////////

String receivedCommand;

float lengthToAngle = 720.0 / (M_PI * RADIUS); //1.091348181201570 with Radius 210
float angleToLength = (M_PI * RADIUS) / 720.0; // 0.9162978572970230 with Radius 210

unsigned long int t_ref;

boolean systemOn, enableMotors;


/////////////////////////// SIN WAVE TESTING //////////////

int radCounter = 0;
float rad = 0;
int sinPWM = 0;
int sinDeg = 0;
int lastSinDeg = 0;
int sinAngleChange = 0;
char sinAngleCmd[2];
boolean sinPositive = 0;
int sendPWM = 0;




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
  //Serial.print("feedback: ");
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) { //all the softwareSerials for arduino nano
    serialNano[i].begin(BAUD_RATE);
    initLength[i] = 32768; //middle
    lengthFeedback[i] = 32768;
    lengthCommand[i] = 32768;
    rangePWMFeedback[i] = maximumPWMFeedback[i] - minimumPWMFeedback[i];
    stepPWMFeedback[i] = 1440.0 / (double)(rangePWMFeedback[i]);
    rangePWMOutput[i] = maximumPWMOutput[i] - minimumPWMOutput[i];
    stepPWMOutput[i] = (double)rangePWMOutput[i] / 1440.0; //360 degree in quarter degree precision -> 1440 steps
    pwmMapping[i] = (double)rangePWMOutput[i] / (double)rangePWMFeedback[i]; //factor for mapping PWMFeedback onto PWMOutput


    /////// TEMPORARY - REVISE LATER AFTER CALIBRATION //////////

    serialNano[i].listen();
    Serial1.println('f' + String(i)); //requests feedback from nano
    Serial1.flush(); //waits for the sending of Serial to be complete before moving on

    readCounter = 0;
    while ((serialNano[i].available() == 0) && readCounter < 200) {
      readCounter++;
    }
    if (serialNano[i].available() > 0) {
      for (int j = 0; j < DIGITS_PWM_FEEDBACK; j++) {
        feedbackNano[j] = serialNano[i].read();
      }
      feedbackNano[DIGITS_PWM_FEEDBACK] = '\0';

      while (serialNano[i].available() > 0) {
        serialNano[i].read(); //clears the buffer of any other bytes
      }
      pwmFeedback[i] = strtol(feedbackNano, 0, 16);
      pwmCommand[i] = (pwmFeedback[i] - minimumPWMFeedback[i]) * pwmMapping[i] + minimumPWMOutput[i];
      //  Serial.print(pwmFeedback[i]);
      //  Serial.print(" step " + String(i));
      //   Serial.println(stepPWMFeedback[i]);
    }
  }
  // Serial.println();
  t_ref = millis();
}

/* Main loop acts to interface with MATLAB (asynchronously) and nano at 20Hz */
void loop() {
  readSerialUSB();
  if ((millis() - t_ref) > TIME_STEP * 1000) { // Operate at roughly 20Hz time
    t_ref = millis(); // Reset the time
    if (systemOn) {
      requestNanoFeedback();
      if (enableMotors) {
        readNanoCommand();
        sendNanoCommand(); // Set up to send command for the nano
      }
      sendNanoFeedback();
    }
  }
}

void readSerialUSB() {
  if (Serial.available() > 0) {  //MATLAB via USB
    receivedCommand = Serial.readStringUntil('\n');
    if (receivedCommand[0] == CASPR_ACKNOWLEDGE && receivedCommand.length() == 1) //a
    {
      systemOn = 0;
      Serial.println(CASPR_ACKNOWLEDGE);
    }

    else if (receivedCommand[0] == CASPR_START && receivedCommand.length() == 1) //s
    {
      systemOn = 1;
    }
    else if (receivedCommand[0] == CASPR_END && receivedCommand.length() == 1) //e
    {
      systemOn = 0;
      enableMotors = 1;
    }
    else if (receivedCommand[0] == CASPR_INITIAL) //i
    {
      setInitialLengths();
      enableMotors = 1;
    }
    else if (receivedCommand[0] == CASPR_LENGTH_CMD) //l
    {
      enableMotors = 1;
    }
    else if (receivedCommand[0] == CASPR_RESET) //r
    {
      resetLengths();
      enableMotors = 1;
    }
    else if (receivedCommand[0] == CASPR_HOLD) //b
    {
      enableMotors = 1;
      for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
        pwmCommand[i] = (int)(pwmFeedback[i] * pwmMapping[i]) - 10;
      }
      sendNanoCommand(); //this sends the detected position from the nano, mapped to the PWMOutput range - no movement, only locking of motors
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
    lengthFeedback[j] += (newInitLength - initLength[j]);
    lengthCommand[j] += (newInitLength - initLength[j]);
    initLength[j] = newInitLength;
  }
}

void resetLengths() {
  char tmp[4];
  unsigned long int resetLength;
  for (int j = 0; j < NUMBER_CONNECTED_NANOS; j++) {
    for (int k = 0; k < HEX_DIGITS_LENGTH; k++) {
      tmp[k] = receivedCommand[j * HEX_DIGITS_LENGTH + k + 1];
    }
    resetLength = strtol(tmp, 0, 16);
    lengthFeedback[j] = resetLength;
    lengthCommand[j] += resetLength;
  }
}

void requestNanoFeedback() {
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    serialNano[i].listen();
    Serial1.println(NANO_FEEDBACK + String(i)); //requests feedback from nano
    Serial1.flush(); //waits for the sending of Serial to be complete before moving on

    readCounter = 0;
    while ((serialNano[i].available() == 0) && readCounter < 200) {
      readCounter++;
    }
    if (serialNano[i].available() > 0) {
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
          crossingFeedback[i] = true;
        } else {
          pwmFeedbackDiff = pwmFeedback[i] - pwmLastFeedback[i];
          crossingFeedback[i] = false;
        }
      } else if ((rangePWMOutput[i] - pwmLastFeedback[i] + pwmFeedback[i]) < (pwmLastFeedback[i] - pwmFeedback[i])) { //crossing CW (left -> right)
        pwmFeedbackDiff = rangePWMOutput[i] - pwmLastFeedback[i] + pwmFeedback[i];
        crossingFeedback[i] = true;
      } else {
        pwmFeedbackDiff = pwmFeedback[i] - pwmLastFeedback[i];
        crossingFeedback[i] = false;
      }
      pwmLastFeedback[i] = pwmFeedback[i];

      int test = (int)(((pwmFeedbackDiff * stepPWMFeedback[i]) * angleToLength));
      //   Serial.print(" test ");
      //   Serial.print(test);

      lengthFeedback[i] += test; //conversion of pwmDiff to angleChange to lengthChange

    }
  }
}

void sendNanoFeedback() {
  Serial.print(CASPR_FEEDBACK);
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    itoa(lengthFeedback[i], feedbackMega, 16);
    strLength = strlen(feedbackMega);

    for (int j = 0; j < (DIGITS_PWM_FEEDBACK + CROSSING_ID - strLength); j++) {
      Serial.print('0');
      Serial.flush();
    }
    for (int j = 0; j < strLength; j++) { //fills sendFeedback array at right position, no conversion necessary
      Serial.print(feedbackMega[j]);
      Serial.flush();
    }
  }
  Serial.println();
  Serial.flush();
}


void readNanoCommand() {
  if (receivedCommand[0] == CASPR_LENGTH_CMD) {
    for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
      for (int j = 0; j < HEX_DIGITS_LENGTH; j++) {
        tmpRead[j] = receivedCommand[HEX_DIGITS_LENGTH * i + j + 1]; //HEX_DIGITS_LENGTH*i gives position in array for respective ID, +1 omits command prefix
      }
      tmpRead[HEX_DIGITS_LENGTH] = '\0';
      tmpSendLength = strtol(tmpRead, 0, 16);
      lengthChangeCommand = tmpSendLength - lengthCommand[i]; //strtol returns long int, lengthCommand is unsigned int (4byte - 2byte), changes will not be >int_max
      lengthCommand[i] += lengthChangeCommand; //update lengthCommand for next command
      angularChangeCommand = (lengthChangeCommand * lengthToAngle) + 0.5;

      pwmCommand[i] += (int)((angularChangeCommand * stepPWMOutput[i] ) + 0.5);

      // keeping pwmCommand in boundaries, enabling crossing
      if (pwmCommand[i] < minimumPWMOutput[i]) { //CROSSING RIGHT -> LEFT
        pwmCommand[i] = maximumPWMOutput[i] - fabs(fmod(minimumPWMOutput[i], pwmCommand[i]));
        crossingCommand[i] = 2;
      } else if (pwmCommand[i] > maximumPWMOutput[i]) { //CROSSING LEFT -> RIGHT
        pwmCommand[i] = minimumPWMOutput[i] + fmod(pwmCommand[i], maximumPWMOutput[i]);
        crossingCommand[i] = 1;
      } else crossingCommand[i] = 0;
    }
  }
  receivedCommand[0] = '\0'; //resets array, so it is not read twice
  enableMotors = 1;
}

void sendNanoCommand() {
  Serial1.print(NANO_PWM_COMMAND);
  //Serial.print("  command ");
  //Serial.print(NANO_SEND_PWM_COMMAND);
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    Serial1.print(crossingCommand[i]);
    //Serial.print(crossingCommand[i]);
    itoa(pwmCommand[i], commandNano, 16);
    for (int j = 0; j < DIGITS_PWM_COMMAND; j++) {
      Serial1.print(commandNano[j]);
      //Serial.print(commandNano[j]);
    }
  }
  Serial1.println();
  //Serial.print(" ");
  Serial1.flush();
}


