/**
   MEGA

   send via TX1
   receive via digital pins

   Not all pins on the Mega and Mega 2560 support change interrupts,
   so only the following can be used for RX:
   10, 11, 12, 13, 14, 15, 50, 51, 52, 53,
   A8 (62), A9 (63), A10 (64), A11 (65),
   A12 (66), A13 (67), A14 (68), A15 (69).
  (18 pins in total.)
*/

#include <SoftwareSerial.h>

// TODO: This should be sent by MATLAB in the future
#define NUMBER_CONNECTED_NANOS 8
// TODO: Peter will update this to be dynamic
#define RADIUS 210 //spool, in average radius in 0.1mm precision  actual radius is 20mm **Improve in future

// These two are useless now because MATLAB is in control of time
//#define FEEDBACK_FREQUENCY 20// In Hz
//#define TIME_STEP 0.07

#define BAUD_RATE_NANO 74880
#define BAUD_RATE_CASPR 74880

#define HEX_DIGITS_LENGTH 4
#define DIGITS_PWM_COMMAND 3
#define DIGITS_PWM_FEEDBACK 3
#define DIGITS_CROSSING_COMMAND 1

#define CASPR_FEEDBACK 'f'
#define CASPR_LENGTH_CMD 'l'
#define CASPR_ACKNOWLEDGE 'a'
#define CASPR_START 's'
#define CASPR_END 'e'
#define CASPR_INITIAL 'i'
#define CASPR_RESET 'r'
#define CASPR_HOLD 'h'
#define CASPR_WAVE 'w'
#define CASPR_SETUP 'k'

#define NANO_PWM_COMMAND 'p'
#define NANO_FEEDBACK 'f'
#define NANO_TEST 't'
#define NANO_TESTDRIVE 'z'

/////////////////////////// SERVO PARAMETERS //////////////
// TODO: move to each nano
int maximumPWMFeedback[8] = {1494, 1500, 1496, 1496, 1509, 1496, 1504, 1494};
int minimumPWMFeedback[8] = {499, 500, 499, 499, 504, 499, 499, 499};
int maximumPWMOutput[8] = {1482, 1488, 1484, 1483, 1495, 1482, 1492, 1482};
int minimumPWMOutput[8] = {485, 485, 485, 485, 491, 485, 485, 485};

//int middlePWMFeedback[8] = {994, 988, 992, 987, 992, 991, 992, 1005}; // all numbers rounded down  //orginal value for servo0 on old dingbot: 992
//int clockwise_max[8] = {2194, 2175, 2185, 2175, 2189, 2188, 2188, 2215};
//int clockwise_min[8] = {2098, 2082, 2090, 2079, 2089, 2088, 2088, 2117};
//int clockwise_max_speed[8] = {55, 278, 272, 269, 272, 281, 278, 278};
//int clockwise_min_speed[8] = {25, 131, 127, 127, 127, 128, 133, 130};
//int anticlockwise_max[8] = {1800, 1780, 1785, 1780, 1785, 1786, 1788, 1811};
//int anticlockwise_min[8] = {1891, 1880, 1887, 1876, 1885, 1886, 1888, 1910};
//int anticlockwise_max_speed[8] = { -55, -278, -273, -269, -270, -279, -273, -279};
//int anticlockwise_min_speed[8] = { -26, -130, -132, -129, -124, -129, -128, -131};

// speed: deltaPWM per 50ms// roughly 8pwm / 10ms on clockwise max

int rangePWMOutput[NUMBER_CONNECTED_NANOS];
int rangePWMFeedback[NUMBER_CONNECTED_NANOS];
unsigned int initLength[NUMBER_CONNECTED_NANOS];
double pwmToAngle[NUMBER_CONNECTED_NANOS];
double angleToPWM[NUMBER_CONNECTED_NANOS];
double pwmMapping[NUMBER_CONNECTED_NANOS];

/////////////////////////// OPERATION ARRAYS //////////////

int pwmFeedback[NUMBER_CONNECTED_NANOS];
int pwmLastFeedback[NUMBER_CONNECTED_NANOS];
int pwmCommand[NUMBER_CONNECTED_NANOS];

boolean crossingFeedback[NUMBER_CONNECTED_NANOS];
int crossingCommand[NUMBER_CONNECTED_NANOS];
unsigned int lengthCommand[NUMBER_CONNECTED_NANOS]; //unsigned int has 2 bytes, range 0 - 65535
unsigned int lengthFeedback[NUMBER_CONNECTED_NANOS]; //with .1 mm precision, this equals ~6.5m

/////////////////////////// TEMPORARY VARIABLES //////////////

int pwmFeedbackDiff = 0;
char feedbackNano[DIGITS_PWM_FEEDBACK + 1]; // Array to store the nano feedback, +1 to store the '\0' NUL terminator
char commandNano[DIGITS_PWM_COMMAND];
char feedbackMega[HEX_DIGITS_LENGTH]; //4 digit hex length from a nano, sent to mega
int lengthChangeCommand;
char tmpRead[HEX_DIGITS_LENGTH + 1]; //+1 to store the '\0' NUL terminator
unsigned int tmpSendLength;
float angularChangeCommand;
int strLength = 0;
int readCounter = 0;

/////////////////////////// MISC //////////////

String receivedCommand;

float lengthToAngle = 180.0 / (M_PI * RADIUS); //1.091348181201570 with Radius 210
float angleToLength = (M_PI * RADIUS) / 180.0; // 0.9162978572970230 with Radius 210

unsigned long int t_ref;

boolean systemOn, enableMotors, wave;


/////////////////////////// SIN WAVE TESTING //////////////

int radCounter = 0;
float rad = 0;
int sinPWM = 0;
double waveFactor = 0.025;

// TODO: Decide on the maximum number of Nanos we would ever connect
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
  Serial.begin(BAUD_RATE_CASPR);  //USB
  Serial1.begin(BAUD_RATE_NANO); //broadcast
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) { //all the softwareSerials for arduino nano
    serialNano[i].begin(BAUD_RATE_NANO);
    initLength[i] = 32768; //middle
    lengthFeedback[i] = 32768;
    lengthCommand[i] = 32768;//+ 16;
    rangePWMFeedback[i] = maximumPWMFeedback[i] - minimumPWMFeedback[i];
    pwmToAngle[i] = 357.0 / (double)(rangePWMFeedback[i]);  //This pwm range corresponds to 357 degrees. The 3 degree left is unusable ("crossing").
    rangePWMOutput[i] = maximumPWMOutput[i] - minimumPWMOutput[i];
    angleToPWM[i] = (double)rangePWMOutput[i] / 357.0; //Depends on the servo's pwm range. This pwm range corresponds to 357 degrees.
    pwmMapping[i] = (double)rangePWMOutput[i] / (double)rangePWMFeedback[i]; //factor for mapping PWMFeedback onto PWMOutput


    /////// TEMPORARY - REVISE LATER AFTER CALIBRATION //////////
    readNanoFeedback(i);
    pwmCommand[i] = (pwmFeedback[i] - minimumPWMFeedback[i]) * pwmMapping[i] + minimumPWMOutput[i] + 0.5;  //+0.5 to turn double-to-int truncation into round-off
  }
  t_ref = millis();
}

/* Main loop acts to interface with MATLAB (asynchronously) and nano at 20Hz */
void loop() {
  readSerialUSB();
  //  if ((millis() - t_ref) > TIME_STEP * 1000) { // Operate at roughly 20Hz time
  //  t_ref = millis(); // Reset the time
  //  }
}

void readSerialUSB() {
  if (Serial.available() > 0) {  //MATLAB via USB
    receivedCommand = Serial.readStringUntil('\n');
    if (receivedCommand[0] == CASPR_ACKNOWLEDGE && receivedCommand.length() == 1) //a
    {
      systemOn = 0;
      wave = 0;
      Serial.println(CASPR_ACKNOWLEDGE);
    }

    else if (receivedCommand[0] == CASPR_START && receivedCommand.length() == 1) //s
    {
      systemOn = 1;
      wave = 0;
    }
    else if (receivedCommand[0] == CASPR_END && receivedCommand.length() == 1) //e
    {
      systemOn = 0;
      enableMotors = 1;
      wave = 0;
    }
    else if (receivedCommand[0] == CASPR_INITIAL) //i
    {
      setInitialLengths();
      requestNanoFeedback();
      sendNanoFeedback();
      enableMotors = 1;
      wave = 0;
    }
    else if (receivedCommand[0] == CASPR_LENGTH_CMD) //l
    {
      enableMotors = 1;
      wave = 0;
      if (systemOn) {
        requestNanoFeedback();
        sendNanoFeedback();
        //Serial.print('x');
        // for (int i = 0; i < 4; i++) {
        //  Serial.print(receivedCommand[i + 1]);
        //   }

        if (enableMotors) {
          readNanoCommand();
          sendNanoCommand(); // Set up to send command for the nano
        }
      }
    }

    else if (receivedCommand[0] == CASPR_RESET) //r
    {
      resetLengths();
      enableMotors = 1;
      wave = 0;
    }
    else if (receivedCommand[0] == CASPR_HOLD) //h
    {
      enableMotors = 1;
      wave = 0;
      for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
        pwmCommand[i] = (int)((pwmFeedback[i] - minimumPWMFeedback[i]) * pwmMapping[i]) + minimumPWMOutput[i] - 10;
      }
      sendNanoCommand(); //this sends the detected position from the nano, mapped to the PWMOutput range - no movement, only locking of motors
    }
    else if (receivedCommand[0] == CASPR_WAVE) //w
    {
      wave = 1;
      enableMotors = 1;
      waveFactor = 0.025 * (receivedCommand[1] - '0');
      if (waveFactor > 0.125) {
        waveFactor = 0.125;
      }
    }
    else if (receivedCommand[0] == CASPR_SETUP) //k
    {
      for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
        readNanoFeedback(i);
        pwmCommand[i] = (pwmFeedback[i] - minimumPWMFeedback[i]) * pwmMapping[i] + minimumPWMOutput[i];
      }
    }
    else if (receivedCommand[0] == NANO_TEST) //t
    {
      enableMotors = 1;
      if (systemOn) {
        Serial1.print('p');
        Serial1.print("f00");

        requestNanoFeedback();
        sendNanoFeedback();
      }
      if (enableMotors) {
        readNanoCommand();
        Serial1.print(NANO_PWM_COMMAND);
      }
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
    pwmLastFeedback[j] = pwmFeedback[j];
    newInitLength = strtol(tmp, 0, 16); //32768
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
    lengthCommand[j] = resetLength;
  }
}

void requestNanoFeedback() {
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    readNanoFeedback(i);
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

    //convert PWM change to length change
    double lengthFeedbackDiff = pwmFeedbackDiff * pwmToAngle[i] * angleToLength;

    //add length change to the current length. For the double to int conversion, +-0.5 is used to round-off the value instead of truncating it.
    if (lengthFeedbackDiff > 0){
      lengthFeedback[i] += (int)(lengthFeedbackDiff + 0.5);
    } else if (lengthFeedbackDiff < 0){
      lengthFeedback[i] += (int)(lengthFeedbackDiff - 0.5);
    }
    
    
   // Serial.println(pwmFeedbackDiff);
  }
 
}

/* Request and read feedback from nano. Convert feedback from HEX to DEC, and store in pwmFeedback. */
void readNanoFeedback(int i) {
  serialNano[i].listen();
  Serial1.println(NANO_FEEDBACK + String(i)); //requests feedback from nano
  Serial1.flush(); //waits for the sending of Serial to be complete before moving on
  
  readCounter = 0;
  while ((serialNano[i].available() == 0) && readCounter < 200) { //wait for the feedback from nano
    readCounter++;
  }
  if (serialNano[i].available() > 0) {
    for (int j = 0; j < DIGITS_PWM_FEEDBACK; j++) { //read feedback
      feedbackNano[j] = serialNano[i].read();
    }
    feedbackNano[DIGITS_PWM_FEEDBACK] = '\0';  

    while (serialNano[i].available() > 0) {
      serialNano[i].read(); //clears the buffer of any other bytes
    }

    pwmFeedback[i] = strtol(feedbackNano, 0, 16); 
  }
}

void sendNanoFeedback() {

  Serial.print(CASPR_FEEDBACK);
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {

    itoa(lengthFeedback[i], feedbackMega, 16);
  //  Serial.println(lengthFeedback[i]); //not 32768 lol
    strLength = strlen(feedbackMega);  //will strLength work when feedbackMega has 4 digit instead of 3, and no room for \0?

    for (int j = 0; j < (DIGITS_PWM_FEEDBACK + DIGITS_CROSSING_COMMAND - strLength); j++) {
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
  if (receivedCommand[0] == CASPR_LENGTH_CMD || wave) {
    for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
      if (!wave) {
        for (int j = 0; j < HEX_DIGITS_LENGTH; j++) {
          tmpRead[j] = receivedCommand[HEX_DIGITS_LENGTH * i + j + 1]; //HEX_DIGITS_LENGTH*i gives position in array for respective ID, +1 omits command prefix
        }
        tmpRead[HEX_DIGITS_LENGTH] = '\0';
        tmpSendLength = strtol(tmpRead, 0, 16);  //HEX string -> long -> unsigned int tmpSendLength??????????????????????????????????????????
        lengthChangeCommand = tmpSendLength - lengthCommand[i]; //strtol returns long int, lengthCommand is unsigned int (4byte - 2byte), changes will not be >int_max
        lengthCommand[i] += lengthChangeCommand; //update lengthCommand for next command  
        //^the line above is the same as lengthCommand[i] = lengthCommand[i] + (tmpSendLength - lengthCommand[i]) = tmpSendLength????????????
        angularChangeCommand = (lengthChangeCommand * lengthToAngle); //(float) = (int) * (float)
        if (angularChangeCommand > 0) {
          pwmCommand[i] += (int)((angularChangeCommand * angleToPWM[i] ) + 0.5);
        } else pwmCommand[i] += (int)((angularChangeCommand * angleToPWM[i] ) - 0.5);

      } else {
        radCounter++;
        rad = radCounter * waveFactor;
        sinPWM = sin(rad) * 480;
        sinPWM += 977;
        for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
          pwmCommand[i] = sinPWM;
          crossingCommand[i] = 0;
        }
      }
      // keeping pwmCommand in boundaries, enabling crossing
      crossingCommand[i] = 0;
      //pwmCommand[i] -= 5;
      if (pwmCommand[i] < minimumPWMOutput[i]) { //CROSSING RIGHT -> LEFT
        pwmCommand[i] = maximumPWMOutput[i] - fabs(fmod(minimumPWMOutput[i], pwmCommand[i]));   //CLARIFY: does this assume the max and min PWMOutput is the same location?
        crossingCommand[i] = 2;
      } else if (pwmCommand[i] > maximumPWMOutput[i]) { //CROSSING LEFT -> RIGHT
        pwmCommand[i] = minimumPWMOutput[i] + fmod(pwmCommand[i], maximumPWMOutput[i]);
        crossingCommand[i] = 1;
      }
    }
    //  Serial.print(crossingCommand[0]);
    //  Serial.println(pwmCommand[0]);
  }
  receivedCommand[0] = '\0'; //resets array, so it is not read twice
  enableMotors = 1;
}

void sendNanoCommand() {
  Serial1.print(NANO_PWM_COMMAND);
  //Serial.print(NANO_PWM_COMMAND);
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    Serial1.print(crossingCommand[i]);
    // Serial.print(crossingCommand[i]);
    if (crossingCommand[i] > 0) {
      crossingCommand[i] = 0;
    }
    itoa(pwmCommand[i], commandNano, 16);
    for (int j = 0; j < DIGITS_PWM_COMMAND; j++) {
      Serial1.print(commandNano[j]);
      // Serial.print(commandNano[j]);
    }
  }
  Serial1.println();
  Serial1.flush();
  //Serial.flush();
}


