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
#define NUMBER_CONNECTED_NANOS 1
// TODO: Peter will update this to be dynamic
#define SPOOL_CIRCUMFERENCE 1355 //in 0.1mm precision.  //TODO: make it change as spool winds

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
#define CASPR_SETUP 'k'

//set in an h file in the future?
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

//unsigned int angleCommand[NUMBER_CONNECTED_NANOS]; //0.1 degree precision, e.g. 3600 => 360 degree

/////////////////////////// TEMPORARY VARIABLES //////////////

int pwmFeedbackDiff = 0;
char feedbackNano[DIGITS_PWM_FEEDBACK + 1]; // Array to store the nano feedback, +1 to store the '\0' NUL terminator
char commandNano[DIGITS_PWM_COMMAND];
char feedbackMega[HEX_DIGITS_LENGTH]; //4 digit hex length from a nano, sent to mega
int lengthChangeCommand;
float angularChangeCommand;
int strLength = 0;
int readCounter = 0;

/////////////////////////// MISC //////////////

String receivedCommand;

float lengthToAngle = 360.0 / SPOOL_CIRCUMFERENCE; 
float angleToLength = SPOOL_CIRCUMFERENCE / 360.0; 

unsigned long int t_ref;

boolean systemOn, enableMotors;

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
  //pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) { //all the softwareSerials for arduino nano
    serialNano[i].begin(BAUD_RATE_NANO);
    initLength[i] = 32768; //middle
    lengthFeedback[i] = 32768;
    lengthCommand[i] = 32768;//+ 16;
    rangePWMFeedback[i] = maximumPWMFeedback[i] - minimumPWMFeedback[i];
    rangePWMOutput[i] = maximumPWMOutput[i] - minimumPWMOutput[i];
    pwmToAngle[i] = 353.0 / (double)(rangePWMFeedback[i]);  //This range of usable pwm command maps to 353 degrees. The 7 degree left is unreliable/unusable in position mode ("the crossing zone").
    angleToPWM[i] = (double)rangePWMOutput[i] / 353.0;      //This range of usable pwm command maps to 353 degrees.
    pwmMapping[i] = (double)rangePWMOutput[i] / (double)rangePWMFeedback[i]; //factor for mapping PWMFeedback onto PWMOutput


    /////// TEMPORARY - REVISE LATER AFTER CALIBRATION //////////
    readNanoFeedback(i);
    pwmCommand[i] = mapFeedbackToCommand(pwmFeedback[i], i);
  }
  t_ref = millis();
}

/* Main loop acts to interface with MATLAB (asynchronously) and nano at 20Hz */
void loop() {
  readSerialUSB();
}

/* Read serial from matlab and decide what to do depending on the 1-character command.  */
void readSerialUSB() {
  if (Serial.available() > 0) {  //MATLAB via USB
    receivedCommand = Serial.readStringUntil('\n');

    switch (receivedCommand[0]){
      case CASPR_ACKNOWLEDGE: {                        //a: handshake with matlab
        if (receivedCommand.length() == 1){
          systemOn = 0;
          Serial.println(CASPR_ACKNOWLEDGE);
        }
        break;
      }
      case CASPR_START: {                              //s: start system
        if (receivedCommand.length() == 1){
          systemOn = 1;
        }
        break;
      }
      case CASPR_END: {                                //e: end/ turn off system
        if (receivedCommand.length() == 1){
          systemOn = 0;
          enableMotors = 1;      //CLARIFY: why??????
        }
        break;
      }
      case CASPR_INITIAL:{                             //i: initialise system (e.g. set initial length)
        setInitialLengths();
        requestNanoFeedback();
        sendNanoFeedback();
        enableMotors = 1;
        break;
      }
      case CASPR_LENGTH_CMD:{                          //l: move cables to set length
        enableMotors = 1;
        if (systemOn) {
          requestNanoFeedback();
          sendNanoFeedback();

          if (enableMotors) {
            readNanoCommand();
            sendNanoCommand();
          }
        }
        break;
      }
      case CASPR_RESET: {                             //r: resetLengths()?
        resetLengths();
        enableMotors = 1;
        break;
      }
      case CASPR_HOLD: {                               //h: tighten all cables
        enableMotors = 1;
        for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
          pwmCommand[i] = mapFeedbackToCommand(pwmFeedback[i], i) - 10;
        }
        sendNanoCommand();
        break;
      }
      case CASPR_SETUP: {                              //k: make the servo go to where it think it is
        for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
          readNanoFeedback(i);
          pwmCommand[i] = mapFeedbackToCommand(pwmFeedback[i], i);
        }
        break;
      }
      case NANO_TEST: {                                //t: 
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
        break;
      }
    } //switch
  } //if
} //void

/* ????????????????? */
void setInitialLengths() {
  char tmp[4];
  unsigned long int newInitLength;
  for (int j = 0; j < NUMBER_CONNECTED_NANOS; j++) {
    for (int k = 0; k < HEX_DIGITS_LENGTH; k++) {
      tmp[k] = receivedCommand[1 + j * HEX_DIGITS_LENGTH + k];
    }
    pwmLastFeedback[j] = pwmFeedback[j];
    newInitLength = strtol(tmp, 0, 16); //32768
    lengthFeedback[j] += (newInitLength - initLength[j]);
    lengthCommand[j] += (newInitLength - initLength[j]);
    initLength[j] = newInitLength;
  }
}

/* ????????????????? */
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

/* Read feedback from ALL nano, adjust for crossing, and update lengthFeedback[]. */
void requestNanoFeedback() {
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    //read feedback from nano
    readNanoFeedback(i);

    //determine if crossing happened or not
    if (pwmFeedback[i] > pwmLastFeedback[i]) { //possible crossing CCW (right -> left)
      if ((-rangePWMOutput[i] - pwmLastFeedback[i] + pwmFeedback[i]) > (pwmLastFeedback[i] - pwmFeedback[i])) {
        pwmFeedbackDiff = pwmFeedback[i] - rangePWMOutput[i] - pwmLastFeedback[i];
        crossingFeedback[i] = true;                                                       //<<<<<<<<<<<<< BUG: crossing feedback is not used anywhere
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
  }
}

/* Request and read feedback from ONE nano. Convert feedback from HEX to DEC, and store in pwmFeedback. */
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

/* Format and send cable length feedback(stored in lengthFeedback[]) to matlab. */
void sendNanoFeedback() {
  Serial.print(CASPR_FEEDBACK);
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    //convert feedback from DEC to HEX
    itoa(lengthFeedback[i], feedbackMega, 16);
    strLength = strlen(feedbackMega);  //will strLength work when feedbackMega has 4 digit instead of 3, and no room for \0?

    //pad 0 in front
    for (int j = 0; j < (DIGITS_PWM_FEEDBACK + DIGITS_CROSSING_COMMAND - strLength); j++) {
      Serial.print('0');
      Serial.flush();
    }

    //print the HEX string
    for (int j = 0; j < strLength; j++) { //fills sendFeedback array at right position, no conversion necessary
      Serial.print(feedbackMega[j]);
      Serial.flush();
    }
  }
  Serial.println();
  Serial.flush();
}

/* Extract length command from the received matlab string, 
   and save to pwmCommand[] and crossingCommand[], and set enableMotors = 1. */
void readNanoCommand() {
  if (receivedCommand[0] == CASPR_LENGTH_CMD) {
    for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
        char hexCommand[HEX_DIGITS_LENGTH + 1]; //+1 to store the '\0' NUL terminator //CLARIFY: is this need?

        //if receivedCommand is a string (ended with \0)
        //string.substring(from-inclusive, to-exclusive)
        //hexCommand = receivedCommand.substring(1 + (HEX_DIGITS_LENGTH * i), 1 + (HEX_DIGITS_LENGTH * i) + HEX_DIGITS_LENGTH);
        for (int j = 0; j < HEX_DIGITS_LENGTH; j++) {
          hexCommand[j] = receivedCommand[1 + (HEX_DIGITS_LENGTH * i) + j]; // +1 omits command prefix, HEX_DIGITS_LENGTH*i gives position in array for respective ID
        }
        hexCommand[HEX_DIGITS_LENGTH] = '\0'; 
        
        unsigned int tmpSendLength = strtol(hexCommand, 0, 16);  //HEX string -> long -> unsigned int tmpSendLength???????????
        lengthChangeCommand = tmpSendLength - lengthCommand[i]; //strtol returns long int, lengthCommand is unsigned int (4byte - 2byte), changes will not be >int_max
        lengthCommand[i] += lengthChangeCommand; //update lengthCommand for next command  
        angularChangeCommand = (lengthChangeCommand * lengthToAngle); //(float) = (int) * (float)


        
        if (angularChangeCommand > 0) {
          pwmCommand[i] += (int)((angularChangeCommand * angleToPWM[i] ) + 0.5);
        } else pwmCommand[i] += (int)((angularChangeCommand * angleToPWM[i] ) - 0.5);

      
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
  }
  receivedCommand[0] = '\0'; //resets array, so it is not read twice
  enableMotors = 1;
}

/* Broadcast command to all nanos via Serial1 */
void sendNanoCommand() {
  Serial1.print(NANO_PWM_COMMAND);  //tell the nanos that this is a pwm command
  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    //send crossing command
    Serial1.print(crossingCommand[i]);
    if (crossingCommand[i] > 0) {
      crossingCommand[i] = 0;
    }

    //convert pwm command to HEX chars and send
    itoa(pwmCommand[i], commandNano, 16);
    for (int j = 0; j < DIGITS_PWM_COMMAND; j++) {
      Serial1.print(commandNano[j]);
    }
  }
  Serial1.println();
  Serial1.flush();
}

//To be put into use
/* Map pwm feedback of the specified position to pwm command */
int mapFeedbackToCommand(int feedback, int id){
  return (pwmFeedback[id] - minimumPWMFeedback[id]) * pwmMapping[id] + minimumPWMOutput[id] + 0.5;  //+0.5 to turn double-to-int truncation into round-off
}


