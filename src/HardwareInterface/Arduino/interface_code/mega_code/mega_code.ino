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

#define NUMBER_CONNECTED_NANOS 8                        //TODO: This should be sent by MATLAB in the future

//for ccalculation of varying spool circumference
#define MAX_CABLE_LENGTH 20400               //(unit: 0.1mm)  MAX_CABLE_LENGTH - length not on the spool = length on the spool
#define AVERAGE_SPOOL_CIRCUMFERENCE 1345     //(unit: 0.1mm)  length on the spool / AVERAGE_SPOOL_CIRCUMFERENCE = estimated number of loops on the spool
#define VARYING_SPOOL_CIRCUMFERENCE_M 5.5051 //(unit: 0.1mm)  linear euqation that maps number of loops on the spool to spool cirucmference
#define VARYING_SPOOL_CIRCUMFERENCE_B 1304   //(unit: 0.1mm)

#define BAUD_RATE_NANO 74880
#define BAUD_RATE_CASPR 74880

#define HEX_DIGITS_LENGTH 4
#define DIGITS_ANGLE_COMMAND 3
#define DIGITS_CROSSING_COMMAND 1
#define DIGITS_ANGLE_FEEDBACK 3

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
#define NANO_ANGLE_COMMAND 'c'
#define NANO_QUICK_FEEDBACK_REQUEST 'f'
#define NANO_UPDATE_FEEDBACK 'u'
#define NANO_FINISH_TRAJECTORY 'e'

#define CLOCKWISE 1
#define ANTICLOCKWISE 2

#define CROSSING_ZONE_SIZE 70   //in 0.1degree

#define CABLE_TIGHTEN_INCREMENT 10  //in 0.1mm precision. e.g. tighten all cables by 1mm when matlab sends an 'h' command

boolean systemOn = false;      //no servo movement or feedback when off
boolean motorsEnabled = false;  //no servo movement when off

//TODO: make sure these are populated before use
unsigned int lastAngleFeedbacks[NUMBER_CONNECTED_NANOS]; //in 0.1 degree precision
//help determine whether crossing has happened (used in processing feedback data) or will happen(processing command data)
//help convert angle feedback (absolute) to angle change feedback (relative). This help decide whether crossing has happened or not

unsigned int currentCableLengths[NUMBER_CONNECTED_NANOS];  //in 0.1mm precision
//help convert angle change feedback (relative) to an updated current cable length (absolute), to feed back to matlab/CASPRROS

unsigned int lastLengthCommands[NUMBER_CONNECTED_NANOS]; //in 0.1mm precision
//help convert new length command (absolute) to angle change command (relative). This help decide whether crossing is needed or not

unsigned int lastAngleCommands[NUMBER_CONNECTED_NANOS];  //in 0.1 degree precision
//help convert angle change command (relative) to angle command (absolute), to send to nano.

////////////////// function prototypes (needed because arduino won't do it for you if you use & references ///////////////////////////////////
void extractLengths(const String &receivedCommand, unsigned int lengths[NUMBER_CONNECTED_NANOS]);
void updateInitialLengths(unsigned int newInitialLengths[NUMBER_CONNECTED_NANOS]);
void resetLengths(unsigned int newLengths[NUMBER_CONNECTED_NANOS]);
void computeCrossingAndAngleCommands(unsigned int lengthCommands[NUMBER_CONNECTED_NANOS], int crossingCommands[NUMBER_CONNECTED_NANOS], unsigned int angleCommands[NUMBER_CONNECTED_NANOS]);
void broadcastCommandsToNanos(int crossingCommands[NUMBER_CONNECTED_NANOS], unsigned int angleCommands[NUMBER_CONNECTED_NANOS]);
unsigned int readAngleFromNano(int nanoID);
void computeCurrentCableLengths(unsigned int currentAngles[NUMBER_CONNECTED_NANOS]);
void sendCurrentCableLengths();
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
   Serial for MATLAB/CASPRROS (via USB)
   Serial1 for broadcasting to Nano devices
   SoftwareSerial for receiving from individual Nanos.
   
   Populate 3 global arrays. */
void setup() {
  Serial.begin(BAUD_RATE_CASPR);  //USB
  Serial1.begin(BAUD_RATE_NANO); //for broadcasting to nano

  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {   
    serialNano[i].begin(BAUD_RATE_NANO); //for receiving from nano
  }
}

void loop() { 
  if (Serial.available() > 0){
    String receivedCommand = Serial.readStringUntil('\n');

    if (receivedCommand.length() == 1){ /////////////////////////////////////////////////////////////////
      switch (receivedCommand[0]){
        case CASPR_ACKNOWLEDGE:                //a: handshake with matlab
          {
            systemOn = false;
            Serial.println(CASPR_ACKNOWLEDGE);
          }
          break;
        case CASPR_SETUP:                      //k: [current usage: populate necessary arrays]
          {
              Serial1.println(NANO_UPDATE_FEEDBACK); //nano need to update its feedback value after idling between trajectories
              delay(12);  //wait for nano to get data from servo. (~2050us/feedback * 4 feedbacks, averaged?)
              
              //populate lastAngleFeedbacks[] and lastAngleCommands[]
              for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++){
                lastAngleFeedbacks[n] = readAngleFromNano(n);
                lastAngleCommands[n] = lastAngleFeedbacks[n];
              }

              //initialLengths[], currentCableLengths[], and lastLengthCommands[] will be populated in the CASPR_INITIAL case
          }
          break;
        case CASPR_START:                      //s: start system
          {
            systemOn = true;
          }
          break;
        case CASPR_END:                        //e: end (turn off) system
          {
            systemOn = false;
            motorsEnabled = false;
            Serial1.println(NANO_FINISH_TRAJECTORY);
          }
          break;
        case CASPR_HOLD:                       //h: tighten all cables  [TODO: implemnt later]
          {
//            unsigned int tightenedLengths[NUMBER_CONNECTED_NANOS];
//            for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++) {
//              tightenedLengths[n] = lastLengthCommands[n] - 10; //1mm, TODO: remove magic number
//            }
//            int crossingCommands[NUMBER_CONNECTED_NANOS];
//            unsigned int angleCommands[NUMBER_CONNECTED_NANOS];
//            computeCrossingAndAngleCommands(tightenedLengths, crossingCommands, angleCommands);
//            broadcastCommandsToNanos(crossingCommands, angleCommands);
          }
          break;
      }//switch
    } else { //receivedCommand.length() > 1  ////////////////////////////////////////////////////////////
      switch (receivedCommand[0]) {
        case CASPR_INITIAL:                    //i: update initial lengths
          {
            unsigned int newInitialLengths[NUMBER_CONNECTED_NANOS];
            extractLengths(receivedCommand, newInitialLengths); //from receivedCommand, extract length info to newInitialLengths[]
            updateInitialLengths(newInitialLengths); //use newInitialLengths to update initialLengths[], currentCableLengths[], and lastLengthCommands[]
            sendCurrentCableLengths(); //assure CASPROS/matlab side that it's at the right position
          }
          break;
        case CASPR_LENGTH_CMD:                 //l: Get nano feedback to matlab, then matlab command to nano
          {
            motorsEnabled = true; //BUG/TODO: this setting is useless?
            
            if (systemOn){
              unsigned int currentAngles[NUMBER_CONNECTED_NANOS];
              for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++){
                currentAngles[n] = readAngleFromNano(n);
              }
              computeCurrentCableLengths(currentAngles); //using currentAngles[], currentCableLengths[], and lastAngleFeedbacks[]; update currentCableLengths[]
              sendCurrentCableLengths(); //send currentCableLengths[] to Serial
              for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++){
                lastAngleFeedbacks[n] = currentAngles[n];
              }

              if (motorsEnabled){
                unsigned int lengthCommands[NUMBER_CONNECTED_NANOS];
                extractLengths(receivedCommand, lengthCommands); //from receivedCommand, extract length info to lengthCommands[]
                int crossingCommands[NUMBER_CONNECTED_NANOS];
                unsigned int angleCommands[NUMBER_CONNECTED_NANOS];
                computeCrossingAndAngleCommands(lengthCommands, crossingCommands, angleCommands); //using lengthCommands[], lastLengthCommands[], and lastAngleFeedbacks[] to generate crossingCommands[] and angleCommands[]
                broadcastCommandsToNanos(crossingCommands, angleCommands);
              }
            }
          }
          break;
        case CASPR_RESET:                   //r: reset lengths (make the cable robot think it is at a specific location)
          {
            unsigned int newLengths[NUMBER_CONNECTED_NANOS];
            extractLengths(receivedCommand, newLengths);
            resetLengths(newLengths);  //using newLengths[], update currentCableLengths[] and lastLengthCommands[]

            motorsEnabled = true;    //CLARIFY: why?
          }
          break;
      }//switch
    }//is multi-char cmd
  }//if Serial.available
}

void extractLengths(const String &receivedCommand, unsigned int lengths[NUMBER_CONNECTED_NANOS]) {
  for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++) {
    char hexLength[HEX_DIGITS_LENGTH + 1]; //+1 to store the '\0' NUL terminator
    for (int d = 0; d < HEX_DIGITS_LENGTH; d++) {
      hexLength[d] = receivedCommand[1 + (HEX_DIGITS_LENGTH * n) + d]; //this ignores the 1-char command from matlab, and lengths for other nanos
    }
    hexLength[HEX_DIGITS_LENGTH] = '\0';
    lengths[n] = strtol(hexLength, 0, 16); //convert from HEX to DEC
  }
}

/* Write to: currentCableLengths[], lastLengthCommands[] */
void updateInitialLengths(unsigned int newInitialLengths[NUMBER_CONNECTED_NANOS]){
  static unsigned int initialLengths[NUMBER_CONNECTED_NANOS]; //equals 0 at the very beginning. Must be updated before running the robot.

  for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++) {
    //update where the robot thinks it currently is
    currentCableLengths[n] += newInitialLengths[n] - initialLengths[n];
    lastLengthCommands[n] += newInitialLengths[n] - initialLengths[n];

    //update where the robot thinks it started
    initialLengths[n] = newInitialLengths[n];
  }
}

/* Write to: currentCableLengths[], lastLengthCommands[] */
void resetLengths(unsigned int newLengths[NUMBER_CONNECTED_NANOS]){
  for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++) {
    currentCableLengths[n] = newLengths[n];
    lastLengthCommands[n] = newLengths[n];
  }
}

//Convert lengthCommands to crossingCommands and angleCommands
//angleCommand can be in the crossing zone
/* read from: lastLengthCommands[], lastAngleFeedbacks[]
   write to: lastLengthCommands[] */
void computeCrossingAndAngleCommands(unsigned int lengthCommands[NUMBER_CONNECTED_NANOS], int crossingCommands[NUMBER_CONNECTED_NANOS], unsigned int angleCommands[NUMBER_CONNECTED_NANOS]) {
  for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++) {
    //convert length command (absolute) to angle change command (relative), to determine whether crossing is needed
    int lengthChangeCommand = lengthCommands[n] - lastLengthCommands[n];
    int angleChangeCommand = round( (double)lengthChangeCommand / spoolCircumference(MAX_CABLE_LENGTH - lengthCommands[n]) * 3600 ); //convert unit from 0.1mm to 0.1degree

    //determine whether crossing is needed, and generate crossing command and angle command (absolute)
    int crossingCommand = 0;  //0, CLOCKWISE, or ANTICLOCKWISE
    int angleCommand = lastAngleCommands[n] + angleChangeCommand; //in 0.1 degree precision

    if (angleChangeCommand > 0 && angleCommand > 3600 - CROSSING_ZONE_SIZE){  //goes over safe zone limit
      crossingCommand = CLOCKWISE;
    }
    else if (angleChangeCommand < 0 && angleCommand < 0){  //goes under safe zone limit (angleCommand temporarily goes under 0. Will be adjusted soon.)
      crossingCommand = ANTICLOCKWISE;
    }
    else if (angleChangeCommand < 0 && lastAngleCommands[n] > 3600 - CROSSING_ZONE_SIZE){  //still under safe zone limit (angleCommand adjusted to a value between 0-3599)
      crossingCommand = ANTICLOCKWISE;
    }

    //keep angleCommand between 0 to 3599
    if (angleCommand >= 3600){
      angleCommand -= 3600;
    }
    else if (angleCommand < 0){
      angleCommand += 3600;
    }

    //return data
    crossingCommands[n] = crossingCommand;
    angleCommands[n] = angleCommand;

    //update these for comparison with the next command
    lastLengthCommands[n] = lengthCommands[n];
    lastAngleCommands[n] = angleCommand;
  }//for each nano
}

/* write to: Serial1 */
void broadcastCommandsToNanos(int crossingCommands[NUMBER_CONNECTED_NANOS], unsigned int angleCommands[NUMBER_CONNECTED_NANOS]){
  Serial1.print(NANO_ANGLE_COMMAND);
  for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++) {
    //send crossing command
    Serial1.print(crossingCommands[n]);

    //convert angle command from DEC to HEX
    char hexAngleCommand[DIGITS_ANGLE_COMMAND + 1];  //+1 for the NUL terminator \0
    itoa(angleCommands[n], hexAngleCommand, 16);
    int strLength = strlen(hexAngleCommand);     //count the number of characters in the resulting string

    //pad 0 in front (in case hexAngleCommand is shorter than DIGITS_ANGLE_COMMAND)
    for (int zeros = 0; zeros < (DIGITS_ANGLE_COMMAND - strLength); zeros++) {
      Serial1.print('0');
    }

    //print the HEX string
    Serial1.print(hexAngleCommand);
  }

  Serial1.println();
  Serial1.flush();
}

//return 65535 if no feedback from nano
/* read from: serialNano[nanoID] <- ONE nano, not all
   write to: Serial1   */
unsigned int readAngleFromNano(int nanoID){
  //listen to the specified nano (mega can only listen to one software serial at a time)
  serialNano[nanoID].listen();

  //send a request for feedback to nano
  Serial1.println(NANO_QUICK_FEEDBACK_REQUEST + String(nanoID));  
  Serial1.flush();

  //wait for the feedback
  int readCounter = 0;
  int timeOut = 200;                              //TODO: REMOVE MAGIC NUMBER
  while ((serialNano[nanoID].available() == 0) && readCounter < timeOut) { 
    readCounter++;
  }

  //read feedback
  if (serialNano[nanoID].available() > 0) {
    char hexAngleFeedback[DIGITS_ANGLE_FEEDBACK + 1]; //+1 to store the '\0' NUL terminator at the end, to make it a string

    //read feedback
    for (int d = 0; d < DIGITS_ANGLE_FEEDBACK; d++) {
      hexAngleFeedback[d] = serialNano[nanoID].read();
    }
    hexAngleFeedback[DIGITS_ANGLE_FEEDBACK] = '\0';

    //clears the buffer of any other bytes
    while (serialNano[nanoID].available() > 0) {
      serialNano[nanoID].read();
    }

    //convert from HEX to DEC
    return strtol(hexAngleFeedback, 0, 16);
  }

  //what if SerialNano[nanoID] is not available? Return a flag (>3600)
  return 65535;
}

/* read from: currentCableLengths[], lastAngleFeedbacks[]
   write to: currentCableLengths[] */
void computeCurrentCableLengths(unsigned int currentAngles[NUMBER_CONNECTED_NANOS]){
  for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++) {
    //Calculate the change in angle comparing to the last position. (the effect of possible crossing is accounted for.)
    int angleChange = 0;   //in 0.1 degree precision
    if (currentAngles[n] != lastAngleFeedbacks[n]) {
      long angleChangeNoCrossing;
      long angleChangeWithCrossing;

      if (currentAngles[n] > lastAngleFeedbacks[n]) {    //angle reading increased
        angleChangeNoCrossing = (long)currentAngles[n] - lastAngleFeedbacks[n];  //positive
        angleChangeWithCrossing = angleChangeNoCrossing - 3600; //drop below min. -> negative
      }
      else if (currentAngles[n] < lastAngleFeedbacks[n]) { //angle reading decreased
        angleChangeNoCrossing = (long)currentAngles[n] - lastAngleFeedbacks[n];  //negative
        angleChangeWithCrossing = angleChangeNoCrossing + 3600; //go over max. -> positive
      }
      //must cast currentAngles[n] to long, otherwise unsigned int minus a big number will cause underflow.

      //shortest path is (assumed to be) the actual path taken
      if ( abs(angleChangeNoCrossing) < abs(angleChangeWithCrossing) ) {
        angleChange = angleChangeNoCrossing;
      } else {
        angleChange = angleChangeWithCrossing;
      }

      //convert angle change to length change
      int lengthChange = round( (double)angleChange / 3600.0 * spoolCircumference(MAX_CABLE_LENGTH - lastLengthCommands[n])); //convert unit from 0.1degree to 0.1mm

      //add the length change to the current length
      currentCableLengths[n] += lengthChange;
    }
  }
}

/* read from: currentCableLengths[]
   write to: Serial */
void sendCurrentCableLengths() {
  Serial.print(CASPR_FEEDBACK);
  for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++) {
    //convert feedback from DEC to HEX
    char hexFeedback[HEX_DIGITS_LENGTH + 1]; //+1 for the null terminator '\0' at the end
    itoa(currentCableLengths[n], hexFeedback, 16);//convert to HEX
    int strLength = strlen(hexFeedback);     //count the number of characters in the resulting string

    //pad 0 in front (in case hexFeedback is shorter than HEX_DIGITS_LENGTH)
    for (int zeros = 0; zeros < (HEX_DIGITS_LENGTH - strLength); zeros++) {
      Serial.print('0');
    }

    //print the HEX string
    Serial.print(hexFeedback);
  }
  Serial.println();
  Serial.flush();
}

/* return the spool circumference (increases as more cable wraps onto the spool) */
double spoolCircumference(unsigned int lengthOnSpool){
  //y = mx + b
  //spool circumference = m * (estimated number of loops on the spool) + b
  return VARYING_SPOOL_CIRCUMFERENCE_M * ((double)lengthOnSpool / (double)AVERAGE_SPOOL_CIRCUMFERENCE) + VARYING_SPOOL_CIRCUMFERENCE_B;
}

