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

#define NUMBER_CONNECTED_NANOS 1                        //TODO: This should be sent by MATLAB in the future
#define SPOOL_CIRCUMFERENCE 1355 //in 0.1mm precision.  //TODO: make it dynamic

#define CROSSING_ZONE_SIZE 70   //in 0.1degree

#define BAUD_RATE_NANO 74880
#define BAUD_RATE_CASPR 74880

#define HEX_DIGITS_LENGTH 4
#define DIGITS_ANGLE_COMMAND 3
#define DIGITS_ANGLE_FEEDBACK 3
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

#define CLOCKWISE 1
#define ANTICLOCKWISE 2

boolean systemOn = false;      //no servo movement or feedback when off
boolean motorsEnabled = false;  //no servo movement when off


int crossingCommand[NUMBER_CONNECTED_NANOS];
boolean crossingFeedback[NUMBER_CONNECTED_NANOS];

//unsigned int has 2 bytes, range 0 - 65535.
//In 0.1 mm precision. This represents 6553.5mm or ~6.5m
unsigned int currentCableLength[NUMBER_CONNECTED_NANOS];  //i.e. where the cable robot think it is
unsigned int lastLengthCommand[NUMBER_CONNECTED_NANOS];   //help calculate angle change command

//TODO: populate this before using in requestNanoFeedback()
//TODO: make this static?
//help decide whether crossing has happened
unsigned int lastAngleFeedback[NUMBER_CONNECTED_NANOS]; //in 0.1 degree precision


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
   SoftwareSerial for receiving from individual Nanos */
void setup() {
  Serial.begin(BAUD_RATE_CASPR);  //USB
  Serial1.begin(BAUD_RATE_NANO); //for broadcasting to nano

  for (int i = 0; i < NUMBER_CONNECTED_NANOS; i++) {
    serialNano[i].begin(BAUD_RATE_NANO); //for receiving from nano

    //Arbitrarily assume all cables to be 3.2768m long at the start.
    //They should be updated by updateInitialLengths() before use.        TODO: still true?
    unsigned int lastLengthCommand[i] = 32768;
    unsigned int currentCableLength[i] = 32768;
  }

}

/* Main loop acts to interface with MATLAB/CASPRROS (asynchronously) and nano at 20Hz */
void loop() {
  if (Serial.available() > 0) {  //USB
    String receivedCommand = Serial.readStringUntil('\n');  //TODO: make it local?

    switch (receivedCommand[0]) {
      case CASPR_ACKNOWLEDGE:                 //a: handshake with matlab
        { 
          if (receivedCommand.length() == 1) {
            systemOn = false;
            Serial.println(CASPR_ACKNOWLEDGE);
          }
        }
        break;
      case CASPR_START:                       //s: start system
        {
          if (receivedCommand.length() == 1) {
            systemOn = true;
          }
        }
        break;
      case CASPR_END:                         //e: end (turn off) system
        {
          if (receivedCommand.length() == 1) {
            systemOn = false;
            motorsEnabled = false;
          }
        }
        break;
      case CASPR_INITIAL:                    //i: update initial lengths
        {
          updateInitialLengths(receivedCommand);
          updateCableLengths();
          sendLengthFeedback();
          motorsEnabled = true;    //CLARIFY: why?
        }
        break;
      case CASPR_LENGTH_CMD:                //l: Gather feedback from nano, send to matlab/CASPRROS, then move cables to length specified.
        { 
          motorsEnabled = true;    //CLARIFY: why?
          if (systemOn) {
            updateCableLengths();
            sendLengthFeedback();

            if (motorsEnabled) {
              translateCommandAndSendToNano();
            }
          }
        }
        break;
      case CASPR_RESET:                   //r: reset lengths (make the cable robot think it is at a specific location)
        {
          resetLengths(receivedCommand);
          motorsEnabled = true;    //CLARIFY: why?
        }
        break;
      case CASPR_HOLD:                    //h: tighten all cables
        {
          //tighten all cables, while the robot think it hasn't move anywhere
          //tighten by percentage?
          //TODO: are all array needed populated?


          //get currentCableLength
          //-x and set as new length command
          //send command to nano
        }

    } //switch
  } //if

}

/* Update the initial lengths, i.e. length of each cable when the cable robot starts.
   Will update currentCableLength[] and lastLengthCommand[].
   -- Why? --
   The cable robot moves depending on where it think it is (currentCableLength[], lastLengthCommand[]),
   and that depends on where it think it started (initialLength[]).
   If during movement/calibration, we realize that its movement is off because it actually started somewhere else,
   we can use this function to update the initial lengths, and also correct its perceived current location on the fly. */
void updateInitialLengths(const String &receivedCommand) {
  char hexNewInitialLength[4];
  unsigned int newInitialLength;
  static int initialLength[NUMBER_CONNECTED_NANOS];  //0 at the start. Must be updated before running the robot.

  for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++) {
    //read to hexNewInitLength[]
    for (int d = 0; d < HEX_DIGITS_LENGTH; d++) {
      hexNewInitLength[d] = receivedCommand[1 + (n * HEX_DIGITS_LENGTH) + d];  //this ignores the 1-char command and the lengths for other nanos
    }

    //convert from HEX to DEC
    newInitLength = strtol(hexNewInitLength, 0, 16);

    //update where the robot think it currently is
    currentCableLength[n] += newInitLength - initialLength[n];
    lastLengthCommand[n] += newInitLength - initialLength[n];

    //update where the robot thinks it started
    initialLength[n] = newInitLength;

    //pwmLastFeedback[n] = pwmFeedback[n]; //TODO: change to lastAngleFeedback? //TODO: can i remove this line??
  }
}

/* Make the cable robot think it is at a specific location.
   Will update currentCableLength[] and lastLengthCommand[]. */
void resetLengths(const String &receivedCommand) {
  char hexNewLength[4];
  unsigned int newLength;
  for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++) {
    for (int d = 0; d < HEX_DIGITS_LENGTH; d++) {
      hexNewLength[d] = receivedCommand[1 + (n * HEX_DIGITS_LENGTH) + d];  //this ignores the 1-char command and the lengths for other nanos
    }
    newLength = strtol(hexNewLength, 0, 16);
    currentCableLength[n] = newLength;
    lastLengthCommand[n] = newLength;
  }
}

/* Read feedback from ALL nanos, adjust for crossing, and update currentCableLength[].
   Will also update lastAngleFeedback[].  */
void updateCableLengths() {
  for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++) {
    //read angle feedback from nano
    unsigned int angle = readNanoFeedback(n); //nano should return angle in 0.1 degree precision

    //Calculate the change in angle comparing to the last position.
    //Adjust for crossing (i.e. the sudden jump in angle feedback when the value goes below min./above max.)
    int angleChange = 0;   //in 0.1 degree precision
    if (angle != lastAngleFeedback[n]) {
      long angleChangeNoCrossing;
      long angleChangeWithCrossing;

      if (angle > lastAngleFeedback[n]) {    //angle reading increased
        angleChangeNoCrossing = angle - lastAngleFeedback[n];  //positive
        angleChangeWithCrossing = angleChangeNoCrossing - 360; //drop below min. -> negative
      }
      else if (angle < lastAngleFeedback[n]) { //angle reading decreased
        angleChangeNoCrossing = angle - lastAngleFeedback[n];  //negative
        angleChangeWithCrossing = angleChangeNoCrossing + 360; //go over max. -> positive
      }

      //shortest path is (assumed to be) the actual path taken
      if ( abs(angleChangeNoCrossing) < abs(angleChangeWithCrossing) ) {
        angleChange = angleChangeNoCrossing;
      } else {
        angleChange = angleChangeWithCrossing;
      }
    }
    //save angle for comparison next time
    lastAngleFeedback[n] = angle;

    //convert angle change to length change
    int lengthChange = round( (double)angleChange / 3600.0 * SPOOL_CIRCUMFERENCE ); //convert unit from 0.1degree to 0.1mm

    //add the length change to the current length
    currentCableLength[n] += lengthChange;
  }
}

/* Request and read position feedback from ONE nano (the specified one), and return the feedback angle. */
unsigned int readNanoFeedback(int nanoID) {
  //listen to the specified nano (mega can only listen to one software serial at a time)
  serialNano[nanoID].listen();

  //requests feedback from nano
  Serial1.println(NANO_FEEDBACK + String(nanoID));
  Serial1.flush(); //waits for the sending of Serial to be complete before moving on

  //wait for the feedback
  int readCounter = 0;
  while ((serialNano[nanoID].available() == 0) && readCounter < 200) { //TODO: 200 is a magic number. CLARIFY
    readCounter++;
  }

  //read feedback
  if (serialNano[nanoID].available() > 0) {
    char hexFeedback[DIGITS_ANGLE_FEEDBACK + 1]; //+1 to store the '\0' NUL terminator at the end, to make it a string

    //read feedback
    for (int d = 0; d < DIGITS_ANGLE_FEEDBACK; d++) {
      hexFeedback[d] = serialNano[nanoID].read();
    }
    hexFeedback[DIGITS_ANGLE_FEEDBACK] = '\0';

    //clears the buffer of any other bytes
    while (serialNano[nanoID].available() > 0) {
      serialNano[nanoID].read();
    }

    //convert from HEX to DEC
    return strtol(hexFeedback, 0, 16);
  }
}

/* Convert currentCableLength[] to HEX and send to matlab/CASPRROS as feedback. */
void sendLengthFeedback() {
  Serial.print(CASPR_FEEDBACK);
  for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++) {
    //convert feedback from DEC to HEX
    char hexFeedback[HEX_DIGITS_LENGTH + 1]; //+1 for the null terminator '\0' at the end
    itoa(lengthFeedback[n], hexFeedback, 16);//convert to HEX
    int strLength = strlen(hexFeedback);     //count the number of characters in the resulting string

    //pad 0 in front
    for (int z = 0; z < (HEX_DIGITS_LENGTH - strLength); z++) {
      Serial.print('0');
    }

    //print the HEX string
    Serial.println(hexFeedback);
    Serial.flush();
  }
}

/* Translate length commands from the received matlab string to crossing commands and angle commands, and broadcast to nanos.
   Requires an updated lastLengthCommand[] to calculate 
   Requires an updated lastAngleFeedback[] to correctly decide whether crossing is need.
   Will also update lastLengthCommand[]. */
void translateCommandAndSendToNano() {
  //if (receivedCommand[0] == CASPR_LENGTH_CMD) {  //TODO: how will it be read twice?

  for (int n = 0; n < NUMBER_CONNECTED_NANOS; n++) { //for each nano

    //extract length command for this nano
    char hexCommand[HEX_DIGITS_LENGTH + 1]; //+1 to store the '\0' NUL terminator
    for (int d = 0; d < HEX_DIGITS_LENGTH; d++) {
      hexCommand[d] = receivedCommand[1 + (HEX_DIGITS_LENGTH * n) + d]; //this ignores the command from matlab, and length command for other nanos
    }
    hexCommand[HEX_DIGITS_LENGTH] = '\0';
    unsigned int lengthCommand = strtol(hexCommand, 0, 16); //convert from HEX to DEC

    ///////////////////////////////////// lengthCommand -> crossingCommand, angleCommand ////////////////////////////////

    //convert length command to angle change command
    int lengthChangeCommand = lengthCommand - lastLengthCommand[n];
    int angleChangeCommand = round( (double)lengthChangeCommand / (double)SPOOL_CIRCUMFERENCE * 3600 ); //convert unit from 0.1mm to 0.1degree

    //convert angle change command to crossing command and angle command (decide whether crossing is needed)
    int crossingCommand = 0;   //either 0, CLOCKWISE, or ANTICLOCKWISE
    int angleCommand = lastAngleFeedback[n] + angleChangeCommand; //in 0.1 degree precision
    //=== went under min. angle while spinning anticlockwise ==========================================================
    if (angleCommand < 0) {
      if (angleCommand < - CROSSING_ZONE_SIZE) {         //went past crossing zone
        //cross to the other side
        crossingCommand = ANTICLOCKWISE;
        angleCommand += 3600;
      }
      else if (angleCommand < - CROSSING_ZONE_SIZE / 2) { //in the crossing zone, went past tipping point
        //cross and stay at the far edge of the crossing zone
        crossingCommand = ANTICLOCKWISE;
        angleCommand = 3600 - CROSSING_ZONE_SIZE;
      }
      else {                                             //in the crossing zone, didn't went past the tipping point
        //stay on the near edge of the crossing zone
        crossingCommand = 0;
        angleCommand = 0;
      }
    }
    //=== went over max. angle while spinning clockwise ==========================================================
    else if (angleCommand > (3600 - CROSSING_ZONE_SIZE) ) {
      if (angleCommand > 3600) {                                  //went past crossing zone
        //cross to the other side
        crossingCommand = CLOCKWISE;
        angleCommand -= 3600;
      }
      else if (angleCommand > (3600 - CROSSING_ZONE_SIZE / 2) ) { //in the crossing zone, went past tipping point
        //cross and stay at the far edge of the crossing zone
        crossingCommand = CLOCKWISE;
        angleCommand = 0;
      }
      else {                                                      //in the crossing zone, didn't went past the tipping point
        //stay on the near edge of the crossing zone
        crossingCommand = 0;
        angleCommand = 3600 - CROSSING_ZONE_SIZE;
      }
    }



    /////////////////////////////send crossing command and angle command//////////////////////////////////////////////////

    //send crossing command
    Serial1.print(crossingCommand);

    //convert angle command to HEX and send
    char hexAngleCommand[DIGITS_ANGLE_COMMAND + 1];  //+1 for the NUL terminator \0
    itoa(angleCommand, hexAngleCommand, 16);
    Serial1.print(hexAngleCommand);

    //update lastLengthCommand[] for comparison with the next command
    lastLengthCommand[n] = lengthCommand;

  }//for each nano

  Serial1.println();
  Serial1.flush();


  //  receivedCommand[0] = '\0'; //resets array, so it is not read twice          //TODO: how will it be read twice?
  //  enableMotors = 1;                                                           //TODO: is it necessary?
}


