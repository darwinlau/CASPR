#include <string.h>
#include <math.h>
#include "servo_properties/servo_08.h"   //servo-specific properties (e.g. the range of pwm command it can execute) is stored here

#define MOTOR_PIN 2
#define BAUD_RATE 74880

#define LENGTH_ANGLE_COMMAND 4
#define DIGITS_ANGLE_FEEDBACK 3

#define RECEIVE_ANGLE_CMD 'c'
#define RECEIVE_QUICK_FEEDBACK_REQUEST 'f'
#define RECEIVE_UPDATE_FEEDBACK 'u'
#define RECEIVE_FINISH_TRAJECTORY 'e'

#define CLOCKWISE 1
#define ANTICLOCKWISE 2

#define CROSSING_ZONE_SIZE 70   //(in 0.1degree)
#define REASONABLE_VARIATION_IN_PWM_FEEDBACKS 35 //(in microsecond) used to check whether feedback varies too much = faulty
//the normal variation is around 7. Abnormal variation (e.g. during crossing) could be 500 or higher

int avgPWMFeedback; //stores the position of the servo.
int lastPWMFeedback = 0; //This variable can be use in a pinch when new feedback is faulty
int lastCrossingAction = 0; //for the doneCrossing()? check, and for continuing of crossing if it's not done

void setup() {
  Serial.begin(BAUD_RATE);

  //read servo position to lastPWMFeedback. This variable can be use in a pinch when new readings are faulty
  while (lastPWMFeedback == 0) {
    lastPWMFeedback = readFeedbackFromServo();
  }
  
  //read average feedback -- prepare itself to answer a request from mega
  avgPWMFeedback = readAvgFeedback(4); //average 4 measurements
}

/* Read command from mega and decide what to do depending on the 1-character command (first char of the command string).  */
void loop() {
  if (Serial.available() > 0) {
    String strReceived = Serial.readStringUntil('\n');
    char commandReceived = strReceived[0];

    switch (commandReceived){    
      case RECEIVE_ANGLE_CMD:                 //c
        executeAngleCommand(strReceived);//pass a pointer to strReceived to the function
        avgPWMFeedback = readAvgFeedback(4); //prepare for the next time when mega send a feedback request. Average 4 measurements.
        break;
      
      case RECEIVE_QUICK_FEEDBACK_REQUEST:    //f
        {
          char id = strReceived[1];
          if (id == NANO_ID + '0') {//if the request is for this nano
            //send an old feeedback value (updated in the previous RECEIVE_ANGLE_CMD)
            int angleFeedback = mapping(avgPWMFeedback, FEEDBACK_PWM_MIN, FEEDBACK_PWM_MAX, (CROSSING_ZONE_SIZE / 2), (3600 - CROSSING_ZONE_SIZE / 2) );
            sendFeedback(angleFeedback);
          }
        }
        break;

      case RECEIVE_UPDATE_FEEDBACK:           //u (nano need to update its avgPWMFeedback after idling between trajectories)
        avgPWMFeedback = readAvgFeedback(4); 
        break;

      case RECEIVE_FINISH_TRAJECTORY:         //e
        while (!doneCrossing){
          //wait
        }
        //stay where it is
        avgPWMFeedback = readAvgFeedback(4); 
        int currentLocationPW = mapping(avgPWMFeedback, FEEDBACK_PWM_MIN, FEEDBACK_PWM_MAX, COMMAND_PWM_MIN, COMMAND_PWM_MAX);
        writePulseToServo(currentLocationPW);
        writePulseToServo(currentLocationPW);
        //start fresh for next trajectory
        lastCrossingAction = 0;
        break;
    }//switch
  }
  // else {
  //   avgPWMFeedback = readAvgFeedback(); //if nano has some spare time, read servo feedback
  //   //FIX: will this interfere with listening to command from mega? How lond does readAvgFeedback() take? 12ms?
  // }
}

/* execute angle command from mega: move to the specified position with position mode, or cross the "crossing zone" with velocity mode */
void executeAngleCommand(const String &strReceived){
  static int lastCrossingCommand = 0;
  
  //extract new crossing command
  int newCrossingCommand = extractCrossingCommand(strReceived);

  //lastCrossingCommand -> newCrossingCommand scenarios:
  // 0 -> 1, clockwise
  // 0 -> 2, anticlockwise
  // 1 -> 2, anticlockwise
  // 2 -> 1, clockwise
  // 1 -> 1, (x: not done crossing) clockwise, or (v: done) stay
  // 2 -> 2, (x) anticlockwise or (v) stay 
  // 1 -> 0, (x) clockwise, or (v) goto
  // 2 -> 0, (x) anticlockwise, or (v) goto
  // 0 -> 0, (x) clockwise/anticlockwise, or (v) goto

  //if mega command says:
  if (lastCrossingCommand == 0 && newCrossingCommand > 0){ //"start new crossing"
    cross(newCrossingCommand);
  }
  else if (lastCrossingCommand > 0 && newCrossingCommand > 0 && newCrossingCommand != lastCrossingCommand){  //"change crossing direction"
    cross(newCrossingCommand);
  }
  else if (lastCrossingCommand > 0 && newCrossingCommand == lastCrossingCommand){ //"continue crossing"
    avgPWMFeedback = readAvgFeedback(4); 
    if (doneCrossing()){
      lastCrossingAction = 0;
      //stay where it is
      int currentLocationPW = mapping(avgPWMFeedback, FEEDBACK_PWM_MIN, FEEDBACK_PWM_MAX, COMMAND_PWM_MIN, COMMAND_PWM_MAX);
      writePulseToServo(currentLocationPW);
      writePulseToServo(currentLocationPW);
    } 
    else {
      cross(newCrossingCommand);
    }
  }
  else if (lastCrossingCommand > 0 && newCrossingCommand == 0){ //"stop crossing; follow angle command"
    avgPWMFeedback = readAvgFeedback(4); 
    if (doneCrossing()){
      lastCrossingAction = 0;
      goToPosition(strReceived);
    }
    else {
      cross(lastCrossingCommand);
    }
  }
  else if (lastCrossingCommand == 0 && newCrossingCommand == 0){ //"just follow angle command"
    avgPWMFeedback = readAvgFeedback(4); 
    if (doneCrossing()){
      lastCrossingAction = 0;
      goToPosition(strReceived);
    }
    else {
      cross(-1);  //repeating last crossing command
    }
  }

  lastCrossingCommand = newCrossingCommand;

  //FIX: send counter? stop sending the same thing after 3 times, to stop vibration caused by noise
}

/* Move servo to position specified in strReceived. Faulty value is corrected. */
void goToPosition(const String &strReceived){
  int newAngleCommand = extractAngleCommand(strReceived);
  int newPWMCommand = mapping(newAngleCommand, (CROSSING_ZONE_SIZE / 2), (3600 - CROSSING_ZONE_SIZE / 2), COMMAND_PWM_MIN, COMMAND_PWM_MAX);

  if (newPWMCommand > COMMAND_PWM_MAX){
    newPWMCommand = COMMAND_PWM_MAX;
  } else if (newPWMCommand < COMMAND_PWM_MIN){
    newPWMCommand = COMMAND_PWM_MIN;
  }

  writePulseToServo(newPWMCommand);
  writePulseToServo(newPWMCommand);
}

/* Convert feedback from DEC to HEX chars, and send to mega via Serial. */
void sendFeedback(int feedback){
  //convert from DEC to HEX
  char hexFeedback[DIGITS_ANGLE_FEEDBACK + 1]; //+1 for the null terminator \0
  itoa(feedback, hexFeedback, 16); 
  int strLength = strlen(hexFeedback);     //count the number of characters in the resulting string

  //pad 0 in front (in case hexFeedback is shorter than DIGITS_ANGLE_FEEDBACK)
  for (int zeros = 0; zeros < (DIGITS_ANGLE_FEEDBACK - strLength); zeros++) {
    Serial.print('0');
  }
  
  //send HEX string
  Serial.println(hexFeedback);
  Serial.flush();
}

/* read feedback pulse width (in microsecond) from servo # times and return the average value. */
int readAvgFeedback(int numOfReadings){
  static int lastValidAvgFeedback; //needed in crossing zone where feedback is buggy

  double sumOfFeedback = 0;
  int feedback;
  int maxFeedback = 0;
  int minFeedback = 32767;
  for (int i = 0; i < numOfReadings; i++){
    feedback = readFeedbackFromServo();

    //error checking and compensation
    if (feedback > maxFeedback){
      maxFeedback = feedback;
    }
    if (feedback < minFeedback){
      minFeedback = feedback;
    }
    if (maxFeedback - minFeedback > REASONABLE_VARIATION_IN_PWM_FEEDBACKS){
      return lastValidAvgFeedback;
    }

    sumOfFeedback += feedback;
    delay(1);
  }

  int average = lround(sumOfFeedback / numOfReadings);  //average the feedback. Round result from double to long
  lastValidAvgFeedback = average; //backup in case feedback is faulty in the future
  return average;
}

/* Request the servo for feedback, and return the feedback pulse width (in microsecond). 
   Faulty value will be corrected using lastPWMFeedback, FEEDBACK_PWM_MIN or FEEDBACK_PWM_MAX. */
int readFeedbackFromServo(){
  //send request for feedback
  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(MOTOR_PIN, LOW);

  //read feedback
  int feedback = pulseIn(MOTOR_PIN, HIGH, 2000); //measure the duration of the returning HIGH pulse (in microseconds), or time-out
  
  //fix faulty measurements
  if ((feedback < 300) || (feedback > 1700)){ //if result is terribly off. (Typical feedback value is around 500-1500)
    feedback = lastPWMFeedback; //use an old value
  }
  else if (feedback < FEEDBACK_PWM_MIN) { //if result is slightly off
    feedback = FEEDBACK_PWM_MIN;
  }
  else if (feedback > FEEDBACK_PWM_MAX) {
    feedback = FEEDBACK_PWM_MAX;
  }
  //the result is always between FEEDBACK_PWM_MIN and FEEDBACK_PWM_MAX.
  //there is no bound to how outdated the result is though.

  lastPWMFeedback = feedback; //store feedback in case the next feedback is terribly off
  return feedback;
}

/* Extract crossing command from the received string. */
int extractCrossingCommand(const String &strReceived){
  return strReceived[1 + (LENGTH_ANGLE_COMMAND * NANO_ID)] - '0';
}

/* Send velocity command to servo to cross the "crossing zone" 
   where position command and feedback doesn't work. */
void cross(int command){
  //repeat the last crossing action if -1 is received
  if (command == -1){
    command = lastCrossingAction;
  }

  //execute crossing command
  switch (command){
    case CLOCKWISE:
      writePulseToServo(CLOCKWISE_PWM_MIN); //min speed
      writePulseToServo(CLOCKWISE_PWM_MIN); //write twice to make sure it moves. FIX?
      break;
    case ANTICLOCKWISE:
      writePulseToServo(ANTICLOCKWISE_PWM_MIN); //min speed
      writePulseToServo(ANTICLOCKWISE_PWM_MIN);
      break;
  }

  lastCrossingAction = command;
}

/* Return the servo's crossing status. 
   Require an updated avgFeedback. */
bool doneCrossing(){
  //not need to check if there's no crossing in the first place/ crossing was completed long ago
  if (lastCrossingAction == 0){
    return true;
  }

  //check crossing
  switch (lastCrossingAction){
    case CLOCKWISE:  //if it was crossing clockwise
      if ((avgPWMFeedback > (FEEDBACK_PWM_MIN)) && (avgPWMFeedback <= FEEDBACK_PWM_MIDDLE)) {
        return true;
      } else {
        return false;
      }
      break;
    case ANTICLOCKWISE: //if it was crossing anticlockwise
      if ((avgPWMFeedback < (FEEDBACK_PWM_MAX)) && (avgPWMFeedback > FEEDBACK_PWM_MIDDLE )) {
        return true;
      } else {
        return false;
      }
      break;
  } //switch

  //if wasn't crossing at all, or if all else failed
  return true;
}

/* Send one PWM pulse to servo.
   pulseWith is in microsecond and should be non-negative number under 3000.
   This function takes 3ms to complete. */
void writePulseToServo(int pulseWidth) {
  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(MOTOR_PIN, LOW);
  delayMicroseconds(3000 - pulseWidth);
}

/* Extract angle command from the received string and convert it from HEX to DEC. */
int extractAngleCommand(const String &strReceived){
  //extract the HEX string for THIS servo
  char hexCommand[LENGTH_ANGLE_COMMAND]; //array size is (LENGTH_ANGLE_COMMAND - 1 + 1): -1 for crossingCommand and +1 for null terminator \0
  const int indexOfCommand = 1 + (LENGTH_ANGLE_COMMAND * NANO_ID) + 1;  //this skips the mega command, command for other servos, and the crossingCommand
  for (int i = 0; i < LENGTH_ANGLE_COMMAND - 1; i++) {
    hexCommand[i] = strReceived[indexOfCommand + i];
  }

  //convert from HEX to DEC
  int intCommand = strtol(hexCommand, 0, 16); 
  
  return intCommand;
}

/* Re-maps a number from one range to another. Unlike the arduino map(), it rounds the result instead of truncating it. */
long mapping(long x, long in_min, long in_max, long out_min, long out_max){
  return round( (double)(x - in_min) * (double)(out_max - out_min) / (double)(in_max - in_min) + out_min );
}

