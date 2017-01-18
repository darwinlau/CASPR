#include <string.h>
#include <math.h>
#include "servo_properties/servo_08.h"   //servo-specific properties (e.g. the range of pwm command it can execute) is stored here

#define MOTOR_PIN 2
#define BAUD_RATE 74880

#define LENGTH_ANGLE_COMMAND 4
#define DIGITS_ANGLE_FEEDBACK 3

#define RECEIVE_ANGLE_CMD 'c'
#define RECEIVE_FEEDBACK_REQUEST 'f'

#define CLOCKWISE 1
#define ANTICLOCKWISE 2

#define CROSSING_ZONE_SIZE 70   //in 0.1degree

int avgPWMFeedback; //stores the position of the servo.
int lastPWMFeedback = 0; //This variable can be use in a pinch when new feedback is faulty
int lastCrossingCommand = 0; //for use in checkCrossingStatus()

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
      case RECEIVE_FEEDBACK_REQUEST:     //f
        {
          char id = strReceived[1];
          if (id == NANO_ID + '0') {//if the request is for this nano
            int angleFeedback = mapping(avgPWMFeedback, FEEDBACK_PWM_MIN, FEEDBACK_PWM_MAX, 0, (3600 - CROSSING_ZONE_SIZE));
            sendFeedback(angleFeedback);
          }
        }
        break;
      
      case RECEIVE_ANGLE_CMD:              //c
        executeAngleCommand(strReceived);//pass a pointer to strReceived
        avgPWMFeedback = readAvgFeedback(4); //prepare for the next time when mega send a feedback request. Average 4 measurements.
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
  //extract new crossing command
  int newCrossingCommand = extractCrossingCommand(strReceived);

  if (newCrossingCommand > 0){                   //if there's new crossing command, execute it
     cross(newCrossingCommand);
     lastCrossingCommand = newCrossingCommand;
  }
  else {
    int crossingStatus = checkCrossingStatus();
    if (crossingStatus > 0){                     //else, finish the previous crossing
      cross(crossingStatus);
    }
    else {                                       //else, extract new position command and execute it
      int newAngleCommand = extractAngleCommand(strReceived);
      int newPWMCommand = mapping(newAngleCommand, 0, (3600 - CROSSING_ZONE_SIZE), COMMAND_PWM_MIN, COMMAND_PWM_MAX);
      writePulseToServo(newPWMCommand);
      writePulseToServo(newPWMCommand);
      //FIX: send counter? stop sending the same thing after 3 times, to stop vibration caused by noise
    }
  }
}

/* Convert feedback from DEC to HEX chars, and send to mega via Serial. */
void sendFeedback(int feedback){
  //convert from DEC to HEX
  char hexFeedback[DIGITS_ANGLE_FEEDBACK + 1]; //+1 for the null terminator \0
  itoa(feedback, hexFeedback, 16); 
  //send HEX string
  Serial.println(hexFeedback);
  Serial.flush();
}

/* read feedback pulse width (in microsecond) from servo # times and return the average value. */
int readAvgFeedback(int numOfReadings){
  double sumOfFeedback = 0;
  for (int i = 0; i < numOfReadings; i++){
    sumOfFeedback += readFeedbackFromServo();
    delay(1);
  }
  int average = lround(sumOfFeedback / numOfReadings);  //average the feedback. Round result from double to long
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
void cross(int crossingCommand){
  switch (crossingCommand){
    case CLOCKWISE:
      writePulseToServo(CLOCKWISE_PWM_MIN); //min speed
      writePulseToServo(CLOCKWISE_PWM_MIN); //write twice to make sure it moves. FIX?
      break;
    case ANTICLOCKWISE:
      writePulseToServo(ANTICLOCKWISE_PWM_MIN); //min speed
      writePulseToServo(ANTICLOCKWISE_PWM_MIN);
      break;
  }
}

/* Return the servo's crossing status.
   Return 0 if it has exited the crossing zone, or CLOCKWISE/ANTICLOCKWISE if it's still crossing. */
int checkCrossingStatus(){
  int servoPosition = readAvgFeedback(4);
  switch (lastCrossingCommand){
    case CLOCKWISE:  //if it was crossing clockwise
      if ((servoPosition > (FEEDBACK_PWM_MIN)) && (servoPosition <= FEEDBACK_PWM_MIDDLE)) {
        return 0;
      } else {
        return CLOCKWISE;
      }
      break;
    case ANTICLOCKWISE: //if it was crossing anticlockwise
      if ((servoPosition < (FEEDBACK_PWM_MAX)) && (servoPosition > FEEDBACK_PWM_MIDDLE )) {
        return 0;
      } else {
        return ANTICLOCKWISE;
      }
      break;
  } //switch

  //if all else failed
  return 0;
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

