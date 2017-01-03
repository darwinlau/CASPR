#include <string.h>
#include <Wire.h>
#include <math.h>
#include "servo_properties/servo_08.h"   //servo-specific properties (e.g. the range of pwm command it can execute) is stored here

#define MOTOR_PIN 2
#define BAUD_RATE 74880
//#define DELTA 0 // freezing regions at crossing area  //TODO:remove this?

#define LENGTH_PWM_COMMAND 4
#define DIGITS_PWM_FEEDBACK 3

#define RECEIVE_PWM_CMD 'p'
#define RECEIVE_FEEDBACK_REQUEST 'f'
#define RECEIVE_TEST_REQUEST 't'
#define RECEIVE_TESTDRIVE_REQUEST 'z'

#define CLOCKWISE 1
#define ANTICLOCKWISE 2

/////////////////////////// COMMUNICATION ///////////////////////// //

String strReceived;
char commandReceived;
char pwmFeedback[DIGITS_PWM_FEEDBACK];


/////////////////////////// FEEDBACK VARIABLES ///////////////////////////

int servoPWM; // servo position as 'pwm' value (see _feedback for scale above)
int lastPWMServo = 0;
int lastloopAveragePWM = 0;
int loopAveragePWM = 0;
int averageSpeed = 0;

/////////////////////////// COMMAND AND MOTOR CONTROL ///////////////////////////

int pwmCommand = 0;
int lastPWMCommand = 0;

int crossingCommand = 0;
int stillCrossing = 0;

int sendCounter = 0;

/////////////////////////// DEBUGGING AND TIMING VARIABLES //////////////

unsigned long int t_ref;
double delayTime;
boolean cw = 1;
int pwmTestrun = 700;

//int speedCounter = 0;
//int speedStore[SPEEDAVG];

/////////////////////////// FUNCTION PRECALLING ///////////////////////////

int readFeedbackFromServo();
void readAvgFeedback();
void crossing();
void updateCrossingStatus();
void ctrl_motor(int pwmMotor);
void writePulseToServo(int pulseWidth);
void sendFeedback();
void testdrive();

void setup() {
  Serial.begin(BAUD_RATE);
  while (servoPWM == 0) {
    readAvgFeedback();
  }
  lastPWMServo = servoPWM;
}

void loop() {
  readSerial();
}

/* Read serial from mega and decide what to do depending on the 1-character command.  */
void readSerial() { //receive characterizing prefix (+ length in 2 digit Hex, with manipulation of first bit for sign)
  if (Serial.available() > 0) {
    strReceived = Serial.readStringUntil('\n');
    commandReceived = strReceived[0];

    switch (commandReceived){
      case RECEIVE_PWM_CMD: {          //p
        //read command
        int newCrossingCommand = readCrossingCommand();
        
        if (newCrossingCommand > 0){                   //if there's new crossing command, execute it
          crossingCommand = newCrossingCommand;
        }
        else {
          int crossingStatus = checkCrossingStatus();
          if (crossingStatus > 0){                    //else, finish the previous crossing
            crossingCommand = crossingStatus;
          }
          else {                                      //else, read new position command
            readPWMCommand();//update int pwmCommand
          }
        }
        ctrl_motor(pwmCommand);
        
//        stillCrossing = 0;                 //stillCrossing is 0 if servo is not crossing; 1 if crossing clockwise, 2 if anticlockwise
//        readAvgFeedback();
//      
//        if (crossingCommand > 0) {       //if servo is in the crossing process
//          updateCrossingStatus();        //decide whether we can quit crossing or not
//        }
//        delay(5);
//        readPWMCommand();                  //update crossingCommand and pwmCommand
//        if (stillCrossing > 0 && crossingCommand == 0) { //if stillCrossing > 0, the crossing has not been quit, but cross has been reassigned by the new incomming command
//          crossingCommand = stillCrossing;               //CLARIFY: if crossing is not done yet, ignore the cmd from mega, continue our crossing?
//        }
//        ctrl_motor(pwmCommand);

        break;
      }
      case RECEIVE_FEEDBACK_REQUEST: {  //f
        char id = strReceived[1];
        if (id == NANO_ID + '0') {  //if the request is for this nano
          sendFeedback();
        }
        break;
      }
      case RECEIVE_TEST_REQUEST: {      //t
        Serial.print('c');
        Serial.println('c');
        Serial.flush();
        delayMicroseconds(10);
        break;
      }
      case RECEIVE_TESTDRIVE_REQUEST: { //z
        testdrive();
        break;
      }
    }//switch
  }//if Serial.available()
  
  // ADD CALIBRATION LATER
}

/* read feedback pulse width (in microsecond) from servo 4 times and save the average value to loopAveragePWM. 
   Will update servoPWM in the process. */
void readAvgFeedback() {
  readFeedbackFromServo();
  loopAveragePWM = servoPWM;
  for (int i = 0; i < 3; i++) {
    delay(3);
    readFeedbackFromServo();
    loopAveragePWM += servoPWM;
  }
  loopAveragePWM = (int)(loopAveragePWM / 4.0);
}


/* Extract crossing command from the received string. */
int readCrossingCommand() {
  return strReceived[1 + (LENGTH_PWM_COMMAND * NANO_ID)] - '0';
}

/* Extract pwm command from the received string, convert it from HEX to DEC,
   and update pwmCommand. */
void readPWMCommand() {
  char pwmReceived[LENGTH_PWM_COMMAND];
  for (int i = 0; i < LENGTH_PWM_COMMAND - 1; i++) {
    pwmReceived[i] = strReceived[1 + i + 1 + (LENGTH_PWM_COMMAND * NANO_ID)]; //+1 for prefix, another +1 to omit the crossing boolean
  }
  pwmCommand = strtol(pwmReceived, 0, 16);
  if (pwmCommand != lastPWMCommand) {
    sendCounter = 3;     //CLARIFY: what is send counter?
  }
}

/* Request the servo for feedback, and save the feedback pulse width (in microsecond) to servoPWM.*/
int readFeedbackFromServo() {
  //backup last value in case new measurement fails. (Not used anywhere else.)
  int lastPWMServo = servoPWM; 
  
  //request servo position feedback by sending it a 50us pulse
  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(MOTOR_PIN, LOW);
  
  //read feedback
  servoPWM = pulseIn(MOTOR_PIN, HIGH, 2000); //measure the duration of the returning HIGH pulse (in microseconds), or time-out

  //fix faulty measurements
  if ((servoPWM < 300) || (servoPWM > 2000)) { //results outside these boundaries are faulty
    servoPWM = lastPWMServo;
  }
  else if (servoPWM < FEEDBACK_PWM_MIN) {
    servoPWM = FEEDBACK_PWM_MIN;
  }
  else if (servoPWM > FEEDBACK_PWM_MAX) {
    servoPWM = FEEDBACK_PWM_MAX;
  }
}

/* Return the servo's crossing status.
   Return 0 if it has exited the crossing zone, or CLOCKWISE/ANTICLOCKWISE if it's still crossing. */
int checkCrossingStatus(){
  if (crossingCommand == CLOCKWISE) { // From left
    if ((loopAveragePWM > (FEEDBACK_PWM_MIN)) && (servoPWM <= FEEDBACK_PWM_MIDDLE)) { //smaller/equal because middlePWMFEedback is rounded down
      return 0;
    } else {
      return CLOCKWISE;
    }
  }
  else if (crossingCommand == ANTICLOCKWISE){ // From right
    if ((loopAveragePWM < (FEEDBACK_PWM_MAX)) && (servoPWM > FEEDBACK_PWM_MIDDLE )) {
      return 0;
    } else {
      return ANTICLOCKWISE;
    }
  }

  //if all else failed
  return 0;
}

/* Send appropriate pwm command to servo, depending on whether it's crossing or not. 
   If it's crossing, send velocity command; otherwise, position command. */
void ctrl_motor(int pwmMotor) {
  if (crossingCommand == CLOCKWISE) {          //crossing from the left
    writePulseToServo(CLOCKWISE_PWM_MIN); //min speed
    writePulseToServo(CLOCKWISE_PWM_MIN);
  } else if (crossingCommand == ANTICLOCKWISE){   //crossing from the right
    writePulseToServo(ANTICLOCKWISE_PWM_MIN); //min speed
    writePulseToServo(ANTICLOCKWISE_PWM_MIN);
  } else if (sendCounter > 0) {
    writePulseToServo(pwmMotor);
    writePulseToServo(pwmMotor);
    sendCounter--;
  }
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

/* Convert averaged length feedback from DEC to HEX char, and send to mega via Serial. */
void sendFeedback() {
  itoa(loopAveragePWM, pwmFeedback, 16); 
  for (int i = 0; i < DIGITS_PWM_FEEDBACK; i++) {
    Serial.print(pwmFeedback[i]);
  }
  Serial.println();
  Serial.flush();
}


void testdrive() {
  if (cw) {
    if (pwmTestrun < (COMMAND_PWM_MAX + 20)) {
      pwmTestrun += 20;
    }
    else {
      pwmTestrun -= 20;
      cw = 0;
    }
  } else {
    if (pwmTestrun > (COMMAND_PWM_MIN - 20)) {
      pwmTestrun -= 20;
    }
    else {
      pwmTestrun += 20;
      cw = 1;
    }
  }
  Serial.print('c');
  Serial.println('c');
  Serial.flush();
  delayMicroseconds(10);

  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(pwmTestrun);
  digitalWrite(MOTOR_PIN, LOW);
  delayMicroseconds(3000 - pwmTestrun);
}

