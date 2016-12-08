#include <string.h>
#include <Wire.h>
#include <math.h>
#include "servo_properties/servo_01.h"   //servo-specific properties (e.g. the range of pwm command it can execute) is stored here
#include <SoftwareSerial.h> //DEBUG

#define MOTOR_PIN 2
#define BAUD_RATE 74880
#define DELTA 7 // freezing regions at crossing area  

#define LENGTH_PWM_COMMAND 4
#define DIGITS_PWM_FEEDBACK 3

#define SPEEDAVG 3

#define RECEIVE_PWM_CMD 'p'
#define RECEIVE_FEEDBACK_REQUEST 'f'
#define RECEIVE_TEST_REQUEST 't'
#define RECEIVE_TESTDRIVE_REQUEST 'z'

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

int cross = 0;
int lastCross = 0;
int crossPulse = 0;
int pwmDifference = 0;
int crossingCounter = 0;

int sendCounter = 0;

/////////////////////////// DEBUGGING AND TIMING VARIABLES //////////////

unsigned long int t_ref;
double delayTime;
boolean cw = 1;
int pwmTestrun = 700;

int speedCounter = 0;
int speedStore[SPEEDAVG];

/////////////////////////// FUNCTION PRECALLING ///////////////////////////

int readPositionFeedback();
void loopAverage();
void crossing();
void quitCrossing();
void ctrl_motor(int pwmMotor);
void servOPulse(int pulseWidth);
void sendFeedback();
void testdrive();

SoftwareSerial softSerial(6,7); //RX, TX DEBUG

void setup() {
  Serial.begin(BAUD_RATE);
  softSerial.begin(BAUD_RATE);
  while (servoPWM == 0) {
    loopAverage();
  }
  lastPWMServo = servoPWM;
}

void loop() {
  readSerial();
}

void readSerial() { //receive characterizing prefix (+ length in 2 digit Hex, with manipulation of first bit for sign)
  if (Serial.available() > 0) {
    strReceived = Serial.readStringUntil('\n');
    commandReceived = strReceived[0];
    if (commandReceived == RECEIVE_PWM_CMD) { //p
      lastCross = 0;                 //CLARIFY: what mean lastCross mean?
      loopAverage();
      speedCounter++;
      //Serial.print(" avg: ");
      //Serial.println(loopAveragePWM);

      if (cross > 0) {               //CLARIFY: if crossing?
        if (crossingCounter > 0) {   //CLARIFY: if not timing-out yet?
          quitCrossing();            //CLARIFY: decide whether or not we can quit crossing (i.e. set cross = 0). If not, set lastCross = cross; ??
        } else {                     //CLARIFY: if the crossing time-out?
          cross = 0;
          lastCross = 0;
          crossingCounter = 0;
        }
      }
      delay(5);
      readPWMCommand();             //CLARIFY: (this will set  cross  according to the command from mega)?
      if (lastCross > 0 && cross == 0) { //if lastCross > 0, the crossing has not been quit, but cross has been reassigned by the new incomming command
        cross = lastCross;          //CLARIFY: if crossing is not done yet, ignore the cmd from mega, continue our crossing?
      }
      crossing();                   //CLARIFY: set crossingCounter depending on......????
      ctrl_motor(pwmCommand);
    }

    else if (commandReceived == RECEIVE_FEEDBACK_REQUEST) { //f
      char id = strReceived[1];
      if (id == NANO_ID + '0') {
        sendFeedback();
      }
    }
    else if (commandReceived == RECEIVE_TEST_REQUEST) { //t
      Serial.print('c');
      Serial.println('c');
      Serial.flush();
      delayMicroseconds(10);
    }
    else if (commandReceived == RECEIVE_TESTDRIVE_REQUEST) { //z
      testdrive();

    }
  }
  // ADD CALIBRATION LATER
}

void loopAverage() {
  readPositionFeedback();
  loopAveragePWM = servoPWM;
  for (int i = 0; i < 3; i++) {
    delay(3);
    readPositionFeedback();
    loopAveragePWM += servoPWM;
  }
  loopAveragePWM = (int)(loopAveragePWM / 4.0);
}


void readPWMCommand() {
  char pwmReceived[LENGTH_PWM_COMMAND];
  cross = strReceived[1 + (LENGTH_PWM_COMMAND * NANO_ID)] - '0';
  for (int i = 0; i < LENGTH_PWM_COMMAND - 1; i++) {
    pwmReceived[i] = strReceived[1 + i + 1 + (LENGTH_PWM_COMMAND * NANO_ID)]; //+1 for prefix, another +1 to omit the crossing boolean
  }
  pwmCommand = strtol(pwmReceived, 0, 16);
  if (pwmCommand != lastPWMCommand) {
    sendCounter = 3;
  }
}

int readPositionFeedback() { //reads position feedback and stores it in servoPWM
  int lastPWMServo = servoPWM; //temporarily stores last value as backup, if new measurement fails - not used anywhere else

  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(MOTOR_PIN, LOW);
  servoPWM = pulseIn(MOTOR_PIN, HIGH, 2000); //triggers servo, then measures time until next HIGH signal, cuts off after 3000us or 3ms

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void crossing() { //for later optimization (exit crossing autonomously after delay, calculation based on pwmDifference)
  pwmDifference = 0;
  if (cross == 1) { //->CW crossing
    pwmDifference = COMMAND_PWM_RANGE + pwmCommand - lastPWMCommand;
    crossingCounter = 2 + (pwmDifference / 40);                     //CLARIFY: what is 2? 40?
    //delayTime = (double)(pwmDifference * 1000 / ANTICLOCKWISE_SPEED_MAX);
  } else if (cross == 2) { //->CCW crossing
    pwmDifference = pwmCommand - COMMAND_PWM_RANGE - lastPWMCommand;
    crossingCounter = 2 + (pwmDifference / 40);                     //CLARIFY: what is 2? 40?
    //delayTime = (double)(pwmDifference * 1000 / CLOCKWISE_SPEED_MAX);
  }
  /*
     Serial.print(" diff: ");
    Serial.print(pwmDifference);
    Serial.print(" ");
    Serial.println(crossingCounter);
  */
  lastPWMCommand = pwmCommand;
}

//Variable crossing speed
// TODO: BUG! AVERAGE SPEED SEEMS TO BE ALWAYS 0. //
int crossPWM() {
  int pulse; //PWM for cross
  if (cross == 1)
  {
    if (( averageSpeed < CLOCKWISE_SPEED_MIN) && (averageSpeed > 0))
    {
      pulse = CLOCKWISE_PWM_MIN;
    }
    else if (( averageSpeed > CLOCKWISE_SPEED_MIN) && ( averageSpeed < CLOCKWISE_SPEED_MAX))
    {
      int speedRange = CLOCKWISE_SPEED_MAX - CLOCKWISE_SPEED_MIN;
      int PWMRange = CLOCKWISE_PWM_MAX - CLOCKWISE_PWM_MIN;
      pulse = (int)((averageSpeed - CLOCKWISE_SPEED_MIN) / speedRange * PWMRange + CLOCKWISE_PWM_MIN);
    }
    else
    {
      pulse = CLOCKWISE_PWM_MAX;
    }
  }
  else if (cross == 2)
  {
    if (( averageSpeed < ANTICLOCKWISE_SPEED_MIN) && (averageSpeed < 0))
    {
      pulse = ANTICLOCKWISE_PWM_MIN;
    }
    else if (( averageSpeed > ANTICLOCKWISE_SPEED_MIN) && ( averageSpeed < ANTICLOCKWISE_SPEED_MAX))
    {
      int speedRange = ANTICLOCKWISE_SPEED_MAX - ANTICLOCKWISE_SPEED_MIN;
      int PWMRange = ANTICLOCKWISE_PWM_MAX - ANTICLOCKWISE_PWM_MIN;
      pulse = (int)((averageSpeed - ANTICLOCKWISE_SPEED_MIN) / speedRange * PWMRange + ANTICLOCKWISE_PWM_MIN);
    }
    else
    {
      pulse = ANTICLOCKWISE_PWM_MAX;  
    }
  }
  return pulse;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void quitCrossing() {
  if (cross == 1) { // From left
    if ((loopAveragePWM > (FEEDBACK_PWM_MIN + DELTA)) && (servoPWM <= FEEDBACK_PWM_MIDDLE)) { //smaller/equal because middlePWMFEedback is rounded down
      cross = 0;
    } else lastCross = cross;
  }
  else if (cross == 2) // From right
  {
    if ((loopAveragePWM < (FEEDBACK_PWM_MAX - DELTA)) && (servoPWM > FEEDBACK_PWM_MIDDLE )) {
      cross = 0;
    } else lastCross = cross;
  }
  else {
  }
}

void ctrl_motor(int pwmMotor) { //transmits the output signal towards the motor
  if (cross == 1) {
    crossingCounter--;
    crossPulse = CLOCKWISE_PWM_MIN;  //cross at min speed
    //crossPulse = crossPWM();
    servoPulse(crossPulse);
    servoPulse(crossPulse);
    softSerial.println(crossPulse); //DEBUG
    softSerial.flush();//DEBUG
  } else if (cross == 2) {
    crossingCounter--;
    crossPulse = ANTICLOCKWISE_PWM_MIN;  ////cross at min speed
    //crossPulse = crossPWM();
    servoPulse(crossPulse);
    servoPulse(crossPulse);
    softSerial.println(crossPulse); //DEBUG
    softSerial.flush();//DEBUG
  } else if (sendCounter > 0) {
    servoPulse(pwmMotor);
    servoPulse(pwmMotor);
    sendCounter--;
    softSerial.println(pwmMotor); //DEBUG
    softSerial.flush();//DEBUG
  }
}

void servoPulse(int pulseWidth) {
  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(MOTOR_PIN, LOW);
  delayMicroseconds(3000 - pulseWidth);
}


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

