
/*
 * receive, a HEX string, representing an angle, command, from mega/serial monitor, via RX pin
 * receive, a HEX string, representing an angle, command, from mega/serial monitor, via RX pin
 * extract the command for itself and discard the command for other servos.
 * send, a pulse, representing an angle/velocity, command, to servo, via MOTOR_PIN (Where angle command fails, velocity command is sent.)
 * read, a pulse, representing an angle, of servo's current position (feedback), from servo, via MOTOR_PIN
 * send, a HEX string, representing and angle, of servo's current position (feedback), to mega/serial monitor, via TX pin
 */

#include <string.h>
#include <Wire.h>
#include <math.h>

#define NANO_ID 0
#define MOTOR_PIN 2
#define BAUD_RATE 74880
#define DELTA 7 // freezing regions at crossing area    //CLARIFY

#define LENGTH_OF_ANGLE_COMMAND 4 
#define LENGTH_OF_ANGLE_FEEDBACK 3  

#define RECEIVE_PWM_CMD 'p'
#define RECEIVE_FEEDBACK_REQUEST 'f'


/////////////////////////// MOTORS DATA BANK ////////////// 
//(k for constant)

//updated for the new dingbot test servo
const int kMaximumPWMFeedback = 1501; 
const int kMinimumPWMFeedback = 487;      
const int kMiddlePWMFeedback = 994; 
const int kMaximumPWMOutput = 1489;   
const int kMinimumPWMOutput = 481;      
const int kRangePWMOutput = 1008;  

//not updated
const int kClockwise_max[8] = {2194, 2175, 2185, 2175, 2189, 2188, 2188, 2215};
const int kClockwise_min[8] = {2094, 2082, 2090, 2079, 2089, 2088, 2088, 2117};
const int kClockwise_max_speed[8] = {283, 278, 272, 269, 272, 281, 278, 278};
const int kClockwise_min_speed[8] = {130, 131, 127, 127, 127, 128, 133, 130};
const int kAntiClockwise_max[8] = {1800, 1780, 1785, 1780, 1785, 1786, 1788, 1811};
const int kAntiClockwise_min[8] = {1891, 1880, 1887, 1876, 1885, 1886, 1888, 1910};
const int kAntiClockwise_max_speed[8] = { -281, -278, -273, -269, -270, -279, -273, -279};
const int kAntiClockwise_min_speed[8] = { -133, -130, -132, -129, -124, -129, -128, -131};

// speed: (deltaPWM / pwmRange) * (360 / 0.01) - time steps 10ms
// roughly 8pwm / 10ms on clockwise max


/////////////////////////// COMMUNICATION ///////////////////////// //

String strReceived;
char typeOfCmdReceived;
char angleFeedback[LENGTH_OF_ANGLE_FEEDBACK];


/////////////////////////// FEEDBACK VARIABLES ///////////////////////////

int servoPWM; // servo position as 'pwm' value (see _feedback for scale above)
int lastPWMServo = 0;
int lastloopAveragePWM = 0;
int loopAveragePWM = 0;
int averageSpeed = 0;

//new
int lastServoFeedback = 0;  //store last feedback in case next feedback fails

/////////////////////////// COMMAND AND MOTOR CONTROL ///////////////////////////

int pwmCommand = 0;
int lastPWMCommand = 0;

int cross = 0;
int lastCross = 0;  //CLARIFY
int crossPulse = 0;
int pwmDifference = 0;
int crossingCounter = 0;

int sendCounter = 0;

/////////////////////////// FUNCTION PRECALLING ///////////////////////////

//int readPositionFeedback();
//void loopAverage();

void crossing();
void quitCrossing();
void ctrl_motor(int pwmMotor);
void servOPulse(int pulseWidth);
void sendFeedback();

//new
int readAvgFeedback();
int readServoFeedback();

/////////////////////////////////////////////////////////////////////////////
//////////////////         SETUP() AND LOOP()         ///////////////////////
/////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(BAUD_RATE);

  //store a valid feedback in case next feedback fails
  //CLARIFY: is that all the reason why I do this?
  do {
    lastServoFeedback = readServoFeedback();
  } while (lastServoFeedback == 0);
}

void loop() {
  readSerial();
}

/////////////////////////////////////////////////////////////////////////////
//////////////////             FUNCTIONS              ///////////////////////
/////////////////////////////////////////////////////////////////////////////

void readSerial() { //receive characterizing prefix (+ length in 2 digit Hex, with manipulation of first bit for sign)
  if (Serial.available() > 0) {
    strReceived = Serial.readStringUntil('\n');
    typeOfCmdReceived = strReceived[0];
    if (typeOfCmdReceived == RECEIVE_PWM_CMD) { //p
      lastCross = 0;
      loopAverage();  //why update loopAveragePWM and servoPWM?

      if (cross > 0) {
        if (crossingCounter > 0) {
          quitCrossing();
        } else {
          cross = 0;
          lastCross = 0;
          crossingCounter = 0;
        }
      }
      delay(5);
      readPWMCommand();
      if (lastCross > 0 && cross == 0) { //if lastCross > 0, the crossing has not been quit, but cross has been reassigned by the new incomming command
        cross = lastCross;
      }
      crossing();
      ctrl_motor(pwmCommand);
    }

    else if (typeOfCmdReceived == RECEIVE_FEEDBACK_REQUEST) { //f
      char id = strReceived[1];
      if (id == NANO_ID + '0') {
        sendFeedback();
      }
    }
  }
  // ADD CALIBRATION LATER
}

/*void loopAverage() {
  readPositionFeedback();
  loopAveragePWM = servoPWM;
  for (int i = 0; i < 3; i++) {
    delay(3);
    readPositionFeedback();
    loopAveragePWM += servoPWM;
  }
  loopAveragePWM = (int)(loopAveragePWM / 4.0);
}
*/

/* read feedback pulse width (in microsecond) from servo # times and return the average value. */
int readAvgFeedback(int numOfSamples) {
  double sumOfFeedback = 0;
  for (int i = 0; i < numOfSamples; i++) {
    sumOfFeedback += readServoFeedback();
    delay(1);
  }
  int avgFeedback = (sumOfFeedback / numOfSamples) + 0.5;  //+0.5 to turn truncate into round-off
  return avgFeedback;
}

/* Read the current position of the servo. 
   Return the pulse width of the feedback pulse the servo produced (in microsecond).
   faulty feedback is corrected. */
int readServoFeedback() {
  //send request for feedback
  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(MOTOR_PIN, LOW);

  //read feedback
  int feedback = pulseIn(MOTOR_PIN, HIGH, 2000); //measure the duration of the returning HIGH pulse (in microseconds), or time-out in 2000 microseconds
  
  //correct faulty feedback
  if ((servoPWM < 300) || (servoPWM > 2000)) { //results outside 300us-2000us are really faulty
    servoPWM = lastServoFeedback;
  } else if (servoPWM < kMinimumPWMFeedback) { //a little out-of-bound
    servoPWM = kMinimumPWMFeedback;
  } else if (servoPWM > kMaximumPWMFeedback) { //a little out-of-bound
    servoPWM = kMaximumPWMFeedback;
  }

  //store this feedback value in case next one fails
  lastServoFeedback = feedback;   

  //return the servo position feedback
  return feedback;
}

void readPWMCommand() {
  char pwmReceived[LENGTH_OF_ANGLE_COMMAND];
  cross = strReceived[1 + (LENGTH_OF_ANGLE_COMMAND * NANO_ID)] - '0';
  for (int i = 0; i < LENGTH_OF_ANGLE_COMMAND - 1; i++) {
    pwmReceived[i] = strReceived[1 + i + 1 + (LENGTH_OF_ANGLE_COMMAND * NANO_ID)]; //+1 for prefix, another +1 to omit the crossing boolean
  }
  pwmCommand = strtol(pwmReceived, 0, 16);
  if (pwmCommand != lastPWMCommand) {
    sendCounter = 3;
  }
}

/*int readPositionFeedback() { //reads position feedback and stores it in servoPWM
  int lastPWMServo = servoPWM; //temporarily stores last value as backup, if new measurement fails - not used anywhere else

  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(MOTOR_PIN, LOW);
  servoPWM = pulseIn(MOTOR_PIN, HIGH, 2000); //triggers servo, then measures time until next HIGH signal, cuts off after 3000us or 3ms

  if ((servoPWM < 300) || (servoPWM > 2000)) { //results outside these boundaries are faulty
    servoPWM = lastPWMServo;
  }
  else if (servoPWM < minimumPWMFeedback[NANO_ID]) {
    servoPWM = minimumPWMFeedback[NANO_ID];
  }
  else if (servoPWM > maximumPWMFeedback[NANO_ID]) {
    servoPWM = maximumPWMFeedback[NANO_ID];
  }
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void crossing() { //for later optimization (exit crossing autonomously after delay, calculation based on pwmDifference)
  pwmDifference = 0;
  if (cross == 1) { //->CW crossing
    pwmDifference = kRangePWMOutput + pwmCommand - lastPWMCommand;
    crossingCounter = 2 + (pwmDifference / 40);
  } else if (cross == 2) { //->CCW crossing
    pwmDifference = pwmCommand - kRangePWMOutput - lastPWMCommand;
    crossingCounter = 2 + (pwmDifference / 40);
  }
  /*
     Serial.print(" diff: ");
    Serial.print(pwmDifference);
    Serial.print(" ");
    Serial.println(crossingCounter);
  */
  lastPWMCommand = pwmCommand;
}

int crossPWM() {
  int pulse; //PWM for cross
  if (cross == 1)
  {
    if (( averageSpeed < kClockwise_min_speed) && (averageSpeed > 0))
    {
      pulse = kClockwise_min_speed[NANO_ID];
    }
    else if (( averageSpeed > kClockwise_min_speed) && ( averageSpeed < kClockwise_max_speed))
    {
      int speedRange = kClockwise_max_speed - kClockwise_min_speed[NANO_ID];
      int PWMRange = kClockwise_max[NANO_ID] - kClockwise_min[NANO_ID];
      pulse = (int)((averageSpeed - kClockwise_min_speed[NANO_ID]) / speedRange * PWMRange + kClockwise_min[NANO_ID]);
    }
    else
    {
      pulse = kClockwise_max_speed;
    }
  }
  else if (cross == 2)
  {
    if (( averageSpeed < kAntiClockwise_min_speed[NANO_ID]) && (averageSpeed < 0))
    {
      pulse = kAntiClockwise_min_speed[NANO_ID];
    }
    else if (( averageSpeed > kAntiClockwise_min_speed[NANO_ID]) && ( averageSpeed < kAntiClockwise_max_speed))
    {
      int speedRange = kAntiClockwise_max_speed - kAntiClockwise_min_speed[NANO_ID];
      int PWMRange = kAntiClockwise_max[NANO_ID] - kAntiClockwise_min[NANO_ID];
      pulse = (int)((averageSpeed - kAntiClockwise_min_speed[NANO_ID]) / speedRange * PWMRange + kAntiClockwise_min[NANO_ID]);
    }
    else
    {
      pulse = kAntiClockwise_max_speed;
    }
  }
  return pulse;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void quitCrossing() {
  if (cross == 1) { // From left
    if ((loopAveragePWM > (kMinimumPWMFeedback + DELTA)) && (servoPWM <= kMiddlePWMFeedback)) { //smaller/equal because middlePWMFeedback is rounded down
      cross = 0;
    } else lastCross = cross;
  }
  else if (cross == 2) // From right
  {
    if ((loopAveragePWM < (kMaximumPWMFeedback - DELTA)) && (servoPWM > kMiddlePWMFeedback )) {
      cross = 0;
    } else lastCross = cross;
  }
  else {
  }
}

void ctrl_motor(int pwmMotor) { //transmits the output signal towards the motor
  if (cross == 1) {
    crossingCounter--;
    //crossPulse = kClockwise_max[NANO_ID];
    crossPulse = crossPWM();
    servoPulse(crossPulse);
    servoPulse(crossPulse);
  } else if (cross == 2) {
    crossingCounter--;
    //crossPulse = antikClockwise_max[NANO_ID];
    crossPulse = crossPWM();
    servoPulse(crossPulse);
    servoPulse(crossPulse);
  } else if (sendCounter > 0) {
    servoPulse(pwmMotor);
    servoPulse(pwmMotor);
    sendCounter--;
  }
}

void servoPulse(int pulseWidth) {
  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(MOTOR_PIN, LOW);
  delayMicroseconds(3000 - pulseWidth);
}

void sendFeedback() {
  itoa(loopAveragePWM, angleFeedback, 16); 
  for (int i = 0; i < LENGTH_OF_ANGLE_FEEDBACK; i++) {
    Serial.print(angleFeedback[i]);
  }
  Serial.println();
  Serial.flush();


}


