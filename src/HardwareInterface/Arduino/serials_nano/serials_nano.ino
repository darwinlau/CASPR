#include <string.h>
#include <Wire.h>
#include <math.h>

#define NANO_ID 0
#define MOTOR_PIN 2
#define BAUD_RATE 74880
#define DELTA 7 // freezing regions at crossing area

#define LENGTH_PWM_COMMAND 4
#define DIGITS_PWM_FEEDBACK 3

#define RECEIVE_PWM_CMD 'p'
#define RECEIVE_FEEDBACK_REQUEST 'f'
#define RECEIVE_TEST_REQUEST 't'
#define RECEIVE_TESTDRIVE_REQUEST 'z'

/////////////////////////// MOTORS DATA BANK //////////////

int maximumPWMFeedback[8] = {1501, 1494, 1501, 1493, 1501, 1499, 1501, 1520};
int minimumPWMFeedback[8] = {484, 483, 484, 482, 484, 484, 484, 491};
int middlePWMFeedback[8] = {992, 988, 992, 987, 992, 991, 992, 1005}; // all numbers rounded down
int maximumPWMOutput[8] = {1488, 1485, 1489, 1481, 1488, 1490, 1490, 1509};
int minimumPWMOutput[8] = {469, 469, 471, 473, 469, 471, 474, 481}; //3, 6, 7 increased by 5
int rangePWMOutput[8] = {1019, 1016, 1018, 1008, 1019, 1019, 1016, 1028};
int clockwise_max[8] = {2194, 2175, 2185, 2175, 2189, 2188, 2188, 2215};
int clockwise_min[8] = {2094, 2082, 2090, 2079, 2089, 2088, 2088, 2117};
int clockwise_max_speed[8] = {283, 278, 272, 269, 272, 281, 278, 278};
int clockwise_min_speed[8] = {130, 131, 127, 127, 127, 128, 133, 130};
int anticlockwise_max[8] = {1800, 1780, 1785, 1780, 1785, 1786, 1788, 1811};
int anticlockwise_min[8] = {1891, 1880, 1887, 1876, 1885, 1886, 1888, 1910};
int anticlockwise_max_speed[8] = { -281, -278, -273, -269, -270, -279, -273, -279};
int anticlockwise_min_speed[8] = { -133, -130, -132, -129, -124, -129, -128, -131};

// speed: (deltaPWM / pwmRange) * (360 / 0.01) - time steps 10ms
// roughly 8pwm / 10ms on clockwise max


/////////////////////////// COMMUNICATION ///////////////////////// //

String strReceived;
char commandReceived;

char pwmFeedback[DIGITS_PWM_FEEDBACK];


/////////////////////////// FEEDBACK VARIABLES ///////////////////////////

int servoPWM; // servo position as 'pwm' value (see _feedback for scale above)
int lastPWMServo = 0;
int loopAveragePWM = 0;

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


/////////////////////////// FUNCTION PRECALLING ///////////////////////////

int readPositionFeedback();
void loopAverage();
void crossing();
void quitCrossing();
void ctrl_motor(int pwmMotor);
void servOPulse(int pulseWidth);
void sendFeedback();
void testdrive();



void setup() {
  Serial.begin(BAUD_RATE);
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
      lastCross = 0;
      loopAverage();
      Serial.print(" avg: ");
      Serial.println(loopAveragePWM);

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
  else if (servoPWM < minimumPWMFeedback[NANO_ID]) {
    servoPWM = minimumPWMFeedback[NANO_ID];
  }
  else if (servoPWM > maximumPWMFeedback[NANO_ID]) {
    servoPWM = maximumPWMFeedback[NANO_ID];
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void crossing() { //for later optimization (exit crossing autonomously after delay, calculation based on pwmDifference)
  pwmDifference = 0;
  if (cross == 1) { //->CW crossing
    pwmDifference = rangePWMOutput[NANO_ID] + pwmCommand - lastPWMCommand;
    crossingCounter = 2 + (pwmDifference / 40);
    //delayTime = (double)(pwmDifference * 1000 / anticlockwise_max_speed[NANO_ID]);
  } else if (cross == 2) { //->CCW crossing
    pwmDifference = pwmCommand - rangePWMOutput[NANO_ID] - lastPWMCommand;
    crossingCounter = 2 + (pwmDifference / 40);
    //delayTime = (double)(pwmDifference * 1000 / clockwise_max_speed[NANO_ID]);
  }
  /*
     Serial.print(" diff: ");
    Serial.print(pwmDifference);
    Serial.print(" ");
    Serial.println(crossingCounter);
  */
  lastPWMCommand = pwmCommand;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void quitCrossing() {
  if (cross == 1) { // From left
    if ((loopAveragePWM > (minimumPWMFeedback[NANO_ID] + DELTA)) && (servoPWM <= middlePWMFeedback[NANO_ID])) { //smaller/equal because middlePWMFEedback is rounded down
      cross = 0;
    } else lastCross = cross;
  }
  else if (cross == 2) // From right
  {
    if ((loopAveragePWM < (maximumPWMFeedback[NANO_ID] - DELTA)) && (servoPWM > middlePWMFeedback[NANO_ID] )) {
      cross = 0;
    } else lastCross = cross;
  }
  else {
  }
}

void ctrl_motor(int pwmMotor) { //transmits the output signal towards the motor
  if (cross == 1) {
    crossingCounter--;
    crossPulse = clockwise_max[NANO_ID];
    servoPulse(crossPulse);
    servoPulse(crossPulse);
  } else if (cross == 2) {
    crossingCounter--;
    crossPulse = anticlockwise_max[NANO_ID];
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
  itoa(loopAveragePWM, pwmFeedback, 16);
  for (int i = 0; i < DIGITS_PWM_FEEDBACK; i++) {
    Serial.print(pwmFeedback[i]);
  }
  Serial.println();
  Serial.flush();
}

void testdrive() {
  if (cw) {
    if (pwmTestrun < (maximumPWMOutput[NANO_ID] + 20)) {
      pwmTestrun += 20;
    }
    else {
      pwmTestrun -= 20;
      cw = 0;
    }
  } else {
    if (pwmTestrun > (minimumPWMOutput[NANO_ID] - 20)) {
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

