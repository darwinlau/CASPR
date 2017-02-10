/*
   Let user enter a pulse width value, and
   Send it to the servo to move it.
   Calculates angular velocity (in 0.1gree/millisecond).
   To quickly increase/decrease the pulse width value by 1,
   enter 8 and 2 respectivively.
   To quickly send the previous command again,
   enter 1, 3, or any number less than 400.
*/

#define MOTOR_PIN 2
#define BAUD_RATE 74880

#define CROSSING_ZONE_SIZE 70   //(in 0.1degree)

#define FEEDBACK_PWM_MIN 502
#define FEEDBACK_PWM_MAX 1505


int pulseWidthCmd = 1000;

int LastPWM;
unsigned long LastTime;

void setup() {
  Serial.begin(BAUD_RATE);
}

void loop() {
  //go to command 2060
  //if speed is 0, command ++
  //remember the last no-movement command
  int pwmCommand = 2060;
  double servoSpeed = 0;
  do {
    pwmCommand++;
    runAndGetContAvgSpeed(pwmCommand);
    delay(500);
    servoSpeed = runAndGetContAvgSpeed(pwmCommand);

    Serial.print(pwmCommand);
    Serial.print("\t");
    Serial.println(servoSpeed);
    Serial.flush();
  } while (servoSpeed < 40);

  Serial.println("------------------------");
  Serial.flush();

  //record the first movement command
  //command++, command--, 
  //wait for speed to stabilize
  //record the corresponding speed
  //go to next command +=10, divisible by 10 and repeat
  //repeat until speed hits similar value 4 times
  unsigned long startTime;
  double servoSpeed1 = 0;
  double servoSpeed2 = 0;
  double servoSpeed3 = 0;
  double servoSpeed4 = 0;
  do {
    servoSpeed = runAndGetContAvgSpeed(pwmCommand + 1);
    servoSpeed = runAndGetContAvgSpeed(pwmCommand);

    startTime = millis();
    while (millis() < startTime + 10000){             //10 seconds
      servoSpeed = runAndGetContAvgSpeed(pwmCommand);
    }

    Serial.print(pwmCommand);
    Serial.print("\t");
    Serial.println(servoSpeed);
    Serial.flush();

    servoSpeed4 = servoSpeed3;
    servoSpeed3 = servoSpeed2;
    servoSpeed2 = servoSpeed1;
    servoSpeed1 = servoSpeed;

    pwmCommand += 10 - (pwmCommand % 10);

  } while(abs(servoSpeed1 - servoSpeed2) > 0.1 && abs(servoSpeed1 - servoSpeed3) > 0.1 && abs(servoSpeed1 - servoSpeed4) > 0.1);

  //go to command 1920
  //if speed is 0, command --
  //remember the last no-movement command
  pwmCommand = 1920;
  servoSpeed = 0;
  do {
    pwmCommand--;
    servoSpeed = runAndGetContAvgSpeed(pwmCommand);
  } while (servoSpeed > -40);

  Serial.print(pwmCommand + 1);
  Serial.print("\t");
  Serial.println("0");
  Serial.flush();

  //record the first movement command
  //command++, command--, 
  //wait for speed to stabilize
  //record the corresponding speed
  //go to next command -=10, divisible by 10 and repeat
  //repeat until speed hits similar value 4 times
  startTime = 0;
  servoSpeed1 = 0;
  servoSpeed2 = 0;
  servoSpeed3 = 0;
  servoSpeed4 = 0;
  do {
    servoSpeed = runAndGetContAvgSpeed(pwmCommand + 1);
    servoSpeed = runAndGetContAvgSpeed(pwmCommand);

    startTime = millis();
    while (millis() < startTime + 5000){             //5 seconds
      servoSpeed = runAndGetContAvgSpeed(pwmCommand);
    }

    Serial.print(pwmCommand);
    Serial.print("\t");
    Serial.println(servoSpeed);
    Serial.flush();

    servoSpeed4 = servoSpeed3;
    servoSpeed3 = servoSpeed2;
    servoSpeed2 = servoSpeed1;
    servoSpeed1 = servoSpeed;

    if (pwmCommand % 10 != 0){
      pwmCommand -= (pwmCommand % 10);
    } else {
      pwmCommand -= 10;
    }

  } while(abs(servoSpeed1 - servoSpeed2) > 0.1 && abs(servoSpeed1 - servoSpeed3) > 0.1 && abs(servoSpeed1 - servoSpeed4) > 0.1);

  while (true){
    //do nothing
  }
}

//input -1 to cycle without new pulse width command
double runAndGetContAvgSpeed(int input){
  //for calculating average angular velocity
  static double sum;
  static long num;

  if (input > -1) {
    if (input == 8) {                                // '8' to increase pwm command by 1
      if (pulseWidthCmd < 2500) {
        pulseWidthCmd++;
        sum = 0;
        num = 0;
      }
    } else if (input == 2) {                         // '2' to decrease pwm command by 1
      if (pulseWidthCmd > 400) {
        pulseWidthCmd--;
        sum = 0;
        num = 0;
      }
    } else if (input >= 400 && input <= 2500) {   // a number between 400-2500 to set pwm command
      pulseWidthCmd = input;
      sum = 0;
      num = 0;
    }

    //send command to servo//
    writePulseToServo(pulseWidthCmd);
    writePulseToServo(pulseWidthCmd);  //the servo won't go to the destination if we only send the pulse once
  }

  //DELAY
  delay(40);

  //measure and calculate angular velocity
  int pwm = readAvgFeedback(4);
  long timeNow = millis();

  int pwmChange = pwm - LastPWM;
  double pwmChangeOverTime = (double)pwmChange / (timeNow - LastTime) * 50;  //pulse width in microsecond / 50ms   
  double angleChangeOverTime = pwmChangeOverTime / (FEEDBACK_PWM_MAX - FEEDBACK_PWM_MIN) * (3600 - CROSSING_ZONE_SIZE);    //0.1 degree / 50ms

  //record the valid angular velocity values for averaging
  if (2000 <= pulseWidthCmd && pulseWidthCmd <= 2500){  //clockwise command (~2100-2200)
    if (angleChangeOverTime > 0){
      sum += angleChangeOverTime;
      num++;
    }
  } else if (1600 <= pulseWidthCmd && pulseWidthCmd < 2000){ //anticlockwise command (~1800-1890)
    if (angleChangeOverTime < 0){
      sum += angleChangeOverTime;
      num++;
    }
  }

  LastPWM = pwm;
  LastTime = timeNow;

  if (num != 0){
    return (sum/num);
  }

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

/* Request the servo for feedback, read and return the feedback pulse width (in microsecond).*/
int readFeedbackFromServo() {
  //send request for feedback
  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(MOTOR_PIN, LOW);

  //read feedback
  int feedback = pulseIn(MOTOR_PIN, HIGH, 2000); //measure the duration of the returning HIGH pulse (in microseconds), or time-out
  return feedback;
}

/* read feedback pulse width (in microsecond) from servo # times and return the average value. */
int readAvgFeedback(int numOfReadings) {
  double sumOfFeedback = 0;
  for (int i = 0; i < numOfReadings; i++) {
    sumOfFeedback += readFeedbackFromServo();
    delay(1);
  }
  int avgFeedback = (sumOfFeedback / numOfReadings) + 0.5;  //+0.5 to turn truncate into round-off
  return avgFeedback;
}
