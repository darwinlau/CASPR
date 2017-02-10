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
  if (Serial.available() > 0) {
    //process user input//
    int input = Serial.parseInt();
    if (input == 8) {                                // '8' to increase pwm command by 1
      if (pulseWidthCmd < 2500) {
        pulseWidthCmd++;
      }
    } else if (input == 2) {                         // '2' to decrease pwm command by 1
      if (pulseWidthCmd > 400) {
        pulseWidthCmd--;
      }
    } else if (input >= 400 && input <= 2500) {   // a number between 400-2500 to set pwm command
      pulseWidthCmd = input;
    }

    //send command to servo//
    writePulseToServo(pulseWidthCmd);
    writePulseToServo(pulseWidthCmd);  //the servo won't go to the destination if we only send the pulse once


  } else {  //no user input
    
    int pwm = readAvgFeedback(4);
    long timeNow = millis();

    int pwmChange = pwm - LastPWM;
    double pwmChangeOverTime = (double)pwmChange / (timeNow - LastTime) * 50;  //pulse width in microsecond / 50ms   
    double angleChangeOverTime = pwmChangeOverTime / (FEEDBACK_PWM_MAX - FEEDBACK_PWM_MIN) * (3600 - CROSSING_ZONE_SIZE);    //0.1 degree / 50ms

    //output//
    Serial.print(pulseWidthCmd);
    Serial.print("\t");
    Serial.println(angleChangeOverTime);
    Serial.flush();

    LastPWM = pwm;
    LastTime = timeNow;

    delay(40); //40
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
