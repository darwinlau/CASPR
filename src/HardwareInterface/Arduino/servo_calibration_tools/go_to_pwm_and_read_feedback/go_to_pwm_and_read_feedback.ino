/*
   Let user enter a pulse width value, and
   Send it to the servo to move it.
   Read feedback too.
   To quickly increase/decrease the pulse width value by 1,
   enter 8 and 2 respectivively.
*/


#define MOTOR_PIN 2
#define BAUD_RATE 74880

int pulseWidthCmd = 1000;

void setup() {
  Serial.begin(BAUD_RATE);
}

void loop() {
  if (Serial.available() > 0) {
    //process user input//
    int input = Serial.parseInt();
    if (input == 8) {                                // '8' to increase pwm command by 1
      if (pulseWidthCmd < 1600) {
        pulseWidthCmd++;
      }
    } else if (input == 2) {                         // '2' to decrease pwm command by 1
      if (pulseWidthCmd > 400) {
        pulseWidthCmd--;
      }
    } else if (input >= 400 && input <= 1600) {   // a number between 400-1600 to set pwm command
      pulseWidthCmd = input;
    }

    //send command to servo//
    writePulseToServo(pulseWidthCmd);
    writePulseToServo(pulseWidthCmd);  //the servo won't go to the destination if we only send the pulse once

    //read feedback//
    delay(200);
    int avgFeedback = readAvgFeedback(4);

    //output//
    Serial.print(pulseWidthCmd);
    Serial.print("\t");
    Serial.println(avgFeedback);
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

