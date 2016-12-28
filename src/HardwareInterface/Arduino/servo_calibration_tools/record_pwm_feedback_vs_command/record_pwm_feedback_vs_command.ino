/*
 * Sweep servo and record pulse width feedback vs command.
 */

#define MOTOR_PIN 2
#define BAUD_RATE 74880

//for my NEW motor: 481 - 1487
#define MAX_PULSEWIDTH 1520
#define MIN_PULSEWIDTH 460

void setup() {
  Serial.begin(BAUD_RATE);
}

void loop() {
  //on your mark, get set...//
  writePulseToServo(MAX_PULSEWIDTH);
  writePulseToServo(MAX_PULSEWIDTH); //the servo won't go to the destination if we only send the pulse once
  delay(4000);
  
  //go! //
  for (int pulseWidthCmd = MAX_PULSEWIDTH; pulseWidthCmd >= MIN_PULSEWIDTH; pulseWidthCmd--) {
    //send command//
    writePulseToServo(pulseWidthCmd);
    writePulseToServo(pulseWidthCmd);

    //read feedback//
    delay(50);
    int avgFeedback = readAvgFeedback(4);
    

    //output//
    Serial.print(pulseWidthCmd);
    Serial.print("\t");
    Serial.println(avgFeedback);
    Serial.flush();
  }

  //done! Go to sleep.//
  while (true) {}
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

