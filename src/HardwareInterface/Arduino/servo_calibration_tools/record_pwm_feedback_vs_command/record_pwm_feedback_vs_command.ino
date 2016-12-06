/*
 * Sweep servo and record pulse width feedback vs command.
 */

#define MOTOR_PIN 2
#define BAUD_RATE 74880

//for my NEW motor: 471 - 1487
#define MIN_PULSEWIDTH 471
#define MAX_PULSEWIDTH 1487

void setup() {
  Serial.begin(BAUD_RATE);
}

void loop() {
  //on your mark, get set...
  writePulseToServo(MIN_PULSEWIDTH);
  delay(1);
  writePulseToServo(MIN_PULSEWIDTH);
  delay(1000);

  //go!
  for (int pulseWidthCmd = MIN_PULSEWIDTH; pulseWidthCmd <= MAX_PULSEWIDTH; pulseWidthCmd++){
    //send command
    writePulseToServo(pulseWidthCmd);
    delay(1);
    writePulseToServo(pulseWidthCmd);
    delay(10);

    //read feedback
    int pulseWidthFeedback[4];
    pulseWidthFeedback[0]= readFeedbackFromServo(); delay(1);
    pulseWidthFeedback[1]= readFeedbackFromServo(); delay(1);
    pulseWidthFeedback[2]= readFeedbackFromServo(); delay(1);
    pulseWidthFeedback[3]= readFeedbackFromServo(); 
    int avgPulseWidthFeedback = (pulseWidthFeedback[0] + pulseWidthFeedback[1] + pulseWidthFeedback[2] + pulseWidthFeedback[3] + 0.5)/4; 
    
    Serial.print(pulseWidthCmd);
    Serial.print("\t");
    Serial.println(avgPulseWidthFeedback);
  }

  while(true){}  //do nothing after 1 sweep
}

/* Send one PWM pulse to servo. 
   pulseWith is in microsecond and should be non-negative number under 3000. 
   This function takes 3ms to complete. */
void writePulseToServo(int pulseWidth){
  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(MOTOR_PIN, LOW);
  delayMicroseconds(3000 - pulseWidth);
}

/*
 * Request the servo for feedback, read and return the feedback pulse width (in microsecond).
 */
int readFeedbackFromServo(){
  //send request for feedback
  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(50);  
  digitalWrite(MOTOR_PIN, LOW);

  //read feedback
  int feedback = pulseIn(MOTOR_PIN, HIGH, 2000); //measure the duration of the returning HIGH pulse (in microseconds), or time-out
  return feedback;
}

