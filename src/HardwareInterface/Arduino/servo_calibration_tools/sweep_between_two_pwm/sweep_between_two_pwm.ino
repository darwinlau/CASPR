/*
 * Make the servo sweep between two positions,
 * the upperBound and the lowerBound (in pulse width in microseconds).
 * Entering an integer set a new upperBound/lowerBound
 * bounding the current position.
 */


#define MOTOR_PIN 2
#define BAUD_RATE 74880

int upperBound = 510; 
int lowerBound = 450;
int pulseWidth = lowerBound;
boolean isIncreasing = true;

void setup() {
  Serial.begin(BAUD_RATE);
}

void loop() {
  /* find max and min position pulse width:
     write ramp to servo
     find out where it start/stop moving*/
  int newBound = -1;
  if (Serial.available() > 0){
    newBound = Serial.parseInt();
  }
  if (newBound < 0){ //no new upperBound/lowerBound input
    if (isIncreasing){
      pulseWidth++;
      if (pulseWidth <= upperBound){
        writePulseToServo(pulseWidth);
        Serial.println(pulseWidth);
        delay(600);
      } else {
        isIncreasing = false;
      }
    } else { //decreasing
      pulseWidth--;
      if (pulseWidth >= lowerBound){
        writePulseToServo(pulseWidth);
        Serial.println(pulseWidth);
        delay(600);
      } else {
        isIncreasing = true;
      }
    }
  } else if (newBound > 400 && newBound < 2500) { //new upperBound/lowerBound
    if (newBound >= pulseWidth){
      upperBound = newBound;
    } else {
      lowerBound = newBound;
    }
    Serial.print("===== ");
    Serial.print(lowerBound);
    Serial.print(" , ");
    Serial.print(upperBound);
    Serial.println(" =====");
  }
  
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

