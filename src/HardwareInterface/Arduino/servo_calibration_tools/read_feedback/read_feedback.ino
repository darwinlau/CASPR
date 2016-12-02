/*
 * Read position feedback from the servo.(in pulse width in microseconds)
 * Display the current position.
 * Keep track of the min and max position it had received.
 */

#define MOTOR_PIN 2
#define BAUD_RATE 74880

int minPW = 1000; 
int maxPW = 1000;
int PW;

void setup() {
  Serial.begin(BAUD_RATE);
}

void loop() {
  //send request for feedback
  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(50);  
  digitalWrite(MOTOR_PIN, LOW);

  //read feedback
  int PW = pulseIn(MOTOR_PIN, HIGH, 2000); //measure the duration of the returning HIGH pulse (in microseconds), or time-out

  //update the maximum and the minimum feedback pulse width observed
  if (PW > maxPW){
    maxPW = PW;
  } else if (PW < minPW){
    minPW = PW;
  }

  //print results
  Serial.print("min:");
  Serial.print(minPW);
  Serial.print("\t mean:");
  Serial.print((maxPW - minPW) / 2.0);
  Serial.print("\t max:");
  Serial.print(maxPW);
  Serial.print("\t current:");
  Serial.println(PW);
}

