/*
   Read position feedback from the servo.(in pulse width in microseconds)
   Average the last few values.
   Print to serial in a graph-friendly format.
*/

#define MOTOR_PIN 2
#define BAUD_RATE 74880
#define NUM_OF_READINGS 5

int PW[NUM_OF_READINGS];

void setup() {
  Serial.begin(BAUD_RATE);
}

void loop() {
  PW[0] = readPositionFeedback();

  //calculate envelop mean from feedback
  int maxPW = PW[0];
  int minPW = PW[0];
  for (int i = 1; i < NUM_OF_READINGS; i++){
    if (PW[i] > maxPW){
      maxPW = PW[i];
    }
    if (PW[i] < minPW){
      minPW = PW[i];
    }
  }
  double meanPW = ( (double)maxPW+(double)minPW ) / 2;

  //display PW cap: 500
  if (meanPW > 500) {
    meanPW = 500;
  }
  double currentPW = PW[0];
  if (currentPW > 500) {
    currentPW = 500;
  }
  Serial.print("500\t");
  Serial.print(currentPW);
  Serial.print("\t");
  Serial.println(meanPW);

  for (int i = NUM_OF_READINGS - 1; i > 0; i--){
    PW[i] = PW[i-1];
  }

  delay(3);
}

int readPositionFeedback() {
  //send request for feedback
  digitalWrite(MOTOR_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(MOTOR_PIN, LOW);
  
  //read feedback
  int feedbackPulseWidth = pulseIn(MOTOR_PIN, HIGH, 2000); //measure the duration of the returning HIGH pulse (in microseconds), or time-out
  return feedbackPulseWidth;
}

