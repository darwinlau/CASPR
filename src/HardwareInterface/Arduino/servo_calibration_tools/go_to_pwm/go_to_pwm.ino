/*
 * Let user enter a pulse width value, and
 * Send it to the servo to move it.
 * To quickly increase/decrease the pulse width value by 1, 
 * enter 8 and 2 respectivively.
 */


#define MOTOR_PIN 2
#define BAUD_RATE 74880

int pulseWidthCmd = 1000;

void setup() {
  Serial.begin(BAUD_RATE);
}

void loop() {
  if (Serial.available() > 0){
    int input = Serial.parseInt();
    if (input == 8){                                 // '8' to increase pwm command by 1
      if (pulseWidthCmd < 1600){
        pulseWidthCmd++;
      }
    } else if (input == 2){                          // '2' to decrease pwm command by 1
      if (pulseWidthCmd > 400){
        pulseWidthCmd--;
      }
    } else if (input >= 400 && input <= 1600){    // a number between 400-1600 to set pwm command
      pulseWidthCmd = input;
    }
    
    int numOfPulseToSend = 2;
    for (int i = 0; i < numOfPulseToSend; i++){
      writePulseToServo(pulseWidthCmd);
      delay(1);
    }
    
    Serial.println(pulseWidthCmd);
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

