/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#include <string.h>
#include <Wire.h>

#define FEEDBACK_FREQUENCY 10// In Hz
#define FEEDBACK_FREQUENCY_COUNT 1000/FEEDBACK_FREQUENCY
#define TIME_STEP 1.0/FEEDBACK_FREQUENCY

double timeVal = 0.0;
int timeCounter = 0;
int feedbackOn = 0;

String str = "";

void setup() {
  Serial.begin(115200);

  // Setup timer0
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

SIGNAL(TIMER0_COMPA_vect)
{
  if (timeCounter < FEEDBACK_FREQUENCY_COUNT - 1)
  {
    timeCounter++;
  }
  else
  {
    timeCounter = 0;
    if (feedbackOn)
    {
      Serial.println(timeVal);
    }
    timeVal += TIME_STEP;
  }
}

void loop()
{
  if (Serial.available() > 0)
  {
    str = Serial.readStringUntil('\n');

    if (str == "a")
    {
      feedbackOn = 0;
      Serial.println("a");
    }
    if (str == "s")
      feedbackOn = 1;
    else if (str == "e")
      feedbackOn = 0;
  }
}
