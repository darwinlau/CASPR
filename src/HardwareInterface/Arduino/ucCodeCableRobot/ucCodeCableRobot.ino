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
#include <math.h>

#define FEEDBACK_FREQUENCY 10// In Hz
#define FEEDBACK_FREQUENCY_COUNT 1000/FEEDBACK_FREQUENCY
#define TIME_STEP 1.0/FEEDBACK_FREQUENCY
#define NUM_MOTORS 8
#define LENGTH_HEX_NUM_DIGITS 4
#define LENGTH_MULT 10

double timeVal = 0.0;
int timeCounter = 0;
int systemOn = 0;
double initLengths[NUM_MOTORS] = {0,0,0,0,0,0,0,0};
double relLengths[NUM_MOTORS] = {100,200,300,400,500,600,700,800};
double cndLengths[NUM_MOTORS] = {0,0,0,0,0,0,0,0};

void sendFeedback();
void setInitialLengths(String str);
void setCmdLengths(String str);
void readSerial();

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
    if (systemOn)
    {
      sendFeedback();
    }
    timeVal += TIME_STEP;
  }
}

void loop()
{
  readSerial();
}

void sendFeedback()
{
  Serial.print("F");
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    double l = (initLengths[i] + relLengths[i])*LENGTH_MULT;
    int l_rounded = l + 0.5;
    char format[4];
    char s[LENGTH_HEX_NUM_DIGITS]; 
    sprintf(format, "%%.%dX", LENGTH_HEX_NUM_DIGITS);
    sprintf(s, format, l_rounded);
    Serial.print(s);
  }
  Serial.print("\n");
}

void readSerial()
{
  if (Serial.available() > 0)
  {
    str = Serial.readStringUntil('\n');

    if (str == "a")
    {
      systemOn = 0;
      Serial.println("a");
    }
    else if (str[0] == 'I')
    {
      setInitialLengths(str);
    }
    else if (str[0] == 'L')
    {
      setCmdLengths(str);
    }
    else if (str == "s")
      systemOn = 1;
    else if (str == "e")
      systemOn = 0;
  }
}

void setInitialLengths(String str)
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    char tmp[LENGTH_HEX_NUM_DIGITS];
    for (int j = 0; j < LENGTH_HEX_NUM_DIGITS; j++)
    {
      tmp[j] = str[LENGTH_HEX_NUM_DIGITS*i + j + 1];
    }
    initLengths[i] = ((double)strtol(tmp, 0, 16))/LENGTH_MULT;
  }
}

void setCmdLengths(String str)
{
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    char tmp[LENGTH_HEX_NUM_DIGITS];
    for (int j = 0; j < LENGTH_HEX_NUM_DIGITS; j++)
    {
      tmp[j] = str[LENGTH_HEX_NUM_DIGITS*i + j + 1];
    }
    cmdLengths[i] = ((double)strtol(tmp, 0, 16))/LENGTH_MULT;
  }
}

