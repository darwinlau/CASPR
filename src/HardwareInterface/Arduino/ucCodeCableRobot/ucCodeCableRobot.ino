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
#define LENGTH_SEND_MAX 65536
#define LENGTH_MULT 10

#define SEND_PREFIX_FEEDBACK 'f'
#define SEND_PREFIX_ERROR 'a'
#define RECEIVE_PREFIX_START 's'
#define RECEIVE_PREFIX_END 'e'
#define RECEIVE_PREFIX_INITIAL 'i'
#define RECEIVE_PREFIX_LENGTH_CMD 'l'
#define COMM_PREFIX_ACKNOWLEDGE 'a'

double timeVal = 0.0;
int timeCounter = 0;
int systemOn = 0;
double initLengths[NUM_MOTORS] = {0,0,0,0,0,0,0,0};
double relLengths[NUM_MOTORS] = {100,200,300,400,500,600,700,800};
double cableLengths[NUM_MOTORS] = {0,0,0,0,0,0,0,0};
double cmdLengths[NUM_MOTORS] = {0,0,0,0,0,0,0,0};

void sendFeedback();
void setInitialLengths(String str);
void setCmdLengths(String str);
void readSerial();


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
      for (int i = 0; i < NUM_MOTORS; i++)
      {
        cableLengths[i] = initLengths[i] + relLengths[i];
      }
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
  // Send the first character of F to indicate feedback
  Serial.print(SEND_PREFIX_FEEDBACK);
  for (int i = 0; i < NUM_MOTORS; i++)
  {
    // Convert length from mm to mm * LENGTH_MULT to send
    double l_to_send = cableLengths[i] * LENGTH_MULT;
    // Round to nearest integer value
    int l_rounded = l_to_send + 0.5;
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
    String str = Serial.readStringUntil('\n');
    if (str[0] == COMM_PREFIX_ACKNOWLEDGE && str.length() == 1)
    {
      systemOn = 0;
      Serial.println(COMM_PREFIX_ACKNOWLEDGE);
    }
    else if (str[0] == RECEIVE_PREFIX_START && str.length() == 1)
    {
      systemOn = 1;
    }
    else if (str[0] == RECEIVE_PREFIX_END && str.length() == 1)
    {
      systemOn = 0;
    }
    else if (str[0] == RECEIVE_PREFIX_INITIAL)
    {
      setInitialLengths(str);
    }
    else if (str[0] == RECEIVE_PREFIX_LENGTH_CMD)
    {
      setCmdLengths(str);
    }
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

