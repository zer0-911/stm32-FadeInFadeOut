#include <Arduino.h>

const int potensioPin = PA4;
const int LedPin = PA1;

unsigned long interval = 200;     // the time we need to wait
unsigned long previousMillis = 0; // Analog input pin that the potentiometer is attached to
int sensorValue = 0;              // value read from the pot
unsigned long currentMillis;
void ledFading(byte led_pin)
{
  /* Global Var */
  static int8_t fadeAmount = 1;
  static uint8_t ledBrightness = 0;
  static unsigned long lastLedFading = 0;

  /* Fading */
  if (currentMillis - lastLedFading >= 5)
  {
    lastLedFading = currentMillis;
    ledBrightness += fadeAmount;
    if (ledBrightness == 0 || ledBrightness == 255)
    {
      fadeAmount = -fadeAmount;
    }
  }
  analogWrite(led_pin, ledBrightness);
}

void setup()
{
  Serial.begin(115200);
  // initialize serial communications at 9600 bps:
  pinMode(USER_BTN, INPUT_PULLUP);
  pinMode(potensioPin, INPUT);
  pinMode(LedPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}
void loop()
{

  currentMillis = millis();
  // read the analog in value:
  sensorValue = analogRead(potensioPin);
  int buttonState = digitalRead(USER_BTN);
  // print the results to the Serial Monitor:
  float volt = sensorValue * 5.0 / 1023.0;
  if (currentMillis - previousMillis >= interval)
  {
    // save the last time blinked the LED
    previousMillis = currentMillis;

    Serial.print("Tegangan: ");
    Serial.print(volt);
    Serial.println(" V");
  }
  if (volt > 2.00)
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if (buttonState == HIGH)
  {
    analogWrite(LedPin, 0);
  }
  else
  {
    ledFading(LedPin);
  }
}