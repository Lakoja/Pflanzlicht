/*
 * Copyright (C) 2019 Lakoja on github.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include <Adafruit_SleepyDog.h>
#include "ActivityMonitor.h"
#include "BrightnessPin.h"

const int BRIGHT_PIN = A3;

ActivityMonitor brightness(8*4);
ActivityMonitor activitym(24*4);
BrightnessPin pin(BRIGHT_PIN);

unsigned long sleptTime;

const int LIGHT_PIN_B = 9;
const int LIGHT_PIN_R = 11;
const int LED_PIN = 13;

const byte MAX_BRIGHT = 240;
const float BLUE_PART = 0.4;
const float CLOCK_ADJUST = 1.045;

const unsigned long MINUTES15 = 15L * 60 * 1000;

unsigned long lastOutputTime;
unsigned long lastBrightCheck;
bool first = true;
unsigned long lastSbaOut;

// record brightness of last 8 hours
byte bright[8*4];
unsigned int brightPointer = 0;
unsigned long lastBrightRecord = 0;
unsigned long lastActivityRecord = 0;
byte activity[24*4];
int lastActivityPointer = -1;

int currentlyDesired = 0;
int currentlyDisplayed = 0;

void setup() {
  Serial.begin(115200);
  memset(bright, 0, sizeof(bright));
  memset(activity, 0, sizeof(activity));

  pinMode(LED_PIN, OUTPUT);

  // show a start animation and blink current value
  // 

  byte current = getBrightnessK();

  int blinkValueLength = 0;
  if (current < 25) {
    Serial.println("Showing "+String(current));
    blinkValueLength = (current / 5) * 5 + (current % 5) * 2;
  }
  int blinkValues[blinkValueLength];
  if (blinkValueLength > 0) {
    int b = 0;
    for (int i=0; i<current / 5; i++) {
      blinkValues[b++] = HIGH;
      blinkValues[b++] = HIGH;
      blinkValues[b++] = HIGH;
      blinkValues[b++] = LOW;
      blinkValues[b++] = LOW;
    }
    for (int i=0; i<current % 5; i++) {
      blinkValues[b++] = HIGH;
      blinkValues[b++] = LOW;
    }
  }

  int startAnimationLength = 11;
  int startAnimation[startAnimationLength];

  int s = 0;
  for (int i=1; i<7 && s<startAnimationLength; i++) {
    int led_value = i * 20;
    startAnimation[s++] = led_value;
  }

  for (int i=5; i>0 && s<startAnimationLength; i--) {
    int led_value = i * 20;
    startAnimation[s++] = led_value;
  }

  for (int i=0; i<startAnimationLength || i<blinkValueLength; i++) {
    if (i<startAnimationLength) {
      analogWrite(random(2) < 1 ? LIGHT_PIN_R : LIGHT_PIN_B, startAnimation[i]);
    }
    if (i<blinkValueLength) {
      digitalWrite(LED_PIN, blinkValues[i]);
    }
    delay(200);
  }

  analogWrite(LIGHT_PIN_R, 0);
  analogWrite(LIGHT_PIN_B, 0);
  digitalWrite(LED_PIN, LOW);

  //double kOhm = readPin(BRIGHT_PIN);

  /*
  // some curve...
  double brightness = (kOhm - 5.0) * 8.7;
  if (kOhm < 5)
    brightness = 0;
  else if (kOhm > 20 && kOhm <= 150)
    brightness = 146.0 - 0.87 * kOhm;
  else if (kOhm > 150)
    brightness = 20;

  int led_brightness = brightness;
  */

  unsigned long now = getNow(0);
  lastBrightCheck = now;
  recordBrightness(now, getBrightnessK(), true);
}

unsigned int num = 0;

void loop() {

  unsigned long now = getNow(0);
  
  // Brightness: ie 20-30kOhm non-brightly lit room
  //  10kOhm starting to get darker in winter
  //  150kOhm is putting the hand mostly over the LDR
  //  2kOhm sunny outside; in the room (north-side). 1kOhm looking out the window

  if (now - lastOutputTime >= 10000) {
    unsigned long seconds = now / 1000;
    unsigned int minutes = seconds / 60;
    unsigned int r_seconds = seconds % 60;
    
    double kOhm = readPin(BRIGHT_PIN);
    // String(num++)+
    Serial.println(String(minutes)+":"+String(r_seconds)+": "+String(kOhm)+" ");
    
    lastOutputTime = now;
  }

  byte current = getBrightnessK();
  recordBrightness(now, current, false);

  if (shouldBeActive() && !isDay(current)) {
    if (now - lastBrightCheck >= 5000 || first) {
      first = false;
      // 10 -> little light (20)
      // 70 -> maximum (180)
  
      currentlyDesired = (int)min(MAX_BRIGHT, round(current * 2.7));
      //Serial.println("bright->desired "+String(current)+"->"+String(currentlyDesired));

      lastBrightCheck = now;
    }
    
    show();
  } else {
    currentlyDesired = 0;
    show();
  }

  recordActivity(now, currentlyDesired);

/*
  if (now - lastLongTime > 100000) {
    unsigned long nowR = now;
    int hours = nowR / (1000L * 60 * 60);
    nowR = nowR % (1000L * 60 * 60);
    int minutes = nowR / (1000L * 60);
    nowR = nowR % (1000L * 60);
    int seconds = nowR / 1000L;

    Serial.println("Passed "+String(hours)+":"+String(minutes)+":"+String(seconds));
    
    lastLongTime = now;
  }
  */

  if (currentlyDesired != currentlyDisplayed) {
    delay(500);
  } else {
    if (currentlyDisplayed == 0) {
      // Sleeping will also deactivate PWM of course
      
      Serial.flush();
      int sleptMs = (int)(CLOCK_ADJUST * Watchdog.sleep(8000));
      getNow(sleptMs);
    } else {
      delay(8000);
    }
  }
}

void show()
{
  if (currentlyDesired != currentlyDisplayed) {
    if (currentlyDesired > currentlyDisplayed) {
      currentlyDisplayed += min(5, currentlyDesired-currentlyDisplayed);
    } else {
      currentlyDisplayed -= min(5, currentlyDisplayed-currentlyDesired);
    }
    
    analogWrite(LIGHT_PIN_R, currentlyDisplayed);
    analogWrite(LIGHT_PIN_B, round(currentlyDisplayed * BLUE_PART));
    //Serial.println("Output "+String(currentlyDisplayed)+"->"+String(currentlyDesired));
  }
}

bool shouldBeActive()
{
  // not in the morning/night (only day and evening)
  // not more than 5 hours in 24 hours
  // not more than 4 hours maximum brightness

  bool nearTheDay = isNearTheDay(false);
  int activityIn24 = countActivity(false);
  int sumActivityIn24 = sumActivity(false);
  int sumBorder = 4*4 * MAX_BRIGHT;

  bool shouldBeActive = (nearTheDay || justStarted()) && activityIn24 < 5*4 && sumActivityIn24 < sumBorder;


  unsigned long now = getNow(0);
  if (now - lastSbaOut >= 2000) {
    //Serial.println("A "+activitym.dump());
    Serial.println("sba near "+String(nearTheDay)+" act "+String(activityIn24)+" sum "+String(sumActivityIn24)+" -> "+String(shouldBeActive));
  
    lastSbaOut = now;
  }
  
  return shouldBeActive;
}

bool isDay(byte brightness)
{
  return brightness < 10;
}

bool isNearTheDay(bool debug)
{
  return countLightsIn6Hours(debug) > 1;
}

int countLightsIn6Hours(bool debug)
{
  return brightness.countLightBack(6*4);
  /*
  int lightCounter = 0;
  if (debug) {
    Serial.print("C");
  }
  for (int i = 0; i < 6*4; i++) {
    byte value = getBrightnessBack(i);
    
    if (debug) {
      Serial.print(" "+String(value));
    }
    
    if (value > 0) {
      if (isDay(value)) {
        lightCounter++;
      }
    }
  }

  if (debug) {
    Serial.println();
  }

  return lightCounter;
  */
}

bool justStarted()
{
  return brightness.countValues() < 2;
  /*
  int validValuesCounter = 0;
  // only probe beginning of array
  for (int i = 0; i < 4; i++) {
    byte value = bright[i];
    
    if (value > 0) {
      validValuesCounter++;
    }
  }

  return validValuesCounter < 2;
  */
}

void recordBrightness(unsigned long now, byte current, bool force)
{
  if (force || now - lastBrightRecord >= MINUTES15) {
    brightness.record(current);
    lastBrightRecord = now;
    /*
    bright[brightPointer] = current;
    brightPointer = (brightPointer + 1) % sizeof(bright);
    */
    Serial.println("Recording B "+String(current));
  }
}

void recordActivity(unsigned long now, byte value)
{
  unsigned int activityPointer = (now / MINUTES15) % activitym.maximumCount();
  //Serial.println("Recording A "+String(activityPointer)+": "+String(value)+" count now "+String(activitym.countValues()));
  bool valueRaised = activitym.record(value, activityPointer);

  if (valueRaised) {
    Serial.println("Recording A "+String(activityPointer)+": "+String(value)+" count now "+String(activitym.countValues()));
  }
  /*
  if (activityPointer != lastActivityPointer) {
    activity[activityPointer] = 0;
    lastActivityPointer = activityPointer;
  }
  if (value > activity[activityPointer]) {
    activity[activityPointer] = value;
    Serial.println("Recording A "+String(activityPointer)+": "+String(value));
  }*/
}

int countActivity(bool debug)
{
  return activitym.countValues();
  /*
  if (debug) {
    Serial.print("A(");
  }
  
  int activityCounter = 0;
  for (unsigned int i=0; i < 20 * 4; i++) {
    byte value = getActivityBack(i);
    
    if (debug) {
      Serial.print(String(value)+",");
    }
    
    if (value > 0) {
      activityCounter++;
    }
  }

  if (debug) {
    Serial.println(") ");
  }

  return activityCounter;
  */
}

int sumActivity(bool debug)
{
  return activitym.sumValues();
  /*
  if (debug) {
    Serial.print("S(");
  }
  
  int activitySum = 0;
  for (int i=0; i < 23 * 4; i++) {
    byte value = getActivityBack(i);

    if (debug) {
      Serial.print(String(i)+":"+String(value)+",");
    }
    
    activitySum += value;
  }

  if (debug) {
    Serial.println(")");
  }

  return activitySum;
  */
}

byte getBrightnessK()
{
  return pin.getBrightnessK();//(byte)round(readPin(BRIGHT_PIN));
}

double readPin(int pin)
{
  // 5V -> 1k2 -> LDR -> GND
  double vcc = readVcc();

  int val = analogRead(pin);

  double valVoltage = (val / 1023.0) * vcc;

  // more complex but the same (*1000)
  //double x = (1200*voltage/vcc)/(1-voltage/vcc);

  double t = vcc - valVoltage;
  if (t == 0) {
    t = 0.00001;
  }
  double kOhm = valVoltage / t * 1.2;

  return kOhm;
}

double readVcc() {
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  long result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  
  return result / 1000.0;
}

unsigned long getNow(unsigned long offset)
{
  sleptTime += offset;

  return millis() + sleptTime;
}

byte getBrightnessBack(int stepsBackward)
{
  int pointer = brightPointer - 1;
  int p = pointer - stepsBackward;
  if (p < 0) {
    p = p + sizeof(bright);
  }
  
  return bright[p];
}

byte getActivityBack(int stepsBackward)
{
  int pointer = lastActivityPointer;
  int p = pointer - stepsBackward;
  if (p < 0) {
    p = p + sizeof(activity);
  }
  
  return activity[p];
}
