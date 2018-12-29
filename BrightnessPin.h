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

#ifndef __BRIGHTNESS_PIN_H__
#define __BRIGHTNESS_PIN_H__

class BrightnessPin
{
private:
  int pin;
  
public:
  BrightnessPin(int p)
  {
    pin = p;
    pinMode(pin, INPUT);
  }

  byte getBrightnessK()
  {
    return (byte)round(readPin(pin));
  }

private:
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
};

#endif
