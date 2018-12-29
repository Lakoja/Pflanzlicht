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

#ifndef __ACTIVITY_MONITOR_H__
#define __ACTIVITY_MONITOR_H__

class ActivityMonitor
{
  static const int LENGTH = 24*4;
private:
  byte values[LENGTH];
  int maximumLength;
  int pointer = 0;
  long sum = 0;
  int count = 0;

public:
  ActivityMonitor(int maxLen)
  {
    maximumLength = min(maxLen, LENGTH);
    memset(values, 0, sizeof(values));
  }

  String dump()
  {
    String s;
    for (int i=0; i<maximumLength; i++) {
      s += String(values[i]) + " ";
    }

    return s;
  }

  bool record(byte value, int index = -1)
  {
    bool internalUsed = true;
    if (index != -1) {
      if (index >= maximumLength || index < -1) {
        Serial.println("!!!! Illegal index value "+String(index)+" vs maximum "+String(maximumLength));
        return;
      }
      
      internalUsed = false;
    } else {
      index = pointer;
    }

    bool valueRaised = false;

    if (value > values[index]) {
      valueRaised = true;
    }
    
    if (values[index] > 0) {
      sum -= values[index];
      values[index] = 0;
      count--;
    }
    
    values[index] = value;

    if (value > 0) {
      sum += value;
      count++;
    }

    if (internalUsed) {
      pointer = (pointer + 1) % maximumLength;
    }

    return valueRaised;
  }

  byte current(int index = -1)
  {
    if (index == -1) {
      int index = pointer - 1;
      if (index < 0) {
        index = index + maximumLength;
      }
    }

    return values[index];
  }

  int countValues()
  {
    return count;
  }

  int countLightBack(int steps)
  {
    int lightCounter = 0;
    
    for (int i=0; i<steps; i++) {
      int p = pointer - 1 - i;
      if (p < 0) {
        p = p + maximumLength;
      }

      // TODO this is rather specific knowledge here; and see ino:isDay()
      if (values[p] > 0 && values[p] < 10) {
        lightCounter++;
      }
    }

    return lightCounter;
  }

  long sumValues()
  {
    return sum;
  }

  int maximumCount() {
    return maximumLength;
  }
};

#endif
