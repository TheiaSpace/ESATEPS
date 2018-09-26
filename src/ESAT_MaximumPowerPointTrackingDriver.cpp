/*
 * Copyright (C) 2017, 2018 Theia Space, Universidad Politécnica de Madrid
 *
 * This file is part of Theia Space's ESAT EPS library.
 *
 * Theia Space's ESAT EPS library is free software: you can
 * redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Theia Space's ESAT EPS library is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Theia Space's ESAT EPS library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "ESAT_MaximumPowerPointTrackingDriver.h"

ESAT_MaximumPowerPointTrackingDriverClass::ESAT_MaximumPowerPointTrackingDriverClass(const int theSensorPin,
                                                                                     const int theOutputPin)
{
  sensorPin = theSensorPin;
  outputPin = theOutputPin;
  mode = FIXED_MODE;
  dutyCycle = DEFAULT_DUTY_CYCLE;
  dutyCycleIncrement = 0;
  previousReading = 0;
}

void ESAT_MaximumPowerPointTrackingDriverClass::begin()
{
  pinMode(sensorPin, INPUT);
  pinMode(outputPin, OUTPUT);
}

byte ESAT_MaximumPowerPointTrackingDriverClass::getDutyCycle()
{
  return dutyCycle;
}

byte ESAT_MaximumPowerPointTrackingDriverClass::getMode()
{
  return mode;
}

int ESAT_MaximumPowerPointTrackingDriverClass::gradientDirection()
{
  const int reading = (analogRead(sensorPin) + analogRead(sensorPin)) / 2;
  const int readingIncrement = reading - previousReading;
  previousReading = reading;
  if ((readingIncrement >= 0) && (dutyCycleIncrement > 0))
  {
    return 1;
  }
  if ((readingIncrement < 0) && (dutyCycleIncrement < 0))
  {
    return 1;
  }
  return -1;
}

void ESAT_MaximumPowerPointTrackingDriverClass::setFixedMode(const byte fixedDutyCycle)
{
  mode = FIXED_MODE;
  dutyCycle = fixedDutyCycle;
}

void ESAT_MaximumPowerPointTrackingDriverClass::setMPPTMode()
{
  mode = MPPT_MODE;
  dutyCycle = DEFAULT_DUTY_CYCLE;
  dutyCycleIncrement = 2;
}

void ESAT_MaximumPowerPointTrackingDriverClass::setSweepMode()
{
  mode = SWEEP_MODE;
  dutyCycle = DEFAULT_DUTY_CYCLE;
  dutyCycleIncrement = 1;
}

void ESAT_MaximumPowerPointTrackingDriverClass::update()
{
  switch (mode)
  {
    case FIXED_MODE:
      updateFixedMode();
      break;
    case MPPT_MODE:
      updateMPPTMode();
      break;
    case SWEEP_MODE:
      updateSweepMode();
      break;
    default:
      break;
  }
}

void ESAT_MaximumPowerPointTrackingDriverClass::updateFixedMode()
{
  analogWrite(outputPin, dutyCycle);
}

void ESAT_MaximumPowerPointTrackingDriverClass::updateMPPTMode()
{
  const int direction = gradientDirection();
  if (direction > 0)
  {
    dutyCycleIncrement = 2;
  }
  else
  {
    dutyCycleIncrement = -2;
  }
  dutyCycle = constrain(int(dutyCycle) + dutyCycleIncrement, 0, 255);
  analogWrite(outputPin, dutyCycle);
}

void ESAT_MaximumPowerPointTrackingDriverClass::updateSweepMode()
{
  if (dutyCycle == 0)
  {
    dutyCycleIncrement = 1;
  }
  if (dutyCycle == 255)
  {
    dutyCycleIncrement = -1;
  }
  dutyCycle = constrain(int(dutyCycle) + dutyCycleIncrement, 0, 255);
  analogWrite(outputPin, dutyCycle);
}

ESAT_MaximumPowerPointTrackingDriverClass ESAT_MaximumPowerPointTrackingDriver1(I_P1_OUT,
                                                                                PWM1);
ESAT_MaximumPowerPointTrackingDriverClass ESAT_MaximumPowerPointTrackingDriver2(I_P2_OUT,
                                                                                PWM2);
