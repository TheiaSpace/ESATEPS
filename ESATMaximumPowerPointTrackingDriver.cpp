/*
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

#include "ESATMaximumPowerPointTrackingDriver.h"

ESATMaximumPowerPointTrackingDriver::ESATMaximumPowerPointTrackingDriver(int sensorPin, int outputPin): sensorPin(sensorPin), outputPin(outputPin), mode(FIXED_MODE), dutyCycle(defaultDutyCycle), dutyCycleIncrement(0), previousReading(0)
{
}

void ESATMaximumPowerPointTrackingDriver::begin()
{
  pinMode(sensorPin, INPUT);
  pinMode(outputPin, OUTPUT);
}

int ESATMaximumPowerPointTrackingDriver::gradientDirection()
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

void ESATMaximumPowerPointTrackingDriver::setFixedMode()
{
  mode = FIXED_MODE;
}

void ESATMaximumPowerPointTrackingDriver::setMPPTMode()
{
  mode = MPPT_MODE;
  dutyCycle = defaultDutyCycle;
  dutyCycleIncrement = 2;
}

void ESATMaximumPowerPointTrackingDriver::setSweepMode()
{
  mode = SWEEP_MODE;
  dutyCycle = defaultDutyCycle;
  dutyCycleIncrement = 1;
}

void ESATMaximumPowerPointTrackingDriver::update()
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

void ESATMaximumPowerPointTrackingDriver::updateFixedMode()
{
  analogWrite(outputPin, dutyCycle);
}

void ESATMaximumPowerPointTrackingDriver::updateMPPTMode()
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

void ESATMaximumPowerPointTrackingDriver::updateSweepMode()
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

ESATMaximumPowerPointTrackingDriver MaximumPowerPointTrackingDriver1(I_P2_IN, PWM1);
ESATMaximumPowerPointTrackingDriver MaximumPowerPointTrackingDriver2(I_P1_IN, PWM2);
