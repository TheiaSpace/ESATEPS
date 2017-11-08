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

#include "ESAT_EPSMeasurements.h"

// Number of samples for current measurements.
static const byte currentSamples = 5;

// Number of samples for voltage measurements.
static const byte voltageSamples = 1;

void ESAT_EPSMeasurementsClass::begin()
{
  analogReference(INTERNAL2V5);
  pinMode(I_3V3, INPUT);
  pinMode(V_3V3, INPUT);
  pinMode(I_5V, INPUT);
  pinMode(V_5V, INPUT);
  pinMode(I_IN, INPUT);
  pinMode(V_IN, INPUT);
  pinMode(I_P1_IN, INPUT);
  pinMode(I_P1_OUT, INPUT);
  pinMode(I_P2_IN, INPUT);
  pinMode(V_P1, INPUT);
  pinMode(I_P2_OUT, INPUT);
  pinMode(V_P2, INPUT);
}

// Return the average of a number of samples of an analog line.
static word read(const byte line, const byte samples)
{
  double sum = 0;
  for (int sample = 0; sample < samples; sample++)
  {
    sum = sum + analogRead(line);
  }
  return round(sum / samples);
}

word ESAT_EPSMeasurementsClass::read3V3LineCurrent()
{
  return read(I_3V3, currentSamples);
}

word ESAT_EPSMeasurementsClass::read3V3LineVoltage()
{
  return read(V_3V3, voltageSamples);
}

word ESAT_EPSMeasurementsClass::read5VLineCurrent()
{
  return read(I_5V, currentSamples);
}

word ESAT_EPSMeasurementsClass::read5VLineVoltage()
{
  return read(V_5V, voltageSamples);
}

word ESAT_EPSMeasurementsClass::readInputLineCurrent()
{
  return read(I_IN, currentSamples);
}

word ESAT_EPSMeasurementsClass::readInputLineVoltage()
{
  return read(V_IN, voltageSamples);
}

word ESAT_EPSMeasurementsClass::readSolarPanel1InputCurrent()
{
  return read(I_P1_IN, currentSamples);
}

word ESAT_EPSMeasurementsClass::readSolarPanel1OutputCurrent()
{
  return read(I_P1_OUT, currentSamples);
}

word ESAT_EPSMeasurementsClass::readSolarPanel1Voltage()
{
  return read(V_P1, voltageSamples);
}

word ESAT_EPSMeasurementsClass::readSolarPanel2InputCurrent()
{
  return read(I_P2_IN, currentSamples);
}

word ESAT_EPSMeasurementsClass::readSolarPanel2OutputCurrent()
{
  return read(I_P2_OUT, currentSamples);
}

word ESAT_EPSMeasurementsClass::readSolarPanel2Voltage()
{
  return read(V_P2, voltageSamples);
}

ESAT_EPSMeasurementsClass ESAT_EPSMeasurements;
