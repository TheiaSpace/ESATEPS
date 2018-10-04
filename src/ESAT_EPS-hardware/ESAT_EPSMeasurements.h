/*
 * Copyright (C) 2017 Theia Space, Universidad Politécnica de Madrid
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

#ifndef ESAT_EPSMeasurements_h
#define ESAT_EPSMeasurements_h

#include <Arduino.h>

// EPS current and voltage measurements.
// Use the global instance ESAT_EPSMeasurements.
class ESAT_EPSMeasurementsClass
{
  public:
    // Return the current of the 3.3 V line.
    word read3V3LineCurrent();

    // Configure the measurement lines.
    void begin();

    // Return the voltage of the 3.3 V line.
    word read3V3LineVoltage();

    // Return the current of the 5 V line.
    word read5VLineCurrent();

    // Return the voltage of the 5 V line.
    word read5VLineVoltage();

    // Return the current of the input line.
    word readInputLineCurrent();

    // Return the voltage of the input line.
    word readInputLineVoltage();

    // Return the input current of solar panel 1.
    word readSolarPanel1InputCurrent();

    // Return the output current of solar panel 1.
    word readSolarPanel1OutputCurrent();

    // Return the voltage of solar panel 1.
    word readSolarPanel1Voltage();

    // Return the input current of solar panel 2.
    word readSolarPanel2InputCurrent();

    // Return the output current of solar panel 2.
    word readSolarPanel2OutputCurrent();

    // Return the voltage of solar panel 2.
    word readSolarPanel2Voltage();
};

// Global instance of the EPS measurements library.
extern ESAT_EPSMeasurementsClass ESAT_EPSMeasurements;

#endif /* ESAT_EPSMeasurements_h */
