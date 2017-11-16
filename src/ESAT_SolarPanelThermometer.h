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

#ifndef ESAT_SolarPanelThermometer_h
#define ESAT_SolarPanelThermometer_h

#include <Arduino.h>

// An interface to the thermometers found on ESAT's solar panels.
// Use the global instances ESAT_SolarPanel1Thermometer (for the
// thermometer on solar panel 1) and ESAT_SolarPanel2Thermometer (for the
// thermometer on solar panel 2).
//
// The underlying hardware is the ADS7823 analog-to-digital converter
// from Texas Instruments measuring the AD8494 thermocouple amplifier
// from Analog Devices.  Communications are done through the EPS I2C
// bus.  There is also support for a different thermometer chip from
// an older, in-house testing design that will be removed in a future
// version of this library.
class ESAT_SolarPanelThermometerClass
{
  public:
    // True after a read error.  Must be reset manually.
    boolean error;

    // Instantiate a solar panel thermometer interface.
    // Support two devices for current and old thermometers.
    ESAT_SolarPanelThermometerClass(byte primaryAddress,
                                    byte primaryRegister,
                                    byte secondaryAddress,
                                    byte SecondaryRegister);

    // Return the temperature measured by the thermometer.
    // Set the error flag on error.
    // Readings are updated up to once every PERIOD milliseconds.
    // The error flag is reset to the previous cached value if fewer
    // than PERIOD milliseconds have ellapsed since the previous
    // update.
    word read();

  private:
    // First try to read the temperature from this I2C address/register pair.
    const byte primaryAddress;
    const byte primaryRegister;

    // Fall back to this I2C address/register pair on error.
    const byte secondaryAddress;
    const byte secondaryRegister;

    // Temperature readings may update up to once every PERIOD
    // milliseconds.  Panel thermal dynamics are slow, so measuring
    // more often would just waste processor time for no real benefit.
    static const unsigned long PERIOD = 1000;

    // True if the last read attempt was successful; false otherwise.
    boolean successfulRead;

    // Try to read the temperature.  Set successfulRead to true on
    // success; otherwise set it to false.
    word tryRead(const byte address, const byte registerNumber);

    // True after a read error on the latest read() actual read
    // operation; false otherwise.
    boolean previousError;

    // Temperature measured in the previous reading operation.
    word previousReading;

    // System uptime at the previous temperature reading.
    unsigned long previousReadingTime;
};

// Thermometer on solar panel 1.
extern ESAT_SolarPanelThermometerClass ESAT_SolarPanel1Thermometer;

// Thermometer on solar panel 2.
extern ESAT_SolarPanelThermometerClass ESAT_SolarPanel2Thermometer;

#endif /* ESAT_SolarPanelThermometer_h */
