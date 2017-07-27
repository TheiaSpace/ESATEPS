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

#ifndef ESATSolarPanelThermometer_h
#define ESATSolarPanelThermometer_h

#include <Energia.h>

// An interface to the thermometers found on ESAT's solar panels.
class ESATSolarPanelThermometer
{
  public:
    // True after a read error.  Must be reset manually.
    byte error;

    // Instantiate a solar panel thermometer interface.
    // Support two devices for current and old thermometers.
    ESATSolarPanelThermometer(byte primaryAddress,
                              byte primaryRegister,
                              byte secondaryAddress,
                              byte SecondaryRegister);

    // Return the temperature measured by the thermometer.
    // Set the error flag on error.
    int read();

  private:
    // True on read success, false otherwise.
    boolean success;

    // First try to read the temperature from this I2C address/register pair.
    const byte primaryAddress;
    const byte primaryRegister;

    // Fall back to this I2C address/register pair on error.
    const byte secondaryAddress;
    const byte secondaryRegister;

    // Try to read the temperature from the given address/register pair.
    // Set the success flag.
    int tryToRead(byte address, byte registerNumber);
};

// Thermometer on solar panel 1.
extern ESATSolarPanelThermometer SolarPanel1Thermometer;

// Thermometer on solar panel 2.
extern ESATSolarPanelThermometer SolarPanel2Thermometer;

#endif /* ESATSolarPanelThermometer_h */
