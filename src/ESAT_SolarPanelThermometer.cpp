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

#include "ESAT_SolarPanelThermometer.h"
#include <Wire.h>

// Address/register pairs.
static const byte solarPanel1PrimaryAddress = 0x4a;
static const byte solarPanel1PrimaryRegister = 0x00;
static const byte solarPanel1SecondaryAddress = 0x18;
static const byte solarPanel1SecondaryRegister = 0x05;
static const byte solarPanel2PrimaryAddress = 0x49;
static const byte solarPanel2PrimaryRegister = 0x00;
static const byte solarPanel2SecondaryAddress = 0x1c;
static const byte solarPanel2SecondaryRegister = 0x05;


ESAT_SolarPanelThermometerClass::ESAT_SolarPanelThermometerClass(const byte primaryAddress,
                                                                 const byte primaryRegister,
                                                                 const byte secondaryAddress,
                                                                 const byte secondaryRegister):
  primaryAddress(primaryAddress),
  primaryRegister(primaryRegister),
  secondaryAddress(secondaryAddress),
  secondaryRegister(secondaryRegister),
  error(false),
  previousReading(0),
  previousReadingTime(0)
{
}

word ESAT_SolarPanelThermometerClass::read()
{
  unsigned long currentReadingTime = millis();
  if ((currentReadingTime - previousReadingTime) < PERIOD)
  {
    return previousReading;
  }
  previousReadingTime = currentReadingTime;
  const word primaryTemperature =
    tryRead(primaryAddress, primaryRegister);
  if (successfulRead)
  {
    previousReading = primaryTemperature;
    return primaryTemperature;
  }
  const word secondaryTemperature =
    tryRead(secondaryAddress, secondaryRegister);
  if (successfulRead)
  {
    previousReading = secondaryTemperature;
    return secondaryTemperature;
  }
  else
  {
    error = true;
    previousReading = 0;
    return 0;
  }
}

word ESAT_SolarPanelThermometerClass::tryRead(const byte address,
                                              const byte registerNumber)
{
  Wire1.beginTransmission(address);
  Wire1.write(registerNumber);
  const byte writeStatus = Wire1.endTransmission();
  if (writeStatus != 0)
  {
    successfulRead = false;
    return 0;
  }
  const byte bytesRead = Wire1.requestFrom(int(address), 2);
  if (bytesRead != 2)
  {
    successfulRead = false;
    return 0;
  }
  successfulRead = true;
  const byte highByte = Wire1.read();
  const byte lowByte = Wire1.read();
  return word(highByte, lowByte);
}

ESAT_SolarPanelThermometerClass ESAT_SolarPanel1Thermometer(solarPanel1PrimaryAddress,
                                                            solarPanel1PrimaryRegister,
                                                            solarPanel1SecondaryAddress,
                                                            solarPanel1SecondaryRegister);

ESAT_SolarPanelThermometerClass ESAT_SolarPanel2Thermometer(solarPanel2PrimaryAddress,
                                                            solarPanel2PrimaryRegister,
                                                            solarPanel2SecondaryAddress,
                                                            solarPanel2SecondaryRegister);