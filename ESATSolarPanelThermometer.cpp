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

#include <Wire.h>
#include "ESATSolarPanelThermometer.h"

// Address/register pairs.
static const byte solarPanel1PrimaryAddress = 0x4a;
static const byte solarPanel1PrimaryRegister = 0x00;
static const byte solarPanel1SecondaryAddress = 0x18;
static const byte solarPanel1SecondaryRegister = 0x05;
static const byte solarPanel2PrimaryAddress = 0x49;
static const byte solarPanel2PrimaryRegister = 0x00;
static const byte solarPanel2SecondaryAddress = 0x1c;
static const byte solarPanel2SecondaryRegister = 0x05;


ESATSolarPanelThermometer::ESATSolarPanelThermometer(const byte primaryAddress,
                                                     const byte primaryRegister,
                                                     const byte secondaryAddress,
                                                     const byte secondaryRegister):
  primaryAddress(primaryAddress),
  primaryRegister(primaryRegister),
  secondaryAddress(secondaryAddress),
  secondaryRegister(secondaryRegister),
  error(false),
  success(false)
{
}

word ESATSolarPanelThermometer::read()
{
  const word primaryTemperature = tryToRead(primaryAddress, primaryRegister);
  if (success)
  {
    return primaryTemperature;
  }
  const word secondaryTemperature = tryToRead(secondaryAddress, secondaryRegister);
  if (success)
  {
    return secondaryTemperature;
  }
  error = true;
  return 0;
}

word ESATSolarPanelThermometer::tryToRead(const byte address, const byte registerNumber)
{
  Wire1.beginTransmission(address);
  Wire1.write(registerNumber);
  const byte wireStatus = Wire1.endTransmission();
  if (wireStatus == 0)
  {
    success = true;
    Wire1.requestFrom((uint8_t) address, (uint8_t) 2);
    const byte highByte = Wire1.read();
    const byte lowByte = Wire1.read();
    return word(highByte, lowByte);
  }
  else
  {
    success = false;
    return 0;
  }
}

ESATSolarPanelThermometer SolarPanel1Thermometer(solarPanel1PrimaryAddress,
                                                 solarPanel1PrimaryRegister,
                                                 solarPanel1SecondaryAddress,
                                                 solarPanel1SecondaryRegister);

ESATSolarPanelThermometer SolarPanel2Thermometer(solarPanel2PrimaryAddress,
                                                 solarPanel2PrimaryRegister,
                                                 solarPanel2SecondaryAddress,
                                                 solarPanel2SecondaryRegister);
