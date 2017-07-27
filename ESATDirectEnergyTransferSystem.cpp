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
#include "ESATDirectEnergyTransferSystem.h"

word ESATDirectEnergyTransferSystem::read(const word channelConfigurationBits)
{
  startConversion(channelConfigurationBits);
  delay(1);
  return readConvertedValue();
}

word ESATDirectEnergyTransferSystem::readConvertedValue()
{
  Wire1.beginTransmission(address);
  Wire1.write(conversionRegister);
  const byte wireStatus = Wire1.endTransmission();
  if (wireStatus == 0)
  {
    error = false;
    Wire1.requestFrom((uint8_t) address, (uint8_t) 2);
    const byte highByte = Wire1.read();
    const byte lowByte = Wire1.read();
    return word(highByte, lowByte);
  }
  else
  {
    error = true;
  }
}

word ESATDirectEnergyTransferSystem::readCurrent()
{
  return read(currentChannelConfigurationBits);
}

word ESATDirectEnergyTransferSystem::readShuntVoltage()
{
  return read(shuntVoltageChannelConfigurationBits);
}

word ESATDirectEnergyTransferSystem::readVoltage()
{
  return read(voltageChannelConfigurationBits);
}

void ESATDirectEnergyTransferSystem::startConversion(const word channelConfigurationBits)
{
  const word configurationBits =
    startSingleShotConversionConfigurationBits |
    comparatorConfigurationBits |
    dataRateConfigurationBits |
    fullScaleRangeConfigurationBits |
    channelConfigurationBits;
  Wire1.beginTransmission(address);
  Wire1.write(configurationRegister);
  Wire1.write(highByte(configurationBits));
  Wire1.write(lowByte(configurationBits));
  Wire1.endTransmission();
}

ESATDirectEnergyTransferSystem DirectEnergyTransferSystem;
