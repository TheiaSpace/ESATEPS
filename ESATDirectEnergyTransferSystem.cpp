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

#include "ESATDirectEnergyTransferSystem.h"

ESATDirectEnergyTransferSystem::ESATDirectEnergyTransferSystem():
  device(Wire1, address)
{
}

word ESATDirectEnergyTransferSystem::read(const word channelConfigurationBits)
{
  startConversion(channelConfigurationBits);
  if (device.error)
  {
    return 0;
  }
  delay(1);
  const word convertedValue = readConvertedValue();
  if (device.error)
  {
    error = true;
  }
  return convertedValue;
}

word ESATDirectEnergyTransferSystem::readConvertedValue()
{
  return device.readBigEndianWord(conversionRegister);
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
  device.writeBigEndianWord(configurationRegister, configurationBits);
}

ESATDirectEnergyTransferSystem DirectEnergyTransferSystem;
