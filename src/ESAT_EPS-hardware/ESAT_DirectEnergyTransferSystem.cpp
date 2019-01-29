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

#include "ESAT_EPS-hardware/ESAT_DirectEnergyTransferSystem.h"
#include <Wire.h>

word ESAT_DirectEnergyTransferSystemClass::read(const word channelConfigurationBits)
{
  // The reading/measurement process consists of two steps:
  // 1) command the measurement device to perform an analog-to-digital
  // conversion;
  // 2) ask the measurement device for the reading.
  // The measurement device has several channel; to select the desired
  // channel, we just pass it a set of channel configuration bits.
  const boolean successfulConversionStart =
    startConversion(channelConfigurationBits);
  if (!successfulConversionStart)
  {
    error = true;
    return 0;
  }
  // The measurement device needs some time to have the measurement
  // ready.  1 ms is enough for this.
  delay(1);
  const word convertedValue = readConvertedValue();
  return convertedValue;
}

word ESAT_DirectEnergyTransferSystemClass::readConvertedValue()
{
  // The reading comes from an I2C device.  It is encoded as an
  // uncalibrated 16-bit unsigned integer in big-endian format.  We
  // leave the calibration step to the ground segment.
  // ADC result is left aligned, so it has to be shifted 4 positions
  // to the right.
  WireEPS.beginTransmission(ADDRESS);
  WireEPS.write(CONVERSION_REGISTER);
  const byte writeStatus = WireEPS.endTransmission();
  if (writeStatus != 0)
  {
    error = true;
    return 0;
  }
  const byte bytesRead = WireEPS.requestFrom(int(ADDRESS), 2);
  if (bytesRead != 2)
  {
    error = true;
    return 0;
  }
  const byte highByte = WireEPS.read();
  const byte lowByte = WireEPS.read();
  return (word(highByte, lowByte)>>4);
}

word ESAT_DirectEnergyTransferSystemClass::readCurrent()
{
  return read(CURRENT_CHANNEL_CONFIGURATION_BITS);
}

word ESAT_DirectEnergyTransferSystemClass::readShuntVoltage()
{
  return read(SHUNT_VOLTAGE_CHANNEL_CONFIGURATION_BITS);
}

word ESAT_DirectEnergyTransferSystemClass::readVoltage()
{
  return read(VOLTAGE_CHANNEL_CONFIGURATION_BITS);
}

boolean ESAT_DirectEnergyTransferSystemClass::startConversion(const word channelConfigurationBits)
{
  // To take a measurement, we have to command the measurement device
  // to start an analog-to-digital conversion.  The command goes
  // through the I2C bus and consists of a set of configuration bits;
  // some of them are common to all measurement channels and others
  // are channel-specific.
  const word configurationBits =
    START_SINGLE_SHOT_CONVERSION_CONFIGURATION_BITS |
    COMPARATOR_CONFIGURATION_BITS |
    DATA_RATE_CONFIGURATION_BITS |
    FULL_SCALE_RANGE_CONFIGURATION_BITS |
    channelConfigurationBits;
  WireEPS.beginTransmission(ADDRESS);
  WireEPS.write(CONFIGURATION_REGISTER);
  WireEPS.write(highByte(configurationBits));
  WireEPS.write(lowByte(configurationBits));
  const byte status = WireEPS.endTransmission();
  if (status == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

ESAT_DirectEnergyTransferSystemClass ESAT_DirectEnergyTransferSystem;
