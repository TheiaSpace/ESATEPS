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

#ifndef ESATDirectEnergyTransferSystem_h
#define ESATDirectEnergyTransferSystem_h

#include <Energia.h>

// Interface with the direct energy transfer system.
class ESATDirectEnergyTransferSystem
{
  public:
    // True after a read error, false on read success.
    boolean error;

    // Read the current circulating through the direct energy transfer
    // system.
    // Reassign the error flag.
    word readCurrent();

    // Read the voltage drop through the shunt resistor.
    // Reassign the error flag.
    word readShuntVoltage();

    // Read the voltage through the direct energy transfer system.
    // Reassign the error flag.
    word readVoltage();

  private:
    // Address of the sensor chip.
    static const byte address = 0x48;

    // Registers.
    static const byte configurationRegister = 0x01;
    static const byte conversionRegister = 0x00;

    // Configuration bits.  The device is a TI ADS1015.
    static const word startSingleShotConversionConfigurationBits = B1 << 15; // Start a single shot conversion.
    static const word comparatorConfigurationBits = B11 << 0; // Disable comparator.
    static const word dataRateConfigurationBits = B100 << 5; // 1600 samples per second.
    static const word operatingModeConfigurationBits = B1 << 8; // Single conversion.
    static const word fullScaleRangeConfigurationBits = B001 << 9; // +-4.096 V.

    // Channel configuration bits.
    static const word currentChannelConfigurationBits = B100 << 12; // AINp = AIN0, AINn = GND.
    static const word voltageChannelConfigurationBits = B101 << 12; // AINp = AIN1, AINn = GND.
    static const word shuntVoltageChannelConfigurationBits = B111 << 12; // AINp = AIN3, AINn = GND.

    // Read a sample from the sensors.
    // Reassign the error flag.
    // Calls startConversion and readConvertedValue.
    word read(word channelConfigurationBits);

    // Read the last converted value.
    // Reassign the error flag.
    word readConvertedValue();

    // Start a conversion.
    void startConversion(word channelConfigurationBits);
};

// The direct energy transfer system found at the battery module.
extern ESATDirectEnergyTransferSystem DirectEnergyTransferSystem;

#endif /* ESATDirectEnergyTransferSystem_h */
