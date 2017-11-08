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

#ifndef ESAT_DirectEnergyTransferSystem_h
#define ESAT_DirectEnergyTransferSystem_h

#include <Arduino.h>

// Interface with the direct energy transfer system.
// Use the global instance ESAT_DirectEnergyTransferSystem.
//
// The direct energy transfer system dissipates excess power from the
// solar panels.
//
// The underlying hadrware is the ADS1015 analog-to-digital converter
// from Texas Instruments measuring the INA199 current-shunt monitor
// from Texas Instruments.  Communications are done through the EPS
// I2C bus.
class ESAT_DirectEnergyTransferSystemClass
{
  public:
    // True after a read error.  Must be reset manually.
    boolean error;

    // Read the current circulating through the direct energy transfer
    // system.
    // Set the error flag on error.
    word readCurrent();

    // Read the voltage drop through the shunt resistor.
    // Set the error flag on error.
    word readShuntVoltage();

    // Read the voltage through the direct energy transfer system.
    // Set the error flag on error.
    word readVoltage();

  private:
    // Address of the sensor chip.
    static const byte ADDRESS = 0x48;

    // Registers.
    static const byte CONFIGURATION_REGISTER = 0x01;
    static const byte CONVERSION_REGISTER = 0x00;

    // Configuration bits.  The device is a TI ADS1015.
    static const word START_SINGLE_SHOT_CONVERSION_CONFIGURATION_BITS =
      B1 << 15; // Start a single shot conversion.
    static const word COMPARATOR_CONFIGURATION_BITS
      = B11 << 0; // Disable comparator.
    static const word DATA_RATE_CONFIGURATION_BITS
      = B100 << 5; // 1600 samples per second.
    static const word OPERATING_MODE_CONFIGURATION_BITS
      = B1 << 8; // Single conversion.
    static const word FULL_SCALE_RANGE_CONFIGURATION_BITS
      = B001 << 9; // +-4.096 V.

    // Channel configuration bits.
    static const word CURRENT_CHANNEL_CONFIGURATION_BITS
      = B100 << 12; // AINp = AIN0, AINn = GND.
    static const word VOLTAGE_CHANNEL_CONFIGURATION_BITS
      = B101 << 12; // AINp = AIN1, AINn = GND.
    static const word SHUNT_VOLTAGE_CHANNEL_CONFIGURATION_BITS
      = B111 << 12; // AINp = AIN3, AINn = GND.

    // Read a sample from the sensors.
    // Set the error flag on error.
    // Calls startConversion and readConvertedValue.
    word read(word channelConfigurationBits);

    // Read the last converted value.
    // Set the error flag on error.
    word readConvertedValue();

    // Start a conversion.
    // Return true on successful conversion; otherwise return false.
    boolean startConversion(word channelConfigurationBits);
};

// The direct energy transfer system found at the battery module.
extern ESAT_DirectEnergyTransferSystemClass ESAT_DirectEnergyTransferSystem;

#endif /* ESAT_DirectEnergyTransferSystem_h */
