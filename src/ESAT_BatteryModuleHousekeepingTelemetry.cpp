/*
 * Copyright (C) 2018 Theia Space, Universidad Politécnica de Madrid
 *
 * This file is part of Theia Space's ESAT Util library.
 *
 * Theia Space's ESAT Util library is free software: you can
 * redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Theia Space's ESAT Util library is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Theia Space's ESAT Util library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "ESAT_BatteryModuleHousekeepingTelemetry.h"
#include "ESAT_BatteryController.h"

boolean ESAT_BatteryModuleHousekeepingTelemetryClass::available()
{
  const unsigned long currentPacketTime = millis();
  if (currentPacketTime >= (previousPacketTime + PERIOD))
  {
    previousPacketTime = currentPacketTime;
    return true;
  }
  else
  {
    return false;
  }
}

byte ESAT_BatteryModuleHousekeepingTelemetryClass::packetIdentifier()
{
  return PACKET_IDENTIFIER;
}

boolean ESAT_BatteryModuleHousekeepingTelemetryClass::fillUserData(ESAT_CCSDSPacket& packet)
{
  packet.writeUnsignedLong(ESAT_BatteryController.readOperationStatus());
  packet.writeUnsignedLong(ESAT_BatteryController.readChargingStatus());
  packet.writeUnsignedLong(ESAT_BatteryController.readManufacturingStatus());
  packet.writeUnsignedLong(ESAT_BatteryController.readSafetyStatus());
  packet.writeWord(ESAT_BatteryController.readBatteryCurrent());
  packet.writeWord(ESAT_BatteryController.readTotalBatteryVoltage());
  packet.writeWord(ESAT_BatteryController.readBattery1Voltage());
  packet.writeWord(ESAT_BatteryController.readBattery2Voltage());
  packet.writeWord(ESAT_BatteryController.readBatteryTemperature());
  packet.writeWord(ESAT_BatteryController.readMicrocontrollerTemperature());
  packet.writeByte(ESAT_BatteryController.readBatteryRelativeStateOfCharge());
  packet.writeByte(ESAT_BatteryController.readBatteryAbsoluteStateOfCharge());
  packet.writeWord(ESAT_BatteryController.readDesiredChargingCurrent());
  packet.writeWord(ESAT_BatteryController.readDesiredChargingVoltage());
  packet.writeWord(ESAT_BatteryController.readCycleCount());
  packet.writeByte(ESAT_BatteryController.error);
  packet.writeWord(ESAT_BatteryController.readSerialNumber());
  packet.writeWord(ESAT_BatteryController.readDesignCapacity());
  packet.writeWord(ESAT_BatteryController.readDesignVoltage());
  packet.writeUnsignedLong(ESAT_BatteryController.readEnabledProtections());
  packet.writeByte(ESAT_BatteryController.readDeviceConfiguration());
  packet.writeByte(ESAT_BatteryController.readBalancingConfiguration());
  packet.writeWord(ESAT_BatteryController.readCellUndervoltageThreshold());
  packet.writeByte(ESAT_BatteryController.readCellUndervoltageRecoveryDelay());
  packet.writeWord(ESAT_BatteryController.readCellUndervoltageRecoveryThreshold());
  packet.writeWord(ESAT_BatteryController.readCellOvervoltageThreshold());
  packet.writeByte(ESAT_BatteryController.readCellOvervoltageRecoveryDelay());
  packet.writeWord(ESAT_BatteryController.readCellOvervoltageRecoveryThreshold());
  packet.writeWord(ESAT_BatteryController.readChemicalIdentifier());
  const ESAT_BatteryControllerFirmwareVersion firmwareVersion =
    ESAT_BatteryController.readFirmwareVersion();
  (void) firmwareVersion.writeTo(packet);
  return true;
}

ESAT_BatteryModuleHousekeepingTelemetryClass ESAT_BatteryModuleHousekeepingTelemetry;
