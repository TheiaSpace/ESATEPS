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

#include "ESAT_EPS-telemetry/ESAT_EPSHousekeepingTelemetry.h"
#include "ESAT_BatteryController.h"
#include "ESAT_DirectEnergyTransferSystem.h"
#include "ESAT_EPSMeasurements.h"
#include "ESAT_MaximumPowerPointTrackingDriver.h"
#include "ESAT_PowerLineSwitch.h"
#include "ESAT_SolarPanelThermometer.h"

boolean ESAT_EPSHousekeepingTelemetryClass::available()
{
  return true;
}

byte ESAT_EPSHousekeepingTelemetryClass::packetIdentifier()
{
  return PACKET_IDENTIFIER;
}

boolean ESAT_EPSHousekeepingTelemetryClass::fillUserData(ESAT_CCSDSPacket& packet)
{
  packet.writeWord(ESAT_EPSMeasurements.read3V3LineCurrent());
  packet.writeWord(ESAT_EPSMeasurements.read3V3LineVoltage());
  packet.writeWord(ESAT_EPSMeasurements.read5VLineCurrent());
  packet.writeWord(ESAT_EPSMeasurements.read5VLineVoltage());
  packet.writeWord(ESAT_EPSMeasurements.readInputLineCurrent());
  packet.writeWord(ESAT_EPSMeasurements.readInputLineVoltage());
  packet.writeWord(ESAT_EPSMeasurements.readSolarPanel1InputCurrent());
  packet.writeWord(ESAT_EPSMeasurements.readSolarPanel1OutputCurrent());
  packet.writeWord(ESAT_EPSMeasurements.readSolarPanel1Voltage());
  packet.writeWord(ESAT_EPSMeasurements.readSolarPanel2InputCurrent());
  packet.writeWord(ESAT_EPSMeasurements.readSolarPanel2OutputCurrent());
  packet.writeWord(ESAT_EPSMeasurements.readSolarPanel2Voltage());
  packet.writeByte(ESAT_PowerLine3V3Switch.read());
  packet.writeByte(ESAT_PowerLine5VSwitch.read());
  ESAT_BatteryController.error = false;
  packet.writeWord(ESAT_BatteryController.readBatteryCurrent());
  packet.writeWord(ESAT_BatteryController.readTotalBatteryVoltage());
  packet.writeWord(ESAT_BatteryController.readBattery1Voltage());
  packet.writeWord(ESAT_BatteryController.readBattery2Voltage());
  packet.writeWord(ESAT_BatteryController.readBatteryTemperature());
  packet.writeByte(ESAT_BatteryController.readBatteryRelativeStateOfCharge());
  packet.writeByte(ESAT_BatteryController.error);
  ESAT_SolarPanel1Thermometer.error = false;
  packet.writeWord(ESAT_SolarPanel1Thermometer.read());
  packet.writeByte(ESAT_SolarPanel1Thermometer.error);
  ESAT_SolarPanel2Thermometer.error = false;
  packet.writeWord(ESAT_SolarPanel2Thermometer.read());
  packet.writeByte(ESAT_SolarPanel2Thermometer.error);
  packet.writeByte(ESAT_MaximumPowerPointTrackingDriver1.getMode());
  packet.writeByte(ESAT_MaximumPowerPointTrackingDriver1.getDutyCycle());
  packet.writeByte(ESAT_MaximumPowerPointTrackingDriver2.getMode());
  packet.writeByte(ESAT_MaximumPowerPointTrackingDriver2.getDutyCycle());
  ESAT_DirectEnergyTransferSystem.error = false;
  packet.writeWord(ESAT_DirectEnergyTransferSystem.readCurrent());
  packet.writeWord(ESAT_DirectEnergyTransferSystem.readVoltage());
  packet.writeWord(ESAT_DirectEnergyTransferSystem.readShuntVoltage());
  packet.writeByte(ESAT_DirectEnergyTransferSystem.error);
  return true;
}

ESAT_EPSHousekeepingTelemetryClass ESAT_EPSHousekeepingTelemetry;
