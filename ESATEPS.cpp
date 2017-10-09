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

#include "ESATEPS.h"
#include <ESATUtil.h>
#include <ESATI2CSlave.h>
#include <USBSerial.h>
#include <Wire.h>
#include "ESATBatteryController.h"
#include "ESATDirectEnergyTransferSystem.h"
#include "ESATEPSMeasurements.h"
#include "ESATMaximumPowerPointTrackingDriver.h"
#include "ESATOvercurrentDetector.h"
#include "ESATPowerLineSwitch.h"
#include "ESATSolarPanelThermometer.h"

void ESATEPS::begin()
{
  newTelemetryPacket = false;
  telemetryPacketSequenceCount = 0;
  telemetry = ESATCCSDSPacket(telemetryPacketData,
                              TELEMETRY_PACKET_DATA_LENGTH);
  EPSMeasurements.begin();
  MaximumPowerPointTrackingDriver1.begin();
  MaximumPowerPointTrackingDriver2.begin();
  MaximumPowerPointTrackingDriver1.setMPPTMode();
  MaximumPowerPointTrackingDriver2.setMPPTMode();
  PowerLine5VSwitch.begin();
  PowerLine5VSwitch.write(PowerLine5VSwitch.OFF);
  PowerLine3V3Switch.begin();
  PowerLine3V3Switch.write(PowerLine3V3Switch.ON);
  OvercurrentDetector.begin();
  Wire1.begin();
  Wire.begin(2);
  USB.begin();
  I2CSlave.begin(Wire,
                 i2cTelecommandPacketData,
                 TELECOMMAND_PACKET_DATA_LENGTH,
                 i2cTelemetryPacketData,
                 TELEMETRY_PACKET_DATA_LENGTH);
}

void ESATEPS::handleTelecommand(ESATCCSDSPacket& packet)
{
  packet.rewind();
  if (packet.readApplicationProcessIdentifier()
      != APPLICATION_PROCESS_IDENTIFIER)
  {
    return;
  }
  if (packet.readPacketType() != packet.TELECOMMAND)
  {
    return;
  }
  if (packet.readPacketDataLength() != TELECOMMAND_PACKET_DATA_LENGTH)
  {
    return;
  }
  const byte majorVersionNumber = packet.readByte();
  const byte minorVersionNumber = packet.readByte();
  const byte patchVersionNumber = packet.readByte();
  if (majorVersionNumber < MAJOR_VERSION_NUMBER)
  {
    return;
  }
  const byte commandCode = packet.readByte();
  const byte commandParameter = packet.readByte();
  switch (commandCode)
  {
  case SWITCH_5V_LINE:
    handleSwitch5VLineCommand(commandParameter);
    break;
  case SWITCH_3V3_LINE:
    handleSwitch3V3LineCommand(commandParameter);
    break;
  case MAXIMUM_POWER_POINT_TRACKING_MODE:
    handleMaximumPowerPointTrackingModeCommand(commandParameter);
    break;
  case SWEEP_MODE:
    handleSweepModeCommand(commandParameter);
    break;
  case FIXED_MODE:
    handleFixedModeCommand(commandParameter);
    break;
  default:
    break;
  }
}

void ESATEPS::handleFixedModeCommand(const byte commandParameter)
{
  const byte dutyCycle = constrain(commandParameter, 0, 255);
  MaximumPowerPointTrackingDriver1.setFixedMode();
  MaximumPowerPointTrackingDriver2.setFixedMode();
  MaximumPowerPointTrackingDriver1.dutyCycle = dutyCycle;
  MaximumPowerPointTrackingDriver2.dutyCycle = dutyCycle;
}

void ESATEPS::handleMaximumPowerPointTrackingModeCommand(const byte commandParameter)
{
  MaximumPowerPointTrackingDriver1.setMPPTMode();
  MaximumPowerPointTrackingDriver2.setMPPTMode();
}

void ESATEPS::handleSweepModeCommand(const byte commandParameter)
{
  MaximumPowerPointTrackingDriver1.setSweepMode();
  MaximumPowerPointTrackingDriver2.setSweepMode();
}

void ESATEPS::handleSwitch3V3LineCommand(const byte commandParameter)
{
  if (commandParameter > 0)
  {
    PowerLine3V3Switch.write(PowerLine3V3Switch.ON);
  }
  else
  {
    PowerLine3V3Switch.write(PowerLine3V3Switch.OFF);
  }
}

void ESATEPS::handleSwitch5VLineCommand(const byte commandParameter)
{
  if (commandParameter > 0)
  {
    PowerLine5VSwitch.write(PowerLine5VSwitch.ON);
  }
  else
  {
    PowerLine5VSwitch.write(PowerLine5VSwitch.OFF);
  }
}

boolean ESATEPS::readTelecommand(ESATCCSDSPacket& packet)
{
  packet.clear();
  if (packet.packetDataBufferLength < TELECOMMAND_PACKET_DATA_LENGTH)
  {
    return false;
  }
  boolean pendingTelecommand = I2CSlave.readTelecommand(packet);
  if (!pendingTelecommand)
  {
    pendingTelecommand = readTelecommandFromUSB(packet);
  }
  if (!pendingTelecommand)
  {
    return false;
  }
  if (packet.readPacketType() != packet.TELECOMMAND)
  {
    return false;
  }
  if (packet.readApplicationProcessIdentifier()
      != APPLICATION_PROCESS_IDENTIFIER)
  {
    return false;
  }
  if (packet.readPacketDataLength() != TELECOMMAND_PACKET_DATA_LENGTH)
  {
    return false;
  }
  return true;
}

boolean ESATEPS::readTelecommandFromUSB(ESATCCSDSPacket& packet)
{
  if (USB.available() == 0)
  {
    return false;
  }
  return packet.readFrom(USB);
}

boolean ESATEPS::readTelemetry(ESATCCSDSPacket& packet)
{
  if (!newTelemetryPacket)
  {
    return false;
  }
  newTelemetryPacket = false;
  return telemetry.copyTo(packet);
}

void ESATEPS::sendTelemetry(ESATCCSDSPacket& packet)
{
  (void) packet.writeTo(USB);
}

void ESATEPS::update()
{
  updateMaximumPowerPointTracking();
  updateTelemetry();
  updateI2CTelemetry();
}

void ESATEPS::updateMaximumPowerPointTracking()
{
  MaximumPowerPointTrackingDriver1.update();
  MaximumPowerPointTrackingDriver2.update();
}

void ESATEPS::updateI2CTelemetry()
{
  const int packetIdentifier = I2CSlave.requestedTelemetryPacket();
  if (packetIdentifier == HOUSEKEEPING)
  {
    (void) I2CSlave.writeTelemetry(telemetry);
  }
}

void ESATEPS::updateTelemetry()
{
  telemetry.clear();
  telemetry.writePacketVersionNumber(0);
  telemetry.writePacketType(telemetry.TELEMETRY);
  telemetry.writeSecondaryHeaderFlag(telemetry.SECONDARY_HEADER_IS_PRESENT);
  telemetry.writeApplicationProcessIdentifier(APPLICATION_PROCESS_IDENTIFIER);
  telemetry.writeSequenceFlags(telemetry.UNSEGMENTED_USER_DATA);
  telemetry.writePacketSequenceCount(telemetryPacketSequenceCount);
  telemetry.writeByte(MAJOR_VERSION_NUMBER);
  telemetry.writeByte(MINOR_VERSION_NUMBER);
  telemetry.writeByte(PATCH_VERSION_NUMBER);
  telemetry.writeByte(HOUSEKEEPING);
  telemetry.writeWord(EPSMeasurements.read3V3LineCurrent());
  telemetry.writeWord(EPSMeasurements.read3V3LineVoltage());
  telemetry.writeWord(EPSMeasurements.read5VLineCurrent());
  telemetry.writeWord(EPSMeasurements.read5VLineVoltage());
  telemetry.writeWord(EPSMeasurements.readInputLineCurrent());
  telemetry.writeWord(EPSMeasurements.readInputLineVoltage());
  telemetry.writeWord(EPSMeasurements.readPanel1InputCurrent());
  telemetry.writeWord(EPSMeasurements.readPanel1OutputCurrent());
  telemetry.writeWord(EPSMeasurements.readPanel1Voltage());
  telemetry.writeWord(EPSMeasurements.readPanel2InputCurrent());
  telemetry.writeWord(EPSMeasurements.readPanel2OutputCurrent());
  telemetry.writeWord(EPSMeasurements.readPanel2Voltage());
  telemetry.writeByte(PowerLine3V3Switch.read());
  telemetry.writeByte(PowerLine5VSwitch.read());
  telemetry.writeByte(OvercurrentDetector.read3V3LineOvercurrentState());
  telemetry.writeByte(OvercurrentDetector.read5VLineOvercurrentState());
  BatteryController.error = false;
  telemetry.writeWord(BatteryController.readBatteryCurrent());
  telemetry.writeWord(BatteryController.readTotalBatteryVoltage());
  telemetry.writeWord(BatteryController.readBattery1Voltage());
  telemetry.writeWord(BatteryController.readBattery2Voltage());
  telemetry.writeWord(BatteryController.readBatteryTemperature());
  telemetry.writeByte(BatteryController.readStateOfCharge());
  telemetry.writeByte(BatteryController.error);
  SolarPanel1Thermometer.error = false;
  telemetry.writeWord(SolarPanel1Thermometer.read());
  telemetry.writeByte(SolarPanel1Thermometer.error);
  SolarPanel2Thermometer.error = false;
  telemetry.writeWord(SolarPanel2Thermometer.read());
  telemetry.writeByte(SolarPanel2Thermometer.error);
  telemetry.writeByte(MaximumPowerPointTrackingDriver1.getMode());
  telemetry.writeByte(MaximumPowerPointTrackingDriver1.getDutyCycle());
  telemetry.writeByte(MaximumPowerPointTrackingDriver2.getMode());
  telemetry.writeByte(MaximumPowerPointTrackingDriver2.getDutyCycle());
  DirectEnergyTransferSystem.error = false;
  telemetry.writeWord(DirectEnergyTransferSystem.readCurrent());
  telemetry.writeWord(DirectEnergyTransferSystem.readVoltage());
  telemetry.writeWord(DirectEnergyTransferSystem.readShuntVoltage());
  telemetry.writeByte(DirectEnergyTransferSystem.error);
  telemetryPacketSequenceCount = telemetryPacketSequenceCount + 1;
  newTelemetryPacket = true;
}

ESATEPS EPS;
