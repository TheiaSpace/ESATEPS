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
  pendingTelecommand = false;
  telemetryBufferIndex = 0;
  telemetryPacketSequenceCount = 0;
  currentTelemetryBuffer = 0;
  EPSMeasurements.begin();
  MaximumPowerPointTrackingDriver1.begin();
  MaximumPowerPointTrackingDriver2.begin();
  MaximumPowerPointTrackingDriver1.setMPPTMode();
  MaximumPowerPointTrackingDriver2.setMPPTMode();
  PowerLine5VSwitch.begin();
  PowerLine5VSwitch.write(PowerLine5VSwitch.off);
  PowerLine3V3Switch.begin();
  PowerLine3V3Switch.write(PowerLine3V3Switch.on);
  OvercurrentDetector.begin();
  Wire1.begin();
  Wire.begin(2);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  USB.begin();
}

void ESATEPS::handleCommands()
{
  if (!pendingTelecommand)
  {
    return;
  }
  byte buffer[COMMAND_PACKET_LENGTH];
  for (int index = 0; index < COMMAND_PACKET_LENGTH; index++)
  {
    buffer[index] = telecommandBuffer[index];
  }
  pendingTelecommand = false;
  ESATCCSDSPacket telecommand(buffer);
  if (telecommand.readApplicationProcessIdentifier() != SUBSYSTEM_IDENTIFIER)
  {
    return;
  }
  if ((telecommand.PRIMARY_HEADER_LENGTH + telecommand.readPacketDataLength())
      != COMMAND_PACKET_LENGTH)
  {
    return;
  }
  const byte majorVersionNumber = telecommand.readByte();
  const byte minorVersionNumber = telecommand.readByte();
  const byte patchVersionNumber = telecommand.readByte();
  if (majorVersionNumber < MAJOR_VERSION_NUMBER)
  {
    return;
  }
  const byte commandCode = telecommand.readByte();
  const byte commandParameter = telecommand.readByte();
  switch (commandCode)
  {
  case TOGGLE_5V_LINE:
    handleToggle5VLineCommand(commandParameter);
    break;
  case TOGGLE_3V3_LINE:
    handleToggle3V3LineCommand(commandParameter);
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

void ESATEPS::handleToggle3V3LineCommand(const byte commandParameter)
{
  PowerLine3V3Switch.toggle();
}

void ESATEPS::handleToggle5VLineCommand(const byte commandParameter)
{
  PowerLine5VSwitch.toggle();
}

boolean ESATEPS::pendingCommands()
{
  receiveTelecommandFromUSB();
  return pendingTelecommand;
}

void ESATEPS::receiveEvent(const int howManyBytes)
{
  const byte registerNumber = Wire.read();
  switch (registerNumber)
  {
  case TELECOMMAND_CONTROL:
    EPS.receiveTelecommandFromI2C(howManyBytes - 1);
    break;
  case TELEMETRY_CONTROL:
    EPS.telemetryBufferIndex = 0;
    break;
  case TELEMETRY_VECTOR:
    EPS.telemetryBufferIndex = 0;
    break;
  }
}

void ESATEPS::requestEvent()
{
  Wire.write(EPS.telemetryBuffer[EPS.currentTelemetryBuffer][EPS.telemetryBufferIndex]);
}

void ESATEPS::sendTelemetry()
{
  USB.write(telemetryBuffer[currentTelemetryBuffer],
            TELEMETRY_BUFFER_LENGTH);
}

void ESATEPS::receiveTelecommandFromI2C(const int packetLength)
{
  if (pendingTelecommand)
  {
    return;
  }
  if (packetLength != COMMAND_PACKET_LENGTH)
  {
    return;
  }
  for (int index = 0; index < COMMAND_PACKET_LENGTH; index++)
  {
    telecommandBuffer[index] = Wire.read();
  }
  pendingTelecommand = true;
}

void::ESATEPS::receiveTelecommandFromUSB()
{
  if (pendingTelecommand)
  {
    return;
  }
  if (USB.available() == 0)
  {
    return;
  }
  char buffer[COMMAND_PACKET_LENGTH];
  const size_t bytesRead = USB.readBytes(buffer, COMMAND_PACKET_LENGTH);
  if (bytesRead != COMMAND_PACKET_LENGTH)
  {
    return;
  }
  for (int index = 0; index < COMMAND_PACKET_LENGTH; index++)
  {
    telecommandBuffer[index] = buffer[index];
  }
  pendingTelecommand = true;
}

void ESATEPS::updateMaximumPowerPointTracking()
{
  MaximumPowerPointTrackingDriver1.update();
  MaximumPowerPointTrackingDriver2.update();
}

void ESATEPS::updateTelemetry()
{
  const byte nextTelemetryBuffer = (currentTelemetryBuffer + 1) % 2;
  ESATCCSDSPacket packet(telemetryBuffer[nextTelemetryBuffer]);
  packet.clear();
  packet.writePacketVersionNumber(0);
  packet.writePacketType(packet.TELEMETRY);
  packet.writeSecondaryHeaderFlag(packet.SECONDARY_HEADER_IS_PRESENT);
  packet.writeApplicationProcessIdentifier(SUBSYSTEM_IDENTIFIER);
  packet.writeSequenceFlags(packet.UNSEGMENTED_USER_DATA);
  packet.writePacketSequenceCount(telemetryPacketSequenceCount);
  packet.writeByte(MAJOR_VERSION_NUMBER);
  packet.writeByte(MINOR_VERSION_NUMBER);
  packet.writeByte(PATCH_VERSION_NUMBER);
  packet.writeByte(HOUSEKEEPING);
  packet.writeWord(EPSMeasurements.read3V3LineCurrent());
  packet.writeWord(EPSMeasurements.read3V3LineVoltage());
  packet.writeWord(EPSMeasurements.read5VLineCurrent());
  packet.writeWord(EPSMeasurements.read5VLineVoltage());
  packet.writeWord(EPSMeasurements.readInputLineCurrent());
  packet.writeWord(EPSMeasurements.readInputLineVoltage());
  packet.writeWord(EPSMeasurements.readPanel1InputCurrent());
  packet.writeWord(EPSMeasurements.readPanel1OutputCurrent());
  packet.writeWord(EPSMeasurements.readPanel1Voltage());
  packet.writeWord(EPSMeasurements.readPanel2InputCurrent());
  packet.writeWord(EPSMeasurements.readPanel2OutputCurrent());
  packet.writeWord(EPSMeasurements.readPanel2Voltage());
  packet.writeByte(PowerLine3V3Switch.read());
  packet.writeByte(PowerLine5VSwitch.read());
  packet.writeByte(OvercurrentDetector.read3V3LineOvercurrentState());
  packet.writeByte(OvercurrentDetector.read5VLineOvercurrentState());
  BatteryController.error = false;
  packet.writeWord(BatteryController.readBatteryCurrent());
  packet.writeWord(BatteryController.readTotalBatteryVoltage());
  packet.writeWord(BatteryController.readBattery1Voltage());
  packet.writeWord(BatteryController.readBattery2Voltage());
  packet.writeWord(BatteryController.readBatteryTemperature());
  packet.writeByte(BatteryController.readStateOfCharge());
  packet.writeByte(BatteryController.error);
  SolarPanel1Thermometer.error = false;
  packet.writeWord(SolarPanel1Thermometer.read());
  packet.writeByte(SolarPanel1Thermometer.error);
  SolarPanel2Thermometer.error = false;
  packet.writeWord(SolarPanel2Thermometer.read());
  packet.writeByte(SolarPanel2Thermometer.error);
  packet.writeByte(MaximumPowerPointTrackingDriver1.getMode());
  packet.writeByte(MaximumPowerPointTrackingDriver1.getDutyCycle());
  packet.writeByte(MaximumPowerPointTrackingDriver2.getMode());
  packet.writeByte(MaximumPowerPointTrackingDriver2.getDutyCycle());
  DirectEnergyTransferSystem.error = false;
  packet.writeWord(DirectEnergyTransferSystem.readCurrent());
  packet.writeWord(DirectEnergyTransferSystem.readVoltage());
  packet.writeWord(DirectEnergyTransferSystem.readShuntVoltage());
  packet.writeByte(DirectEnergyTransferSystem.error);
  telemetryPacketSequenceCount = telemetryPacketSequenceCount + 1;
  currentTelemetryBuffer = nextTelemetryBuffer;
}

ESATEPS EPS;
