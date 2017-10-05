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
  currentTelemetryBuffer = 0;
  i2cTelemetryBufferIndex = TELEMETRY_BUFFER_LENGTH;
  newTelemetryPacket = false;
  pendingI2CTelecommand = false;
  telemetryPacketSequenceCount = 0;
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
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  USB.begin();
}

void ESATEPS::handleTelecommand(ESATCCSDSPacket& packet)
{
  packet.rewind();
  if (packet.readApplicationProcessIdentifier() != SUBSYSTEM_IDENTIFIER)
  {
    return;
  }
  if (packet.readPacketType() != packet.TELECOMMAND)
  {
    return;
  }
  if ((packet.PRIMARY_HEADER_LENGTH + packet.readPacketDataLength())
      != COMMAND_PACKET_LENGTH)
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
  if (packet.bufferLength < COMMAND_PACKET_LENGTH)
  {
    return false;
  }
  if (pendingI2CTelecommand)
  {
    readTelecommandFromI2C(packet);
  }
  else
  {
    readTelecommandFromUSB(packet);
  }
  if (packet.readPacketType() != packet.TELECOMMAND)
  {
    packet.clear();
    return false;
  }
  if (packet.readApplicationProcessIdentifier() != SUBSYSTEM_IDENTIFIER)
  {
    packet.clear();
    return false;
  }
  if ((packet.PRIMARY_HEADER_LENGTH + packet.readPacketDataLength())
      != COMMAND_PACKET_LENGTH)
  {
    packet.clear();
    return false;
  }
  return true;
}

void ESATEPS::readTelecommandFromI2C(ESATCCSDSPacket& packet)
{
  for (int i = 0; i < COMMAND_PACKET_LENGTH; i++)
  {
    packet.buffer[i] = i2cTelecommandBuffer[i];
  }
  pendingI2CTelecommand = false;
}

void ESATEPS::readTelecommandFromUSB(ESATCCSDSPacket& packet)
{
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
    packet.buffer[index] = buffer[index];
  }
}

boolean ESATEPS::readTelemetry(ESATCCSDSPacket& packet)
{
  packet.clear();
  if (!newTelemetryPacket)
  {
    return false;
  }
  newTelemetryPacket = false;
  if (packet.bufferLength < TELEMETRY_BUFFER_LENGTH)
  {
    return false;
  }
  for (int i = 0; i < TELEMETRY_BUFFER_LENGTH; i++)
  {
    packet.buffer[i] = telemetryDoubleBuffer[currentTelemetryBuffer][i];
  }
  if (packet.readPacketType() != packet.TELEMETRY)
  {
    packet.clear();
    return false;
  }
  if (packet.readApplicationProcessIdentifier() != SUBSYSTEM_IDENTIFIER)
  {
    packet.clear();
    return false;
  }
  if (packet.readPacketLength() != TELEMETRY_BUFFER_LENGTH)
  {
    packet.clear();
    return false;
  }
  return true;
}

void ESATEPS::receiveEvent(const int howManyBytes)
{
  const byte registerNumber = Wire.read();
  byte buffer[howManyBytes - 1];
  for (int i = 0; i < howManyBytes; i++)
  {
    buffer[i] = Wire.read();
  }
  switch (registerNumber)
  {
  case TELECOMMAND_CONTROL:
    EPS.receiveTelecommandFromI2C(buffer, howManyBytes - 1);
    break;
  case TELEMETRY_CONTROL:
    EPS.receiveTelemetryRequestFromI2C(buffer, howManyBytes - 1);
    break;
  }
}

void ESATEPS::receiveTelecommandFromI2C(const byte packet[],
                                        const int packetLength)
{
  if (pendingI2CTelecommand)
  {
    return;
  }
  if (packetLength != COMMAND_PACKET_LENGTH)
  {
    return;
  }
  for (int index = 0; index < COMMAND_PACKET_LENGTH; index++)
  {
    i2cTelecommandBuffer[index] = packet[index];
  }
  pendingI2CTelecommand = true;
}

void ESATEPS::receiveTelemetryRequestFromI2C(const byte request[],
                                             const int requestLength)
{
  if (requestLength < 4)
  {
    EPS.i2cTelemetryBufferIndex = TELEMETRY_BUFFER_LENGTH;
    return;
  }
  const byte packetIdentifier = request[0];
  if (packetIdentifier != HOUSEKEEPING)
  {
    EPS.i2cTelemetryBufferIndex = TELEMETRY_BUFFER_LENGTH;
    return;
  }
  const boolean newPacket = request[1];
  if (newPacket)
  {
    for (int i = 0; i < TELEMETRY_BUFFER_LENGTH; i++)
    {
      EPS.i2cTelemetryBuffer[i] =
        EPS.telemetryDoubleBuffer[currentTelemetryBuffer][i];
    }
  }
  const byte indexHighByte = request[2];
  const byte indexLowByte = request[3];
  EPS.i2cTelemetryBufferIndex = word(indexHighByte, indexLowByte);
}

void ESATEPS::requestEvent()
{
  for (int i = 0; i < BUFFER_LENGTH; i++)
  {
    if (EPS.i2cTelemetryBufferIndex >= TELEMETRY_BUFFER_LENGTH)
    {
      return;
    }
    (void) Wire.write(EPS.i2cTelemetryBuffer[EPS.i2cTelemetryBufferIndex]);
    EPS.i2cTelemetryBufferIndex = EPS.i2cTelemetryBufferIndex + 1;
  }
}

void ESATEPS::sendTelemetry(ESATCCSDSPacket& packet)
{
  const long packetLength =
    packet.PRIMARY_HEADER_LENGTH + packet.readPacketDataLength();
  for (long i = 0; i < packetLength; i++)
  {
    (void) USB.write(packet.buffer[i]);
  }
}

void ESATEPS::update()
{
  updateMaximumPowerPointTracking();
  updateTelemetry();
}

void ESATEPS::updateMaximumPowerPointTracking()
{
  MaximumPowerPointTrackingDriver1.update();
  MaximumPowerPointTrackingDriver2.update();
}

void ESATEPS::updateTelemetry()
{
  const byte nextTelemetryBuffer = ((currentTelemetryBuffer + 1) % 2);
  ESATCCSDSPacket packet(telemetryDoubleBuffer[nextTelemetryBuffer],
                         TELEMETRY_BUFFER_LENGTH);
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
  newTelemetryPacket = true;
}

ESATEPS EPS;
