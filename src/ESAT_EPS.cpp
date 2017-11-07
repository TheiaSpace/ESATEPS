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

#include "ESAT_EPS.h"
#include <ESAT_CCSDSPrimaryHeader.h>
#include <ESAT_I2CSlave.h>
#include <ESAT_KISSStream.h>
#include <ESAT_Timestamp.h>
#include <USBSerial.h>
#include <Wire.h>
#include "ESAT_BatteryController.h"
#include "ESAT_DirectEnergyTransferSystem.h"
#include "ESAT_EPSMeasurements.h"
#include "ESAT_MaximumPowerPointTrackingDriver.h"
#include "ESAT_PowerLineSwitch.h"
#include "ESAT_SolarPanelThermometer.h"

void ESAT_EPSClass::begin()
{
  newTelemetryPacket = false;
  telemetryPacketSequenceCount = 0;
  telemetry = ESAT_CCSDSPacket(telemetryPacketData,
                               TELEMETRY_PACKET_DATA_LENGTH);
  usbTelecommandDecoder = ESAT_KISSStream(USB,
                                          usbTelecommandBuffer,
                                          sizeof(usbTelecommandBuffer));
  ESAT_EPSMeasurements.begin();
  ESAT_MaximumPowerPointTrackingDriver1.begin();
  ESAT_MaximumPowerPointTrackingDriver2.begin();
  ESAT_MaximumPowerPointTrackingDriver1.setMPPTMode();
  ESAT_MaximumPowerPointTrackingDriver2.setMPPTMode();
  ESAT_PowerLine5VSwitch.begin();
  ESAT_PowerLine5VSwitch.write(ESAT_PowerLine5VSwitch.OFF);
  ESAT_PowerLine3V3Switch.begin();
  ESAT_PowerLine3V3Switch.write(ESAT_PowerLine3V3Switch.ON);
  Wire1.begin();
  Wire.begin(byte(APPLICATION_PROCESS_IDENTIFIER));
  USB.begin();
  ESAT_I2CSlave.begin(Wire,
                      i2cTelecommandPacketData,
                      MAXIMUM_TELECOMMAND_PACKET_DATA_LENGTH,
                      i2cTelemetryPacketData,
                      TELEMETRY_PACKET_DATA_LENGTH);
}

void ESAT_EPSClass::handleTelecommand(ESAT_CCSDSPacket& packet)
{
  packet.rewind();
  const ESAT_CCSDSPrimaryHeader primaryHeader = packet.readPrimaryHeader();
  if (primaryHeader.applicationProcessIdentifier
      != APPLICATION_PROCESS_IDENTIFIER)
  {
    return;
  }
  if (primaryHeader.packetType != primaryHeader.TELECOMMAND)
  {
    return;
  }
  if (primaryHeader.packetDataLength < ESAT_CCSDSSecondaryHeader::LENGTH)
  {
    return;
  }
  const ESAT_CCSDSSecondaryHeader secondaryHeader =
    packet.readSecondaryHeader();
  if (secondaryHeader.majorVersionNumber < MAJOR_VERSION_NUMBER)
  {
    return;
  }
  switch (secondaryHeader.packetIdentifier)
  {
    case SWITCH_5V_LINE:
      handleSwitch5VLineCommand(packet);
      break;
    case SWITCH_3V3_LINE:
      handleSwitch3V3LineCommand(packet);
      break;
    case MAXIMUM_POWER_POINT_TRACKING_MODE:
      handleMaximumPowerPointTrackingModeCommand(packet);
      break;
    case SWEEP_MODE:
      handleSweepModeCommand(packet);
      break;
    case FIXED_MODE:
      handleFixedModeCommand(packet);
      break;
    case SET_CURRENT_TIME:
      handleSetCurrentTimeCommand(packet);
      break;
    default:
      break;
  }
}

void ESAT_EPSClass::handleFixedModeCommand(ESAT_CCSDSPacket& packet)
{
  const byte commandParameter = packet.readByte();
  const byte dutyCycle = constrain(commandParameter, 0, 255);
  ESAT_MaximumPowerPointTrackingDriver1.setFixedMode();
  ESAT_MaximumPowerPointTrackingDriver2.setFixedMode();
  ESAT_MaximumPowerPointTrackingDriver1.dutyCycle = dutyCycle;
  ESAT_MaximumPowerPointTrackingDriver2.dutyCycle = dutyCycle;
}

void ESAT_EPSClass::handleMaximumPowerPointTrackingModeCommand(ESAT_CCSDSPacket& packet)
{
  ESAT_MaximumPowerPointTrackingDriver1.setMPPTMode();
  ESAT_MaximumPowerPointTrackingDriver2.setMPPTMode();
}

void ESAT_EPSClass::handleSweepModeCommand(ESAT_CCSDSPacket& packet)
{
  ESAT_MaximumPowerPointTrackingDriver1.setSweepMode();
  ESAT_MaximumPowerPointTrackingDriver2.setSweepMode();
}

void ESAT_EPSClass::handleSwitch3V3LineCommand(ESAT_CCSDSPacket& packet)
{
  const byte commandParameter = packet.readByte();
  if (commandParameter > 0)
  {
    ESAT_PowerLine3V3Switch.write(ESAT_PowerLine3V3Switch.ON);
  }
  else
  {
    ESAT_PowerLine3V3Switch.write(ESAT_PowerLine3V3Switch.OFF);
  }
}

void ESAT_EPSClass::handleSwitch5VLineCommand(ESAT_CCSDSPacket& packet)
{
  const byte commandParameter = packet.readByte();
  if (commandParameter > 0)
  {
    ESAT_PowerLine5VSwitch.write(ESAT_PowerLine5VSwitch.ON);
  }
  else
  {
    ESAT_PowerLine5VSwitch.write(ESAT_PowerLine5VSwitch.OFF);
  }
}

void ESAT_EPSClass::handleSetCurrentTimeCommand(ESAT_CCSDSPacket& packet)
{
  const ESAT_Timestamp timestamp = packet.readTimestamp();
  clock.write(timestamp);
}

boolean ESAT_EPSClass::readTelecommand(ESAT_CCSDSPacket& packet)
{
  packet.clear();
  if (packet.capacity() < ESAT_CCSDSSecondaryHeader::LENGTH)
  {
    return false;
  }
  boolean pendingTelecommand = ESAT_I2CSlave.readTelecommand(packet);
  if (!pendingTelecommand)
  {
    pendingTelecommand = readTelecommandFromUSB(packet);
  }
  if (!pendingTelecommand)
  {
    return false;
  }
  const ESAT_CCSDSPrimaryHeader primaryHeader = packet.readPrimaryHeader();
  if (primaryHeader.packetType != primaryHeader.TELECOMMAND)
  {
    return false;
  }
  if (primaryHeader.applicationProcessIdentifier
      != APPLICATION_PROCESS_IDENTIFIER)
  {
    return false;
  }
  if (primaryHeader.packetDataLength < ESAT_CCSDSSecondaryHeader::LENGTH)
  {
    return false;
  }
  return true;
}

boolean ESAT_EPSClass::readTelecommandFromUSB(ESAT_CCSDSPacket& packet)
{
  const boolean gotFrame = usbTelecommandDecoder.receiveFrame();
  if (!gotFrame)
  {
    return false;
  }
  return packet.readFrom(usbTelecommandDecoder);
}

boolean ESAT_EPSClass::readTelemetry(ESAT_CCSDSPacket& packet)
{
  if (!newTelemetryPacket)
  {
    return false;
  }
  newTelemetryPacket = false;
  return telemetry.copyTo(packet);
}

void ESAT_EPSClass::update()
{
  updateMaximumPowerPointTracking();
  updateTelemetry();
  updateI2CTelemetry();
}

void ESAT_EPSClass::updateMaximumPowerPointTracking()
{
  ESAT_MaximumPowerPointTrackingDriver1.update();
  ESAT_MaximumPowerPointTrackingDriver2.update();
}

void ESAT_EPSClass::updateI2CTelemetry()
{
  const int packetIdentifier = ESAT_I2CSlave.requestedTelemetryPacket();
  switch (packetIdentifier)
  {
    case ESAT_I2CSlave.NO_TELEMETRY_PACKET_REQUESTED:
      break;
    case HOUSEKEEPING:
      (void) ESAT_I2CSlave.writeTelemetry(telemetry);
      break;
    default:
      ESAT_I2CSlave.rejectTelemetryRequest();
      break;
  }
}

void ESAT_EPSClass::updateTelemetry()
{
  telemetry.clear();
  // Primary header.
  ESAT_CCSDSPrimaryHeader primaryHeader;
  primaryHeader.packetVersionNumber = 0;
  primaryHeader.packetType =
    primaryHeader.TELEMETRY;
  primaryHeader.secondaryHeaderFlag =
    primaryHeader.SECONDARY_HEADER_IS_PRESENT;
  primaryHeader.applicationProcessIdentifier =
    APPLICATION_PROCESS_IDENTIFIER;
  primaryHeader.sequenceFlags =
    primaryHeader.UNSEGMENTED_USER_DATA;
  primaryHeader.packetSequenceCount =
    telemetryPacketSequenceCount;
  telemetry.writePrimaryHeader(primaryHeader);
  // Secondary header.
  ESAT_CCSDSSecondaryHeader secondaryHeader;
  secondaryHeader.preamble =
    secondaryHeader.CALENDAR_SEGMENTED_TIME_CODE_MONTH_DAY_VARIANT_1_SECOND_RESOLUTION;
  secondaryHeader.timestamp = clock.read();
  secondaryHeader.majorVersionNumber = MAJOR_VERSION_NUMBER;
  secondaryHeader.minorVersionNumber = MINOR_VERSION_NUMBER;
  secondaryHeader.patchVersionNumber = PATCH_VERSION_NUMBER;
  secondaryHeader.packetIdentifier = HOUSEKEEPING;
  telemetry.writeSecondaryHeader(secondaryHeader);
  // User data.
  telemetry.writeWord(ESAT_EPSMeasurements.read3V3LineCurrent());
  telemetry.writeWord(ESAT_EPSMeasurements.read3V3LineVoltage());
  telemetry.writeWord(ESAT_EPSMeasurements.read5VLineCurrent());
  telemetry.writeWord(ESAT_EPSMeasurements.read5VLineVoltage());
  telemetry.writeWord(ESAT_EPSMeasurements.readInputLineCurrent());
  telemetry.writeWord(ESAT_EPSMeasurements.readInputLineVoltage());
  telemetry.writeWord(ESAT_EPSMeasurements.readSolarPanel1InputCurrent());
  telemetry.writeWord(ESAT_EPSMeasurements.readSolarPanel1OutputCurrent());
  telemetry.writeWord(ESAT_EPSMeasurements.readSolarPanel1Voltage());
  telemetry.writeWord(ESAT_EPSMeasurements.readSolarPanel2InputCurrent());
  telemetry.writeWord(ESAT_EPSMeasurements.readSolarPanel2OutputCurrent());
  telemetry.writeWord(ESAT_EPSMeasurements.readSolarPanel2Voltage());
  telemetry.writeByte(ESAT_PowerLine3V3Switch.read());
  telemetry.writeByte(ESAT_PowerLine5VSwitch.read());
  ESAT_BatteryController.error = false;
  telemetry.writeWord(ESAT_BatteryController.readBatteryCurrent());
  telemetry.writeWord(ESAT_BatteryController.readTotalBatteryVoltage());
  telemetry.writeWord(ESAT_BatteryController.readBattery1Voltage());
  telemetry.writeWord(ESAT_BatteryController.readBattery2Voltage());
  telemetry.writeWord(ESAT_BatteryController.readBatteryTemperature());
  telemetry.writeByte(ESAT_BatteryController.readBatteryStateOfCharge());
  telemetry.writeByte(ESAT_BatteryController.error);
  ESAT_SolarPanel1Thermometer.error = false;
  telemetry.writeWord(ESAT_SolarPanel1Thermometer.read());
  telemetry.writeByte(ESAT_SolarPanel1Thermometer.error);
  ESAT_SolarPanel2Thermometer.error = false;
  telemetry.writeWord(ESAT_SolarPanel2Thermometer.read());
  telemetry.writeByte(ESAT_SolarPanel2Thermometer.error);
  telemetry.writeByte(ESAT_MaximumPowerPointTrackingDriver1.getMode());
  telemetry.writeByte(ESAT_MaximumPowerPointTrackingDriver1.getDutyCycle());
  telemetry.writeByte(ESAT_MaximumPowerPointTrackingDriver2.getMode());
  telemetry.writeByte(ESAT_MaximumPowerPointTrackingDriver2.getDutyCycle());
  ESAT_DirectEnergyTransferSystem.error = false;
  telemetry.writeWord(ESAT_DirectEnergyTransferSystem.readCurrent());
  telemetry.writeWord(ESAT_DirectEnergyTransferSystem.readVoltage());
  telemetry.writeWord(ESAT_DirectEnergyTransferSystem.readShuntVoltage());
  telemetry.writeByte(ESAT_DirectEnergyTransferSystem.error);
  // End of user data
  telemetry.flush();
  telemetryPacketSequenceCount = telemetryPacketSequenceCount + 1;
  newTelemetryPacket = true;
}

void ESAT_EPSClass::writeTelemetry(ESAT_CCSDSPacket& packet)
{
  packet.rewind();
  const unsigned long encoderBufferLength =
    ESAT_KISSStream::frameLength(packet.length());
  byte encoderBuffer[encoderBufferLength];
  ESAT_KISSStream encoder(USB, encoderBuffer, encoderBufferLength);
  (void) encoder.beginFrame();
  (void) packet.writeTo(encoder);
  (void) encoder.endFrame();
}

ESAT_EPSClass ESAT_EPS;
