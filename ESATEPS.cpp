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

ESATEPS::ESATEPS()
{
}

void ESATEPS::begin()
{
  Flash.read(flash, &identifier, sizeof(identifier));
  command.pending = false;
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

void ESATEPS::handleCommand()
{
  if (command.pending)
  {
    switch (command.commandCode)
    {
    case SET_IDENTIFIER:
      handleSetIdentifierCommand();
      break;
    case TOGGLE_5V_LINE:
      handleToggle5VLineCommand();
      break;
    case TOGGLE_3V3_LINE:
      handleToggle3V3LineCommand();
      break;
    case MAXIMUM_POWER_POINT_TRACKING_MODE:
      handleMaximumPowerPointTrackingModeCommand();
      break;
    case SWEEP_MODE:
      handleSweepModeCommand();
      break;
    case FIXED_MODE:
      handleFixedModeCommand();
      break;
    default:
      break;
    }
    command.pending = false;
  }
}

void ESATEPS::handleFixedModeCommand()
{
  const byte dutyCycle = constrain(command.parameter, 0, 255);
  MaximumPowerPointTrackingDriver1.setFixedMode();
  MaximumPowerPointTrackingDriver2.setFixedMode();
  MaximumPowerPointTrackingDriver1.dutyCycle = dutyCycle;
  MaximumPowerPointTrackingDriver2.dutyCycle = dutyCycle;
}

void ESATEPS::handleMaximumPowerPointTrackingModeCommand()
{
  MaximumPowerPointTrackingDriver1.setMPPTMode();
  MaximumPowerPointTrackingDriver2.setMPPTMode();
}

void ESATEPS::handleSetIdentifierCommand()
{
  Flash.erase(flash);
  identifier = command.parameter;
  unsigned char p;
  p = command.parameter;
  Flash.write(flash, &p ,1);
}

void ESATEPS::handleSweepModeCommand()
{
  MaximumPowerPointTrackingDriver1.setSweepMode();
  MaximumPowerPointTrackingDriver2.setSweepMode();
}

void ESATEPS::handleToggle3V3LineCommand()
{
  PowerLine3V3Switch.toggle();
}

void ESATEPS::handleToggle5VLineCommand()
{
  PowerLine5VSwitch.toggle();
}

void ESATEPS::queueCommand(const byte commandCode, const byte parameter)
{
  if (!command.pending)
  {
    command.commandCode = commandCode;
    command.parameter = parameter;
    command.pending = true;
  }
}

void ESATEPS::queueIncomingUSBCommands()
{
  while (USB.available())
  {
    const String packet = USB.readStringUntil('\r');
    const String identifier = packet.substring(0, 1);
    if (identifier == "@")
    {
      const byte commandCode = Util.hexadecimalToByte(packet.substring(5, 7));
      const byte length = Util.hexadecimalToByte(packet.substring(3, 5));
      const byte parameter = Util.hexadecimalToByte(packet.substring(7, 7 + length));
      queueCommand(commandCode, parameter);
    }
  }
}

void ESATEPS::receiveEvent(const int howMany)
{
  const int commandCode = Wire.read();
  if (commandCode < 0)
  {
    return;
  }
  if (howMany > 1)
  {
    const int parameter = Wire.read();
    if (parameter < 0)
    {
      return;
    }
    EPS.queueCommand(commandCode, parameter);
  }
  else
  {
    EPS.queueCommand(commandCode, 0);
  }
}

void ESATEPS::requestEvent()
{
  for (int index = 0; index < TELEMETRY_BUFFER_LENGTH; index++)
  {
    Wire.write(highByte(EPS.telemetry[index]));
    Wire.write(lowByte(EPS.telemetry[index]));
  }
}

void ESATEPS::sendTelemetry()
{
  const byte type = 1;
  // build packet with given data (hex), type
  // ID(b3)|TM/TC(b1)|APID(h1)|length(h2)|type(h2)|data|CRC(h2)
  String packet = "";
  packet += Util.byteToHexadecimal(identifier).substring(1, 2);
  packet += "2";
  packet += Util.byteToHexadecimal((TELEMETRY_BUFFER_LENGTH + 1) * 2);
  packet += Util.byteToHexadecimal(type);
  for (int index = 0; index < TELEMETRY_BUFFER_LENGTH; index++)
  {
    packet += Util.wordToHexadecimal(telemetry[index]);
  }
  packet += "00000000";
  packet += "FF"; // implement CRC
  packet =
    "{\"type\":\"onPacket\",\"id\":\""
    + String(identifier)
    + "\",\"data\":\""
    + packet
    +"\"}";
  USB.println(packet);
}

void ESATEPS::updateBatteryTelemetry()
{
  BatteryController.error = false;
  telemetry[TOTAL_BATTERY_VOLTAGE_OFFSET] =
    BatteryController.readTotalBatteryVoltage();
  telemetry[BATTERY_1_VOLTAGE_OFFSET] =
    BatteryController.readBattery1Voltage();
  telemetry[BATTERY_2_VOLTAGE_OFFSET] =
    BatteryController.readBattery2Voltage();
  telemetry[BATTERY_TEMPERATURE_OFFSET] =
    BatteryController.readBatteryTemperature();
  telemetry[BATTERY_CURRENT_OFFSET] =
    BatteryController.readBatteryCurrent();
  telemetry[STATE_OF_CHARGE_OFFSET] =
    BatteryController.readStateOfCharge();
  bitWrite(telemetry[STATUS_REGISTER_2_OFFSET],
           BATTERY_STATUS_OFFSET,
           !BatteryController.error);
}

void ESATEPS::updateDirectEnergyTransferSystemTelemetry()
{
  DirectEnergyTransferSystem.error = false;
  telemetry[DIRECT_ENERGY_TRANSFER_SYSTEM_CURRENT_OFFSET] =
    DirectEnergyTransferSystem.readCurrent();
  telemetry[DIRECT_ENERGY_TRANSFER_SYSTEM_VOLTAGE_OFFSET] =
    DirectEnergyTransferSystem.readVoltage();
  telemetry[DIRECT_ENERGY_TRANSFER_SYSTEM_SHUNT_VOLTAGE_OFFSET] =
    DirectEnergyTransferSystem.readShuntVoltage();
  bitWrite(telemetry[STATUS_REGISTER_2_OFFSET],
           DIRECT_ENERGY_TRANSFER_SYSTEM_STATUS_OFFSET,
           !DirectEnergyTransferSystem.error);
}

void ESATEPS::updateMainTelemetry()
{
  telemetry[CURRENT_5V_OFFSET] =
    EPSMeasurements.read5VLineCurrent();
  telemetry[VOLTAGE_5V_OFFSET] =
    EPSMeasurements.read5VLineVoltage();
  telemetry[CURRENT_3V3_OFFSET] =
    EPSMeasurements.read3V3LineCurrent();
  telemetry[VOLTAGE_3V3_OFFSET] =
    EPSMeasurements.read3V3LineVoltage();
  telemetry[INPUT_CURRENT_OFFSET] =
    EPSMeasurements.readInputLineCurrent();
  telemetry[INPUT_VOLTAGE_OFFSET] =
    EPSMeasurements.readInputLineVoltage();
  telemetry[PANEL_1_OUTPUT_CURRENT_OFFSET] =
    EPSMeasurements.readPanel1OutputCurrent();
  telemetry[PANEL_1_VOLTAGE_OFFSET] =
    EPSMeasurements.readPanel1Voltage();
  telemetry[PANEL_1_INPUT_CURRENT_OFFSET] =
    EPSMeasurements.readPanel1InputCurrent();
  telemetry[PANEL_2_VOLTAGE_OFFSET] =
    EPSMeasurements.readPanel2Voltage();
  telemetry[PANEL_2_INPUT_CURRENT_OFFSET] =
    EPSMeasurements.readPanel2InputCurrent();
  telemetry[PANEL_2_OUTPUT_CURRENT_OFFSET] =
    EPSMeasurements.readPanel2OutputCurrent();
}

void ESATEPS::updateMPPT()
{
  MaximumPowerPointTrackingDriver1.update();
  MaximumPowerPointTrackingDriver2.update();
}

void ESATEPS::updatePanelTelemetry()
{
  SolarPanel1Thermometer.error = false;
  telemetry[PANEL_1_TEMPERATURE_OFFSET] =
    SolarPanel1Thermometer.read();
  bitWrite(telemetry[STATUS_REGISTER_2_OFFSET],
           PANEL_1_THERMOMETER_STATUS_OFFSET,
           !SolarPanel1Thermometer.error);
  SolarPanel2Thermometer.error = false;
  telemetry[PANEL_2_TEMPERATURE_OFFSET] =
    SolarPanel2Thermometer.read();
  bitWrite(telemetry[STATUS_REGISTER_2_OFFSET],
           PANEL_2_THERMOMETER_STATUS_OFFSET,
           !SolarPanel2Thermometer.error);
}

void ESATEPS::updateSoftwareVersionTelemetry()
{
  telemetry[STATUS_REGISTER_1_OFFSET] =
    soft_v << SOFTWARE_VERSION_OFFSET;
}

void ESATEPS::updateStatusTelemetry()
{
  bitWrite(telemetry[STATUS_REGISTER_2_OFFSET],
           SWITCH_5V_ON_OFFSET,
           PowerLine5VSwitch.read());
  bitWrite(telemetry[STATUS_REGISTER_2_OFFSET],
           SWITCH_3V3_ON_OFFSET,
           PowerLine3V3Switch.read());
  bitWrite(telemetry[STATUS_REGISTER_2_OFFSET],
           OVERCURRENT_3V3_OFFSET,
           OvercurrentDetector.read3V3LineOvercurrentState());
  bitWrite(telemetry[STATUS_REGISTER_2_OFFSET],
           OVERCURRENT_5V_OFFSET,
           OvercurrentDetector.read5VLineOvercurrentState());
}

void ESATEPS::updateTelemetry()
{
  updateMainTelemetry();
  updateSoftwareVersionTelemetry();
  updateBatteryTelemetry();
  updatePanelTelemetry();
  updateDirectEnergyTransferSystemTelemetry();
  updateStatusTelemetry();
}

boolean ESATEPS::pendingCommands()
{
  queueIncomingUSBCommands();
  return command.pending;
}

ESATEPS EPS;
