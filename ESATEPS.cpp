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

void receiveEvent(const int howMany)
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

void requestEvent()
{
  Wire.write(EPS.bufferH, 51);
}

void ESATEPS::init()
{
  unsigned char Id;
  Flash.read(flash, &Id, 1);
  myId = (int) Id;
  command.pending = false;
  EPSMeasurements.begin();
  MaximumPowerPointTrackingDriver1.begin();
  MaximumPowerPointTrackingDriver2.begin();
  MaximumPowerPointTrackingDriver1.setMPPTMode();
  MaximumPowerPointTrackingDriver2.setMPPTMode();
  EPSStatus = 0;
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
  myId = command.parameter;
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

String ESATEPS::build_tm_packet(int type, int apid=1)
{
  // build packet with given data (hex), type
  // ID(b3)|TM/TC(b1)|APID(h1)|length(h2)|type(h2)|data|CRC(h2)
  String packet = "";
  packet += Util.byteToHexadecimal(myId).substring(1, 2);
  packet += "2";
  packet += Util.byteToHexadecimal(51 * 2);
  packet += Util.byteToHexadecimal(type);
  for (int i = 0; i < 50; i++)
  {
    packet += Util.byteToHexadecimal(bufferH[i]);
  }
  packet += "00000000";
  packet += "FF"; // implement CRC
  packet =
    "{\"type\":\"onPacket\",\"id\":\""
    + String(myId)
    + "\",\"data\":\""
    + packet
    +"\"}";
  USB.println(packet);
  return packet;
}

void ESATEPS::housekeeping()
{
  // EPS (Main) TM
  const word current5V = EPSMeasurements.read5VLineCurrent();
  bufferH[4] = highByte(current5V);
  bufferH[5] = lowByte(current5V);
  const word voltage5V = EPSMeasurements.read5VLineVoltage();
  bufferH[6] = highByte(voltage5V);
  bufferH[7] = lowByte(voltage5V);
  const word current3V3 = EPSMeasurements.read3V3LineCurrent();
  bufferH[8] = highByte(current3V3);
  bufferH[9] = lowByte(current3V3);
  const word voltage3V3 = EPSMeasurements.read3V3LineVoltage();
  bufferH[10] = highByte(voltage3V3);
  bufferH[11] = lowByte(voltage3V3);
  const word inputCurrent = EPSMeasurements.readInputLineCurrent();
  bufferH[12] = highByte(inputCurrent);
  bufferH[13] = lowByte(inputCurrent);
  const word inputVoltage = EPSMeasurements.readInputLineVoltage();
  bufferH[14] = highByte(inputVoltage);
  bufferH[15] = lowByte(inputVoltage);
  const word panel1OutputCurrent = EPSMeasurements.readPanel1OutputCurrent();
  bufferH[16] = highByte(panel1OutputCurrent);
  bufferH[17] = lowByte(panel1OutputCurrent);
  const word panel1Voltage = EPSMeasurements.readPanel1Voltage();
  bufferH[18] = highByte(panel1Voltage);
  bufferH[19] = lowByte(panel1Voltage);
  const word panel1InputCurrent = EPSMeasurements.readPanel1InputCurrent();
  bufferH[20] = highByte(panel1InputCurrent);
  bufferH[21] = lowByte(panel1InputCurrent);
  const word panel2Voltage = EPSMeasurements.readPanel2Voltage();
  bufferH[22] = highByte(panel2Voltage);
  bufferH[23] = lowByte(panel2Voltage);
  const word panel2InputCurrent = EPSMeasurements.readPanel2InputCurrent();
  bufferH[24] = highByte(panel2InputCurrent);
  bufferH[25] = lowByte(panel2InputCurrent);
  const word panel2OutputCurrent = EPSMeasurements.readPanel2OutputCurrent();
  bufferH[26] = highByte(panel2OutputCurrent);
  bufferH[27] = lowByte(panel2OutputCurrent);

  // Software version
  bufferH[1] = (soft_v << 3);

  // EPS (Bat) TM
  BatteryController.error = false;
  const int totalBatteryVoltage = BatteryController.readTotalBatteryVoltage();
  bufferH[28] = highByte(totalBatteryVoltage);
  bufferH[29] = lowByte(totalBatteryVoltage);
  const int battery1Voltage = BatteryController.readBattery1Voltage();
  bufferH[30] = highByte(battery1Voltage);
  bufferH[31] = lowByte(battery1Voltage);
  const int battery2Voltage = BatteryController.readBattery2Voltage();
  bufferH[32] = highByte(battery2Voltage);
  bufferH[33] = lowByte(battery2Voltage);
  const byte stateOfCharge = BatteryController.readStateOfCharge();
  bufferH[48] = 0;
  bufferH[49] = stateOfCharge;
  const int batteryTemperature = BatteryController.readBatteryTemperature();
  bufferH[34] = highByte(batteryTemperature);
  bufferH[35] = lowByte(batteryTemperature);
  const int batteryCurrent = BatteryController.readBatteryCurrent();
  bufferH[36] = highByte(batteryCurrent);
  bufferH[37] = lowByte(batteryCurrent);
  bitWrite(EPSStatus, 3, !BatteryController.error); // EPSStatus.BAT

  // Solar array TM
  SolarPanel1Thermometer.error = false;
  const int solarPanel1Temperature = SolarPanel1Thermometer.read();
  bufferH[38] = highByte(solarPanel1Temperature);
  bufferH[39] = lowByte(solarPanel1Temperature);
  bitWrite(EPSStatus, 2, !SolarPanel1Thermometer.error);
  SolarPanel1Thermometer.error = false;
  const int solarPanel2Temperature = SolarPanel2Thermometer.read();
  bufferH[0] = highByte(solarPanel2Temperature);
  bufferH[1] = lowByte(solarPanel2Temperature);
  bitWrite(EPSStatus, 1, !SolarPanel2Thermometer.error);

  // DET TM
  DirectEnergyTransferSystem.error = false;
  const int directEnergyTransferSystemCurrent =
    DirectEnergyTransferSystem.readCurrent();
  bufferH[42] = highByte(directEnergyTransferSystemCurrent);
  bufferH[43] = lowByte(directEnergyTransferSystemCurrent);
  const int directEnergyTransferSystemVoltage =
    DirectEnergyTransferSystem.readVoltage();
  bufferH[44] = highByte(directEnergyTransferSystemVoltage);
  bufferH[45] = lowByte(directEnergyTransferSystemVoltage);
  const int directEnergyTransferSystemShuntVoltage =
    DirectEnergyTransferSystem.readShuntVoltage();
  bufferH[46] = highByte(directEnergyTransferSystemShuntVoltage);
  bufferH[47] = lowByte(directEnergyTransferSystemShuntVoltage);
  bitWrite(EPSStatus, 0, !DirectEnergyTransferSystem.error);

  // EPS Status registers
  bitWrite(EPSStatus, 7, PowerLine5VSwitch.read());
  bitWrite(EPSStatus, 6, PowerLine3V3Switch.read());
  bitWrite(EPSStatus, 5, OvercurrentDetector.read3V3LineOvercurrentState());
  bitWrite(EPSStatus, 4, OvercurrentDetector.read5VLineOvercurrentState());
  bufferH[3] = EPSStatus;
  build_tm_packet(1, 2);
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

String ESATEPS::toHex(int i, int L)
{
  String ch = String(i, HEX);
  while (ch.length() < L)
  {
    ch = "0" + ch;
  }
  return ch.substring(0, L);
}

void ESATEPS::updateMPPT()
{
  MaximumPowerPointTrackingDriver1.update();
  MaximumPowerPointTrackingDriver2.update();
}


boolean ESATEPS::pendingCommands()
{
  queueIncomingUSBCommands();
  return command.pending;
}

ESATEPS EPS;
