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
#include <USBSerial.h>
#include <Wire.h>
#include "ESATBatteryController.h"
#include "ESATSolarPanelThermometer.h"

bool ENABLED5 = false;
boolean ENABLED3 = true;
boolean isOC3V3 = false;

ESATEPS::ESATEPS(): mppt1(I_P2_IN, PWM1), mppt2(I_P1_IN, PWM2)
{
}

void receiveEvent(int howMany)
{
  EPS.command = Wire.read();
  if (howMany > 1)
  {
    EPS.param = Wire.read();
  }
  else
  {
    EPS.param = 0;
  }
}

void requestEvent()
{
  Wire.write(EPS.bufferH, 51);
}

byte ESATEPS::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire1.beginTransmission(Address);
  Wire1.write(Register);
  byte error = Wire1.endTransmission();
  if (error != 0)
  {
  }
  else
  {
    // Read Nbytes
    Wire1.requestFrom(Address, Nbytes);
    uint8_t index = 0;
    for (int index=0; index < Nbytes; index++)
    {
      Data[index] = Wire1.read();
    }
  }
  return error;
}

byte ESATEPS::I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire1.beginTransmission(Address);
  Wire1.write(Register);
  Wire1.write(Data);
  byte error = Wire1.endTransmission();
  return error;
}

void ESATEPS::init()
{
  unsigned char Id;
  Flash.read(flash, &Id, 1);
  myId = (int) Id;
  command = 0;
  param = 0;
  mppt1.begin();
  mppt2.begin();
  mppt1.setMPPTMode();
  mppt2.setMPPTMode();
  EPSStatus = 0;
  pinMode(EN5V, OUTPUT);
  digitalWrite(EN5V, LOW);
  pinMode(EN3V3, OUTPUT);
  digitalWrite(EN3V3, HIGH);
  pinMode(OC5V, INPUT_PULLUP);
  pinMode(OC3V3, INPUT_PULLUP);
  attachInterrupt(OC3V3, switch_fun, FALLING);
  Wire1.begin();
  Wire.begin(2);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  analogReference(INTERNAL2V5);
  USB.begin();
}

void ESATEPS::serialLog(String comment)
{
  USB.println("{\"type\":\"log\",\"data\":\"" + comment + "\"}");
}
void ESATEPS::handleCommand()
{
  switch (EPS.command)
  {
  case 0:
    // set id and store in flash
    Flash.erase(flash);
    myId = param;
    unsigned char p;
    p = param;
    Flash.write(flash, &p ,1);
    EPS.command = 0;
    break;
  case 1:
    ENABLED5 = !ENABLED5;
    if (ENABLED5)
    {
      digitalWrite(EN5V, HIGH);
    }
    else
    {
      digitalWrite(EN5V, LOW);
    }
    EPS.command = 0;
    break;
  case 2:
    ENABLED3 = !ENABLED3;
    if (ENABLED3)
    {
      digitalWrite(EN3V3, HIGH);
    }
    else
    {
      digitalWrite(EN3V3, LOW);
    }
    EPS.command = 0;
    break;
  case 3:
    mppt1.setMPPTMode();
    mppt2.setMPPTMode();
    EPS.command = 0;
    break;
  case 4:
    mppt1.setSweepMode();
    mppt2.setSweepMode();
    EPS.command = 0;
    break;
  case 5:
    EPS.param = constrain(EPS.param, 0, 255);
    mppt1.setFixedMode();
    mppt2.setFixedMode();
    mppt1.dutyCycle = EPS.param;
    mppt2.dutyCycle = EPS.param;
    EPS.command = 0;
    break;
  default:
    EPS.command = 0;
    break;
  }
}

String ESATEPS::build_tm_packet(int type, int apid=1)
{
  // build packet with given data (hex), type
  // ID(b3)|TM/TC(b1)|APID(h1)|length(h2)|type(h2)|data|CRC(h2)
  String packet = "";
  packet += toHex(myId, 1);
  packet += "2";
  packet += toHex(51 * 2, 2);
  packet += toHex(type, 2);
  for (int i = 0; i < 50; i++)
  {
    packet += toHex(bufferH[i], 2);
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
  // Check and dispatch commands
  if(EPS.command!=0)
  {
    handleCommand();
  }
  if (USB.available())
  {
    String cmd = USB.readStringUntil('\r');
    String identifier = cmd.substring(0, 1);
    if (identifier == "@")
    {
      decode_tc_packet(cmd.substring(1, cmd.length() + 1));
    }
  }
  // EPS (Main) TM
  int channels[14] = {
    I_12V,
    V_12V,
    I_5V,
    V_5V,
    I_3V3,
    V_3V3,
    I_IN,
    V_IN,
    I_P2_IN,
    V_P2,
    I_P2_OUT,
    I_P1_OUT,
    V_P1,
    I_P1_IN
  };
  int x[14];
  int numAvgs[14] = { 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 5 };
  for (int p = 0; p < 14; p++)
  {
    double y = 0;
    for (int i = 0; i < numAvgs[p]; i++)
    {
      y += double(analogRead(channels[p])) / numAvgs[p];
      delay(10);
    }
    x[p] = round(y);
  }

  for(int i = 2; i < 14; i++)
  {
    bufferH[2*i+1] = (x[i] & 255);
    bufferH[2*i] = (x[i] >> 8);
  }
  // Software version
  bufferH[1] = (soft_v << 3);

  // EPS (Bat) TM
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
  const int solarPanel1Temperature = SolarPanel1Thermometer.read();
  bufferH[38] = highByte(solarPanel1Temperature);
  bufferH[39] = lowByte(solarPanel1Temperature);
  bitWrite(EPSStatus, 2, !SolarPanel1Thermometer.error);
  const int solarPanel2Temperature = SolarPanel2Thermometer.read();
  bufferH[0] = highByte(solarPanel2Temperature);
  bufferH[1] = lowByte(solarPanel2Temperature);
  bitWrite(EPSStatus, 1, !SolarPanel2Thermometer.error);

  // DET TM
  uint16_t channels_ADC;
  for (int i = 0; i < 3; i++)
  {
    channels_ADC = readADC(i);
    bufferH[42+2*i+1] = (channels_ADC & 255);
    bufferH[42+2*i] = (channels_ADC >> 8);
  }

  // EPS Status registers
  bitWrite(EPSStatus, 7, ENABLED5);
  bitWrite(EPSStatus, 6, ENABLED3);
  bitWrite(EPSStatus, 5, isOC3V3);
  bitWrite(EPSStatus, 4, !digitalRead(OC5V));
  bufferH[3] = EPSStatus;
  build_tm_packet(1, 2);
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
  mppt1.update();
  mppt2.update();
}

uint16_t ESATEPS::readADC(int channel)
{
  int I2C_address = 0x48;
  uint16_t res;
  uint16_t config = 0x0003 | 0x0080 | 0x0100;
  config |= 0x0200;
  switch (channel)
  {
  case (0):
    config |= 0x4000;
    break;
  case (1):
    config |= 0x5000;
    break;
  case (2):
    config |= 0x7000;
    break;
  default:
    return 0;
    break;
  }
  // Set 'start single-conversion' bit
  config |=0x8000; // ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  Wire1.beginTransmission(I2C_address);
  Wire1.write((uint8_t) 0x01);
  Wire1.write((uint8_t) (config >> 8));
  Wire1.write((uint8_t) (config & 0xFF));
  Wire1.endTransmission();
  // Wait for the conversion to complete
  delay(1);
  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015
  byte result[2];
  byte error = I2Cread(I2C_address, 0x00, 2, result);
  bitWrite(EPSStatus, 0, error == 0);
  if (error==0)
  {
    res = ((result[0] << 8) | result[1]) >>4;
    return (int16_t) res;
  }
  else
  {
    return 0;
  }
}


void ESATEPS::switch_fun()
{
  detachInterrupt(OC3V3);
  isOC3V3 = true;
  attachInterrupt(OC3V3, switch_fun_n, RISING);
}

void ESATEPS::switch_fun_n()
{
  // Each rotation, this interrupt function is run twice
  detachInterrupt(OC3V3);
  isOC3V3 = false;
  attachInterrupt(OC3V3, switch_fun, FALLING);
}

void ESATEPS::decode_tc_packet(String hexstring)
{
  EPS.command = (int) strtol(hexstring.substring(4, 6).c_str(), 0, 16);
  int length = (int) strtol(hexstring.substring(2, 4).c_str(), 0, 16);
  EPS.param = hexstring.substring(6, 6 + length).toInt();
  handleCommand();
}

void ESATEPS::invokeBSL(){
    SFRIE1 &= ~OFIE; /* Disable oscillator fault enable interrupt */
    delay(500);
    SYSBSLC &= ~(SYSBSLPE | SYSBSLOFF);
    __disable_interrupt(); /* Ensure no application interrupts occur while in BSL */
    /*
     * This sends execution to the BSL. When execution returns
     * to the user app, it will be via the reset vector, meaning
     * execution will re-start.
     */
    ((void (*)()) 0x1000)();
}

ESATEPS EPS;
