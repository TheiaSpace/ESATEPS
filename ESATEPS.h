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

#ifndef ESATEPS_h
#define ESATEPS_h
#include <Energia.h>
#include <MspFlash.h>
#define flash SEGMENT_C
#define EN5V 58
#define EN3V 60
#define bqAddress 0x0B
#define PWM1  P1_2
#define PWM2  P1_3
#define OC1 P2_4
#define OC2 P2_5
#define soft_v 1
class ESATEPS {
public:
byte bufferH[51];
byte EPSStatus;
boolean ACTIVEMPPT;
boolean MPPTMODE;
boolean MPPTFIXED;
void init();
void serialLog(String comment);
void housekeeping();
void updateMPPT();
void invokeBSL();
int command;
int param;
int myId;
private:
byte I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
byte I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
void handleCommand();
String build_tm_packet(int type, int apid);
String toHex(int i, int L);
uint16_t readADC(int channel);
void decode_tc_packet(String packet);
static void switch_fun();
static void switch_fun_n();
};
extern ESATEPS EPS;
#endif
