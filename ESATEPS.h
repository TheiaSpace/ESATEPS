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
#define bqAddress 0x0B
#define soft_v 1

class ESATEPS
{
  public:
    byte bufferH[51];
    byte EPSStatus;

    // Instantiate a new ESATEPS object.
    ESATEPS();

    void init();
    void serialLog(String comment);
    void housekeeping();
    void updateMPPT();
    void invokeBSL();
    int command;
    int param;
    int myId;

  private:
    // Command codes.
    enum CommandCode
    {
      SET_IDENTIFIER = 0,
      TOGGLE_5V_LINE = 1,
      TOGGLE_3V3_LINE = 2,
      MAXIMUM_POWER_POINT_TRACKING_MODE = 3,
      SWEEP_MODE = 4,
      FIXED_MODE = 5,
    };

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
