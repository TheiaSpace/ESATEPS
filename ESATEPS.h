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
    int myId;

    // Add a command to the command queue.
    void queueCommand(byte commandCode, byte parameter);

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

    // Command buffer structure.
    struct Command
    {
      byte commandCode;
      byte parameter;
      boolean pending;
    };

    // Command queue.
    volatile Command command;

    // Last received command code.
    byte commandCode;

    // Last received command parameter.
    byte commandParameter;

    // Set the maximum power point tracking drivers in fixed mode.
    void handleFixedModeCommand();

    // Set the maximum power point tracking drivers in maximum power
    // point tracking mode.
    void handleMaximumPowerPointTrackingModeCommand();

    // Set and store the identifier number.
    void handleSetIdentifierCommand();

    // Set the maximum power point tracking drivers in sweep mode.
    void handleSweepModeCommand();

    // Toggle the 3V3 line.
    void handleToggle3V3LineCommand();

    // Toggle the 5V line.
    void handleToggle5VLineCommand();

    void handleCommand();
    String build_tm_packet(int type, int apid);
    String toHex(int i, int L);
    void decode_tc_packet(String packet);
};

extern ESATEPS EPS;

#endif
