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

#ifndef ESATPowerLineSwitch_h
#define ESATPowerLineSwitch_h

#include <Energia.h>

// Power line switch controller.
class ESATPowerLineSwitch
{
  public:
    // State of the switch.
    enum SwitchState
    {
      OFF = 0,
      ON = 1,
    };

    // Instantiate a new power line switch on a given line.
    ESATPowerLineSwitch(byte line);

    // Set up the power line controller.
    void begin();

    // Return the state of the line.
    SwitchState read();

    // Toggle the state of the line.
    void toggle();

    // Switch the line.
    void write(SwitchState newState);

  private:
    // Control this line.
    const boolean line;

    // State of the switch.
    SwitchState state;
};

extern ESATPowerLineSwitch PowerLine3V3Switch;
extern ESATPowerLineSwitch PowerLine5VSwitch;

#endif /* ESATPowerLineSwitch_h */
