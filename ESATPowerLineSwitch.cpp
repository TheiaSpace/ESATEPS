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

#include "ESATPowerLineSwitch.h"

ESATPowerLineSwitch PowerLine3V3Switch(EN3V3);
ESATPowerLineSwitch PowerLine5VSwitch(EN5V);

ESATPowerLineSwitch::ESATPowerLineSwitch(const byte line):
  line(line), state(off)
{
}

void ESATPowerLineSwitch::begin()
{
  pinMode(line, OUTPUT);
  write(state);
}

ESATPowerLineSwitch::SwitchState ESATPowerLineSwitch::read()
{
  return state;
}

void ESATPowerLineSwitch::toggle()
{
  if (state == on)
  {
    write(off);
  }
  else
  {
    write(on);
  }
}

void ESATPowerLineSwitch::write(const ESATPowerLineSwitch::SwitchState newState)
{
  state = newState;
  if (state == on)
  {
    digitalWrite(line, HIGH);
  }
  else
  {
    digitalWrite(line, LOW);
  }
}
