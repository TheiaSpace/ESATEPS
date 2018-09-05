/*
 * Copyright (C) 2017-2018 Theia Space, Universidad Politécnica de Madrid
 *
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

#include "ESAT_PowerLineSwitch.h"

ESAT_PowerLineSwitchClass::ESAT_PowerLineSwitchClass(const byte theLine)
{
  line = theLine;
  state = OFF;
}

void ESAT_PowerLineSwitchClass::begin()
{
  pinMode(line, OUTPUT);
  write(state);
}

ESAT_PowerLineSwitchClass::SwitchState ESAT_PowerLineSwitchClass::read()
{
  return state;
}

void ESAT_PowerLineSwitchClass::toggle()
{
  if (state == ON)
  {
    write(OFF);
  }
  else
  {
    write(ON);
  }
}

void ESAT_PowerLineSwitchClass::write(const ESAT_PowerLineSwitchClass::SwitchState newState)
{
  state = newState;
  if (state == ON)
  {
    digitalWrite(line, HIGH);
  }
  else
  {
    digitalWrite(line, LOW);
  }
}

ESAT_PowerLineSwitchClass ESAT_PowerLine3V3Switch(EN3V3);
ESAT_PowerLineSwitchClass ESAT_PowerLine5VSwitch(EN5V);
