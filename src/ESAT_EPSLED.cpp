/*
 * Copyright (C) 2018 Theia Space, Universidad Politécnica de Madrid
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ESAT_EPSLED.h"

void ESAT_EPSLEDClass::begin()
{
  pinMode(LED_CONTROL_LINE, OUTPUT);
  write(0);
}

void ESAT_EPSLEDClass::write(const float brightness)
{
  const byte dutyCycle = map(brightness, 0, 100, 0, 255);
  analogWrite(LED_CONTROL_LINE, dutyCycle);
}

ESAT_EPSLEDClass ESAT_EPSLED;
