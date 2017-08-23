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

#include "ESATOvercurrentDetector.h"

ESATOvercurrentDetector OvercurrentDetector;

static volatile boolean overcurrentOn3V3Line = false;

static void overcurrentOn3V3LineStarts();
static void overcurrentOn3V3LineStops();

// Interrupt handler called when an overcurrent on the 3.3 V line
// starts.
static void overcurrentOn3V3LineStarts()
{
  detachInterrupt(OC3V3);
  overcurrentOn3V3Line = true;
  attachInterrupt(OC3V3,
                  overcurrentOn3V3LineStops,
                  RISING);
}

// Interrupt handler called when the overcurrent on the 3.3 V line
// stops.
static void overcurrentOn3V3LineStops()
{
  detachInterrupt(OC3V3);
  overcurrentOn3V3Line = false;
  attachInterrupt(OC3V3,
                  overcurrentOn3V3LineStarts,
                  FALLING);
}

void ESATOvercurrentDetector::begin()
{
  pinMode(OC3V3, INPUT_PULLUP);
  pinMode(OC5V, INPUT_PULLUP);
  attachInterrupt(OC3V3,
                  overcurrentOn3V3LineStarts,
                  FALLING);
}

boolean ESATOvercurrentDetector::read3V3LineOvercurrentState()
{
  return overcurrentOn3V3Line;
}

boolean ESATOvercurrentDetector::read5VLineOvercurrentState()
{
  return !digitalRead(OC5V);
}
