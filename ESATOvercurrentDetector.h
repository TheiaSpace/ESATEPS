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

#ifndef ESATOvercurrentDetector_h
#define ESATOvercurrentDetector_h

#include <Arduino.h>

// Overcurrent detection.
// Use the global instance OvercurrentDetector.
class ESATOvercurrentDetector
{
  public:
    // Set up the overcurrent detector.
    void begin();

    // Return true if the 3.3 V line is in overcurrent state;
    // otherwise return false.
    boolean read3V3LineOvercurrentState();

    // Return true if the 5 V line is in overcurrent state; otherwise
    // return false.
    boolean read5VLineOvercurrentState();
};

// Global instance of the overcurrent detector library.
extern ESATOvercurrentDetector OvercurrentDetector;

#endif /* ESATOvercurrentDetector_h */
