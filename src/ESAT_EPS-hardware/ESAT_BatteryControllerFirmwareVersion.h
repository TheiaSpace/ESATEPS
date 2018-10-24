/*
 * Copyright (C) 2018 Theia Space, Universidad Politécnica de Madrid
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

#ifndef ESAT_BatteryControllerFirmwareVersion_h
#define ESAT_BatteryControllerFirmwareVersion_h

#include <Arduino.h>

// Firmware version of the battery controller.
class ESAT_BatteryControllerFirmwareVersion
{
  public:
    // Number of bytes used by the firmware version.
    static const byte LENGTH = 11;

    // Firmware version.
    byte version[LENGTH];

    // Write the firmware version to a stream.
    // Return true on success; otherwise return false.
    boolean writeTo(Stream& stream) const;
};

#endif /* ESAT_BatteryControllerFirmwareVersion_h */
