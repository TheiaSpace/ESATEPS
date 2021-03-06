﻿Copyright (C) 2017, 2018, 2019 Theia Space, Universidad Politécnica de Madrid

This file is part of Theia Space's ESAT EPS library.

Theia Space's ESAT EPS library is free software: you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation, either
version 3 of the License, or (at your option) any later version.

Theia Space's ESAT EPS library is distributed in the hope that it
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Theia Space's ESAT EPS library.  If not, see
<http://www.gnu.org/licenses/>.


ESAT EPS library version 2.1.1.

Electrical Power Subsystem (EPS) for ESAT.

This software runs in the ESAT EPS board.  Use it with Arduino 1.8.0
or newer together with the Arduino core for MSP430-based ESAT boards
(Theia Space's ESAT Boards (MSP430)) 2.2.0 or a newer 2.x.y version
and the ESAT utility library (ESATUtil) version 2.1.0 or a newer 2.x.y
version.  Use ESAT-EPS as the target board.

See the example EPS program (examples/EPS/EPS.ino).  This program
uses the modules of the EPS library.

The src/ directory contains the EPS library (ESAT_EPS) as well as
several subdirectories:


# ESAT_EPS

This is the main library that provides the general EPS functionality.


# ESAT_EPS-hardware directory

This directory contains drivers for the peripherals of the ESAT EPS
board used by ESAT_EPS.


# ESAT_EPS-telecommands directory

Telecommand handlers.


# ESAT_EPS-telemetry directory

Telemetry packets.
