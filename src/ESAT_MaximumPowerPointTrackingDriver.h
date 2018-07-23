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

#ifndef ESAT_MaximumPowerPointTrackingDriver_h
#define ESAT_MaximumPowerPointTrackingDriver_h

#include <Arduino.h>

// Maximum power point tracking (MPPT) driver.
// Use the global instances ESAT_MaximumPowerPointTrackingDriver1 (for
// solar panel 1) and ESAT_MaximumPowerPointTrackingDriver2 (for solar
// panel 2).
//
// The maximum power point tracking system regulates the operating
// point of the solar panels to provide the maximum possible power.
class ESAT_MaximumPowerPointTrackingDriverClass
{
  public:
    // Instantiate an MPPT driver that takes input from a sensor pin
    // and drives an output pin with a PWM signal.
    ESAT_MaximumPowerPointTrackingDriverClass(int sensorPin, int outputPin);

    // Configure the pins of the MPPT driver.
    void begin();

    // Return the duty cycle.
    byte getDutyCycle();

    // Return the mode of operation.
    byte getMode();

    // Set the mode of operation to fixed mode.
    void setFixedMode(byte fixedDutyCycle);

    // Set the mode of operation to MPPT mode.
    void setMPPTMode();

    // Set the mode of operation to sweep mode.
    void setSweepMode();

    // Update the MPPT driver.
    void update();

  private:
    // Modes of operation.
    enum Mode
    {
      FIXED_MODE = 0,
      MPPT_MODE = 1,
      SWEEP_MODE = 2,
    };

    // Default duty cycle.
    static const byte DEFAULT_DUTY_CYCLE = 50;

    // Duty cycle relative to 255.
    byte dutyCycle;

    // Current mode of operation.
    Mode mode;

    // Previous sensor reading.
    int previousReading;

    // Increment to the output setting.
    int dutyCycleIncrement;

    // Read the current from this pin.
    int sensorPin;

    // Write a PWM signal to this pin.
    int outputPin;

    // Compute the sign of setpointIncrement for increasing the
    // delivered power.  Helper function for updateMPPTMode().
    int gradientDirection();

    // Update the MPPT driver in fixed mode.
    void updateFixedMode();

    // Update the MPPT driver in MPPT mode.
    void updateMPPTMode();

    // Update the MPPT driver in sweep mode.
    void updateSweepMode();
};

// Maximum power point tracking driver for solar panel 1.
extern ESAT_MaximumPowerPointTrackingDriverClass ESAT_MaximumPowerPointTrackingDriver1;

// Maximum power point tracking driver for solar panel 2.
extern ESAT_MaximumPowerPointTrackingDriverClass ESAT_MaximumPowerPointTrackingDriver2;

#endif /* ESAT_MaximumPowerPointTrackingDriver_h */
