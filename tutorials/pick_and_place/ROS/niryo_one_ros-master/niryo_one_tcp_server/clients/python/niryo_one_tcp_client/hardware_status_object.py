#!/usr/bin/env python

# hardware_status_object.py
# Copyright (C) 2019 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


class HardwareStatusObject:
    def __init__(self, rpi_temperature, hardware_version, connection_up,
                 error_message, calibration_needed, calibration_in_progress,
                 motor_names, motor_types,
                 temperatures, voltages, hardware_errors):
        # Number representing the rpi temperature
        self.rpi_temperature = rpi_temperature
        # Number representing the hardware version
        self.hardware_version = hardware_version
        # Boolean indicating if the connection with the robot is up
        self.connection_up = connection_up
        # Error message status on error
        self.error_message = error_message
        # Boolean indicating if a calibration is needed
        self.calibration_needed = calibration_needed
        # Boolean indicating if calibration is in progress
        self.calibration_in_progress = calibration_in_progress

        # Following list describe each motor
        # Row 0 for first motor, row 1 for second motor, row 2 for third motor, row 3 for fourth motor
        # List of motor names
        self.motor_names = motor_names
        # List of motor types
        self.motor_types = motor_types
        # List of temperatures
        self.temperatures = temperatures
        # List of voltages
        self.voltages = voltages
        # List of hardware errors
        self.hardware_errors = hardware_errors
