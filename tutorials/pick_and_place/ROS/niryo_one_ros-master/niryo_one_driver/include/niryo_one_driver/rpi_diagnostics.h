/*
    rpi_diagnostics.h
    Copyright (C) 2017 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef RPI_DIAGNOSTICS_H
#define RPI_DIAGNOSTICS_H

#ifdef __arm__
#include <fstream> 
#endif

#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <thread>

#include <ros/ros.h>

class RpiDiagnostics {

    public:

        RpiDiagnostics();

        int getRpiCpuTemperature(); 

        void startReadingData();

    private:

        int cpu_temperature;
      
        void readCpuTemperature();

        void readHardwareDataLoop();
        
        boost::shared_ptr<std::thread> read_hardware_data_thread;
};

#endif
