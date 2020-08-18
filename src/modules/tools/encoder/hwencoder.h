/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include <stdint.h>
#include <stdio.h>
#include "Kernel.h"
#include "mbed.h"
#include "libs/LPC17xx/LPC17xxLib/inc/lpc17xx_qei.h"

/*
**************************************************************************************
Module will enable the QEI (Quadrature Encoder Inferface) on the LPC176x chip.       *
This is a hardware interface that uses predefined pins.                              *
Channel A - pin 34 (P1-20)                                                           *
Channel B - pin 37 (P1-23)                                                           *
Channel I - pin 38 (P1-24)                                                           *
                                                                                     *
Count register - QEIPOS                                                              *
Index register - INXCNT                                                              *
Public functions accssed through:  HWE->                                             *
                                                                                     *
Config Values>                                                                       *
hwencoder.enable            >set true to enable the encoder                          *
hwencoder.cpr               >decoded counts per revolution as int [360]              *
hwencoder.filter            >input filter in pclk cycles as int - [200]              *
hwencoder.debug             >set true to determine correct count direction [false]   *
hwencoder.inv               >set true to reverse direction of count [false]          *
**************************************************************************************
*/

class HwEncoder : public Module{
    public:
        HwEncoder();
        virtual ~HwEncoder() {};
        void on_module_loaded();          
        int get_count_per_rev();
        long read_enc();
        void invert();
        void reset();
        static HwEncoder* instance;                              

    private:  
        mbed::Ticker t; 
        QEI_CFG_Type hwe;             
        
        // value from config
        bool debug;        
        int cpr;
        int filter;
        bool inv;
        
        bool inverse_count;
                
        void debug_out();     
};
