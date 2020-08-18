/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include <stdint.h>
#include "us_ticker_api.h"
#include "Kernel.h"
#include "InterruptIn.h"
#include "port_api.h"
#include "mbed.h"

/*
*********************************************************************************************************
Module creates a software based quadrature single ended encoder. 

Public functions accssed through:  SWE0->

Config Values>
swencoder0.enable                   > set true to enable the encoder
swencoder0.cpr                      > decoded counts per revolution as int [360]
swencoder0.chan_A                   > interrupt capable pin for channel A
swencoder0.chan_B                   > interrupt capable pin for channel B
swencoder0.chan_I                   > interrupt capable pin for INDEX channel - optional
swencoder0.debug                    > set true to determine correct count direction [false]

*********************************************************************************************************
*/

#define PREV_MASK 0x1 //Mask for the previous state in determining direction of rotation.
#define CURR_MASK 0x2 //Mask for the current state in determining direction of rotation.
#define INVALID   0x3 //XORing two states where both bits have changed.

namespace mbed {
    class InterruptIn;     
}
class Pin;

class SwEncoder0 : public Module{
    public:
        SwEncoder0();
        virtual ~SwEncoder0() {};
        void on_module_loaded();
        void on_console_line_received(void *argument);
        void on_get_public_data(void* argument);        
        int get_count_per_rev();
        long read_enc();
        void reset();
        void disable_enc();
        void enable_enc();
        long get_revs();
        static SwEncoder0* instance;

    private:
        
        mbed::InterruptIn *chan_A; // Channel A for encoder
        mbed::InterruptIn *chan_B; // Channel B for encoder
        mbed::InterruptIn *chan_I; // Index channel for encoder
        mbed::Ticker t;
        
        // 2X or 4X decoding
        int decoding_type;
        // value from config
        bool debug;
        //void on_pin_rise();
        void encode();
        void on_index();
        void debug_out();
        // count per rev
        int cpr;
        bool has_I;

        int currState;
        int prevState;
        
        volatile long pulses; 
        long revs;      
};
