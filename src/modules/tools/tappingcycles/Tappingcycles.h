/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TAPPINGCYCLES_MODULE_H
#define TAPPINGCYCLES_MODULE_H

/*
/!\ This code expects a clean gcode, no fail safe at this time.

Implemented     : G74 G84
Absolute mode   : yes
Relative mode   : no
Incremental (L) : no
G74 or G84 X_ Y_ Z_ R_ F_ S_
G79 X_ Y_
.....
G80

Config Values>
   tappingcycles.enable             > true to enable
   tappingcycles.ms_update          > time between updates in ms [50]
   tappingcycles.encoder_select     > what encoder to use 0:hwencoder 1:swencoder0 [0]  
   tappingcycles.debug              > enable troubleshooting features [false]
   
*/

#include "libs/Module.h"
#include "mbed.h"
#include "modules/tools/encoder/swencoder0.h"
#include "modules/tools/encoder/hwencoder.h"
#include "StepperMotor.h"
#include "StepTicker.h"
#include "libs/LPC17xx/LPC17xxLib/inc/lpc17xx_qei.h"

class Gcode;

class Tappingcycles : public Module
{
    public:
        Tappingcycles();
        virtual ~Tappingcycles() {};
        void on_module_loaded();

    private:
        void on_config_reload(void *argument);
        void on_gcode_received(void *argument);
        void reset_sticky();
        void update_sticky(Gcode *gcode);
        int  send_gcode(const char* format, ...);
        void tap_hole(Gcode *gcode);
        void start_sync();
        void stop_sync();
        void calc_speed();
        void do_step();
        void z_pos_update();
        long get_count();
        void load_vals();

        mbed::Ticker pulse_timer;        
        mbed::Timer t;

        bool cycle_started; // cycle status
        bool vals_loaded;
        bool forward;
        bool is_G84;  
        bool is_Tapping;      
        int  retract_type;  // rretract type
        
        // values from config
        int update_rate;
        int encoder_select;
        bool debug;
        int pulses_per_rev;      // count per rev of encoder

        float initial_z;    // Initial-Z
        float r_plane;      // R-Plane

        float sticky_z;     // final depth
        float sticky_r;     // R-Plane
        float sticky_f;     // feedrate

        float sticky_q;     // depth increment
        int sticky_s;       // tapping rpm        

        int totalPulses;   // pulse output from man_pulse
        int last_pulses;
        int target_pulses;
        
        long last_c;        
        float z_axis_res;   // count per mm of z axis
        float timeslice;
        float ratio;
        float calcVal;
        

        int   dwell_units;  // units for dwell
};

#endif
