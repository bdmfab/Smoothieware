/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ENCODER_SPINDLE_MODULE_H
#define ENCODER_SPINDLE_MODULE_H

/*
************************************************************************************************************
This module implements closed loop PID control for spindle RPM with a 2 channel encoder.

Config Values>
spindle.enable                   > enable spindle [false]
spindle.type                     > encoder
spindle.pulses_per_rev           > decoded pulses per rev of encoder [360]
spindle.pwm_pin                  > pin to output pulse width modulated signal to drive      
spindle.pwm_period               > frequncy of pwm signal (microseconds between pulses) [1000]
spindle.max_pwm                  > limit pwm to this (.5 is 50%) [1.0]
spindle.default_rpm              > min rpm to overcome friction [max_rpm * .1]
spindle.max_rpm                  > highest rpm spindle will allow [10000]
spindle.control_P                > proportional speed control [0.0001f]
spindle.control_I                > integrator speed control [0.0001f]
spindle.control_D                > derivitive speed control [0.0001f]
spindle.control_smoothing        > low pass filter time constant [0.1f]
spindle.reverse_dir_pin          > pin to output when M4 is called (optional)
spindle.switch_on_pin            > pin to output for enable drive (optional)
spindle.max_error                > max error value used by loop control [maxrpm * .25f]
spindle.update_freq              > times per second to update encoder pulses [20]
spindle.debug                    > enable troubleshooting features [false]
spindle.encoder_select           > what encoder to use 0:hwencoder 1:swencoder0 [0]
TODO:
Autotune

*************************************************************************************************************
*/

#include "SpindleControl.h"
#include <stdint.h>
#include "us_ticker_api.h"
#include "Kernel.h"
#include "modules/tools/encoder/swencoder0.h"
#include "modules/tools/encoder/hwencoder.h"
#include "libs/LPC17xx/LPC17xxLib/inc/lpc17xx_qei.h"


namespace mbed {
    class PwmOut;     
}

class Pin;

class EncoderSpindleControl: public SpindleControl {
    public:
        EncoderSpindleControl();
        virtual ~EncoderSpindleControl() {};
        void on_module_loaded();         
            
    private:        
               
        uint32_t on_update_speed(uint32_t dummy);        
        
        Pin *reverse_dir; // digital output pin for reverse
        Pin *switch_on; // digital output pin to Enable spindle drive 
        mbed::PwmOut *pwm_pin; // PWM output for spindle speed control        
        bool output_inverted;        
        bool debug;
        bool vals_loaded;
        std::string reverse_dir_pin;
        std::string switch_on_pin;              

        // Current values, updated at runtime
        float current_rpm;
        float target_rpm;
        float default_rpm;
        float max_rpm;
        float current_I_value;
        float prev_error;
        float current_pwm_value;        
       
        // Values from config
        int pulses_per_rev;
        float control_P_term;
        float control_I_term;
        float control_D_term;
        float smoothing_decay;
        float max_pwm;      
        float max_error;
        int update_freq;
        int encoder_select;

        
        long pulses;       
        int stall;
        int change;
        int bad_enc;
        
        void turn_on(void);
        void turn_on_rev(void);
        void turn_off(void);
        void set_speed(int);
        void report_speed(void);
        void set_p_term(float);
        void set_i_term(float);
        void set_d_term(float);
        void report_settings(void);
        void load_vals(void);
};

#endif

