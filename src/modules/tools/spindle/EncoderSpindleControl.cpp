/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "EncoderSpindleControl.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutputPool.h"
#include "SlowTicker.h"
#include "Conveyor.h"
#include "system_LPC17xx.h"
#include "utils.h"


#include "libs/Pin.h"
#include "InterruptIn.h"
#include "PwmOut.h"
#include "port_api.h"


#define spindle_checksum                    CHECKSUM("spindle")
#define pwm_pin_checksum                    CHECKSUM("pwm_pin")
#define pwm_period_checksum                 CHECKSUM("pwm_period")
#define max_pwm_checksum                    CHECKSUM("max_pwm")
#define default_rpm_checksum                CHECKSUM("default_rpm")
#define max_rpm_checksum                    CHECKSUM("max_rpm")
#define control_P_checksum                  CHECKSUM("control_P")
#define control_I_checksum                  CHECKSUM("control_I")
#define control_D_checksum                  CHECKSUM("control_D")
#define control_smoothing_checksum          CHECKSUM("control_smoothing")
#define reverse_dir_pin_checksum            CHECKSUM("reverse_dir_pin")
#define switch_on_pin_checksum              CHECKSUM("switch_on_pin")
#define max_error_checksum                  CHECKSUM("max_error")
#define debug_checksum                      CHECKSUM("debug")
#define update_freq_checksum                CHECKSUM("update_freq")
#define encoder_select_checksum             CHECKSUM("encoder_select")


EncoderSpindleControl::EncoderSpindleControl()
{
}

void EncoderSpindleControl::on_module_loaded()
{    
    current_rpm = 0;
    current_I_value = 0;
    current_pwm_value = 0;          
    spindle_on = false;    
    stall = 0;
    bad_enc = 0;
    vals_loaded = false;
    
    //pulses_per_rev = THEKERNEL->config->value(spindle_checksum, pulses_per_rev_checksum)->by_default(360.0f)->as_number();    
    max_rpm = THEKERNEL->config->value(spindle_checksum, max_rpm_checksum)->by_default(10000.0f)->as_number();
    default_rpm = THEKERNEL->config->value(spindle_checksum, default_rpm_checksum)->by_default(max_rpm * .1f)->as_number();
    control_P_term = THEKERNEL->config->value(spindle_checksum, control_P_checksum)->by_default(0.0001f)->as_number();
    control_I_term = THEKERNEL->config->value(spindle_checksum, control_I_checksum)->by_default(0.0001f)->as_number();
    control_D_term = THEKERNEL->config->value(spindle_checksum, control_D_checksum)->by_default(0.0001f)->as_number();
    max_error = THEKERNEL->config->value(spindle_checksum, max_error_checksum)->by_default(max_rpm * .25f)->as_number() * .1f;
    debug = THEKERNEL->config->value(spindle_checksum, debug_checksum)->by_default(false)->as_bool();
    update_freq = THEKERNEL->config->value(spindle_checksum, update_freq_checksum)->by_default(20)->as_int();
    encoder_select = THEKERNEL->config->value(spindle_checksum, encoder_select_checksum)->by_default(0)->as_int();

    // Smoothing value is low pass filter time constant in seconds.
    float smoothing_time = THEKERNEL->config->value(spindle_checksum, control_smoothing_checksum)->by_default(0.1f)->as_number();
    if (smoothing_time * update_freq < 1.0f)
        smoothing_decay = 1.0f;
    else
        smoothing_decay = 1.0f / (update_freq * smoothing_time);
             

    // Get the pin for hardware pwm
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(spindle_checksum, pwm_pin_checksum)->by_default("nc")->as_string());
        pwm_pin = smoothie_pin->as_output()->hardware_pwm();
        output_inverted = smoothie_pin->is_inverting();
        delete smoothie_pin;
    }
    
    if (pwm_pin == NULL)
    {
        THEKERNEL->streams->printf("Error: Spindle PWM pin must be P2.0-2.5 or other PWM pin\n");
        delete this;
        return;
    }

    max_pwm = THEKERNEL->config->value(spindle_checksum, max_pwm_checksum)->by_default(1.0f)->as_number();
    
    int period = THEKERNEL->config->value(spindle_checksum, pwm_period_checksum)->by_default(1000)->as_int();
    pwm_pin->period_us(period);
    pwm_pin->write(output_inverted ? 1 : 0);     

    // Get digital out pin for reverse direction
    reverse_dir_pin = THEKERNEL->config->value(spindle_checksum, reverse_dir_pin_checksum)->by_default("nc")->as_string();
    reverse_dir = NULL;
    if(reverse_dir_pin.compare("nc") != 0) {
        reverse_dir = new Pin();
        reverse_dir->from_string(reverse_dir_pin)->as_output()->set(false);
    }

    // Get digital out pin for switch on
    switch_on_pin = THEKERNEL->config->value(spindle_checksum, switch_on_pin_checksum)->by_default("nc")->as_string();
    switch_on = NULL;
    if(switch_on_pin.compare("nc") != 0) {
        switch_on = new Pin();
        switch_on->from_string(switch_on_pin)->as_output()->set(false);
    }
    
    THEKERNEL->slow_ticker->attach(update_freq, this, &EncoderSpindleControl::on_update_speed);        
}

uint32_t EncoderSpindleControl::on_update_speed(uint32_t dummy)
{
    // Calculate current RPM
    if(!vals_loaded) {load_vals();}
    
    long tc;
    if(encoder_select == 0){
        tc = fabsl(HWE->read_enc());
    }else{
        tc = fabsl(SWE0->read_enc());
    }
    
    change = fabsf(tc - pulses);    
    
    current_rpm = ( (change * update_freq) / pulses_per_rev ) * 60; 
    pulses = tc;    

    if (spindle_on) {

        if(change == 0) {
            bad_enc++;
            if(bad_enc > update_freq) {spindle_on = false;}
        }

        float error = (target_rpm - current_rpm) * .1;

        if( error > max_error) {
            error = max_error;
        }          

        current_I_value += control_I_term * error * 1.0f / update_freq;
        current_I_value = confine(current_I_value, -1.0f, 1.0f);

        float new_pwm = default_rpm / max_rpm;
        new_pwm += control_P_term * error;
        new_pwm += current_I_value;
        new_pwm += control_D_term * update_freq * (error - prev_error);
        new_pwm = confine(new_pwm, 0.0f, 1.0f);
        prev_error = error;
       
        current_pwm_value = new_pwm;       

        if (current_pwm_value > max_pwm) {
            current_pwm_value = max_pwm;
        }

        // ignore the error until we get spinning....added stall detection
        if(current_rpm < 30 || stall < update_freq * .75f) {
            current_pwm_value = default_rpm / max_rpm;
            current_I_value = 0;        
            prev_error = 0; 
            stall++;          
        }

    } else {
        current_I_value = 0;
        current_pwm_value = 0;
        stall = 0;
        bad_enc = 0;        
    }
    
    if (output_inverted)
        pwm_pin->write(1.0f - current_pwm_value);
    else
        pwm_pin->write(current_pwm_value);
    
    return 0;
}

void EncoderSpindleControl::load_vals()
{
    if(encoder_select == 0) {
        pulses_per_rev = HWE->get_count_per_rev();
    }else{
        pulses_per_rev = SWE0->get_count_per_rev();
    }
    vals_loaded = true;
}

void EncoderSpindleControl::turn_on() {
    // clear output for reverse direction     
    if(reverse_dir != NULL)
        reverse_dir->set(false);
    // set output for switch on    
    if(switch_on != NULL)
        switch_on->set(true);       
    spindle_on = true;    
}

void EncoderSpindleControl::turn_on_rev() {
    // check if reverse is being used     
    if(reverse_dir != NULL) {
        // set output for switch on
        if(switch_on != NULL)
            switch_on->set(true);
        // set output for reverse direction    
        reverse_dir->set(true);
        spindle_on = true;       
    }    
}

void EncoderSpindleControl::turn_off() {
    // clear output for reverse direction     
    if(reverse_dir != NULL)
        reverse_dir->set(false);
    // clear output for switch on    
    if(switch_on != NULL)
            switch_on->set(false);
    spindle_on = false;    
}


void EncoderSpindleControl::set_speed(int rpm) {
    target_rpm = rpm;
}


void EncoderSpindleControl::report_speed() {
    THEKERNEL->streams->printf("Current RPM: %5.0f  Target RPM: %5.0f  PWM value: %5.3f \n",
                               current_rpm, target_rpm, current_pwm_value);
}


void EncoderSpindleControl::set_p_term(float p) {
    control_P_term = p;
}


void EncoderSpindleControl::set_i_term(float i) {
    control_I_term = i;
}


void EncoderSpindleControl::set_d_term(float d) {
    control_D_term = d;            
}

void EncoderSpindleControl::report_settings() {
    THEKERNEL->streams->printf("P: %0.6f I: %0.6f D: %0.6f\n",
                               control_P_term, control_I_term, control_D_term);
}

