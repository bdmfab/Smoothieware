/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Kernel.h"
#include "Tappingcycles.h"
#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "Robot.h"
#include "Conveyor.h"
#include "SlowTicker.h"
#include "StepperMotor.h"
#include "StreamOutputPool.h"
#include "nuts_bolts.h"

#include <math.h> /* fmod */

// retract modes
#define RETRACT_TO_Z 0
#define RETRACT_TO_R 1

// config names
#define tappingcycles_checksum      CHECKSUM("tappingcycles")
#define enable_checksum             CHECKSUM("enable")
#define ms_update_checksum          CHECKSUM("ms_update")
#define encoder_select_checksum     CHECKSUM("encoder_select")
#define debug_checksum              CHECKSUM("debug")

Tappingcycles::Tappingcycles() {}

void Tappingcycles::on_module_loaded()
{
    // if the module is disabled -> do nothing
    if(! THEKERNEL->config->value(tappingcycles_checksum, enable_checksum)->by_default(false)->as_bool()) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }

    this->update_rate = THEKERNEL->config->value(tappingcycles_checksum, ms_update_checksum)->by_default(50)->as_int(); 
    this->debug = THEKERNEL->config->value(tappingcycles_checksum, debug_checksum)->by_default(false)->as_bool();   
    this->encoder_select = THEKERNEL->config->value(tappingcycles_checksum, encoder_select_checksum)->by_default(0)->as_int(); 
    
    // Settings
    this->on_config_reload(this);

    // events
    this->register_for_event(ON_GCODE_RECEIVED);

    // reset values
    
    this->retract_type  = RETRACT_TO_Z;
    
    this->forward = true;
    this->initial_z = 0;
    this->r_plane = 0;
    this->totalPulses = 0;
    this->last_pulses = 0;
    this->vals_loaded = false;
    this->is_Tapping = false;
    this->reset_sticky();
}

void Tappingcycles::on_config_reload(void *argument)
{
    
}

/* reset all sticky values, called before each cycle */
void Tappingcycles::reset_sticky()
{
    this->sticky_z = 0; // Z depth
    this->sticky_r = 0; // R plane
    this->sticky_f = 0; // feedrate in distance per rev
    this->sticky_q = 0; // peck tapping increment
    this->sticky_s = 0; // tapping rpm    
}

/* update all sticky values, called before each hole */
void Tappingcycles::update_sticky(Gcode *gcode)
{
    if (gcode->has_letter('Z')) this->sticky_z = gcode->get_value('Z');
    if (gcode->has_letter('R')) this->sticky_r = gcode->get_value('R');
    if (gcode->has_letter('F')) this->sticky_f = gcode->get_value('F');
    if (gcode->has_letter('Q')) this->sticky_q = gcode->get_value('Q');
    if (gcode->has_letter('S')) this->sticky_s = gcode->get_int('S');

    // set retract plane
    if (this->retract_type == RETRACT_TO_Z)
        this->r_plane = this->initial_z;
    else
        this->r_plane = this->sticky_r;
}

/* send a formatted Gcode line */
int Tappingcycles::send_gcode(const char* format, ...)
{
    // handle variable arguments
    va_list args;
    va_start(args, format);
    // make the formatted string
    char line[32]; // max length for an gcode line
    int n = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    // debug, print the gcode sended
    //THEKERNEL->streams->printf(">>> %s\n", line);
    // make gcode object and send it (right way)
    Gcode gc(line, &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
    // return the gcode srting length
    return n;
}

void Tappingcycles::tap_hole(Gcode *gcode)
{
     // compile X and Y values
    char x[16] = "";
    char y[16] = "";
    if (gcode->has_letter('X'))
        snprintf(x, sizeof(x), " X%1.4f", gcode->get_value('X'));
    if (gcode->has_letter('Y'))
        snprintf(y, sizeof(y), " Y%1.4f", gcode->get_value('Y'));    
    
    // stop spindle
    this->send_gcode("M5");
    
    // rapids to X/Y
    this->send_gcode("G0%s%s", x, y);
    
    // rapids to retract position (R)
    this->send_gcode("G0 Z%1.4f", this->sticky_r);

    THEKERNEL->conveyor->wait_for_idle();    

    // start spindle sync
    start_sync();
    THEKERNEL->step_ticker->motor[Z_AXIS]->set_direction(true); 

    // start spindle for cycle at S value
    if(this->is_G84) {
        this->send_gcode("M3 S%i", this->sticky_s); 
    }else{
        this->send_gcode("M4 S%i", this->sticky_s);
    }        

    int c = 0;
    this->t.start();
    long last_t = this->t.read_ms();
    long last_pud = last_t;    
    this->forward = true;   
    this->is_Tapping = true;
    bool reached_bottom = false;
    int reverse_delay = 0;

    while(this->forward == true) {

        if(c > this->target_pulses && reached_bottom == false) {
            this->send_gcode("M5");
            if(this->debug){ THEKERNEL->streams->printf("totalPulses = %i \n", c); }
            reached_bottom = true;
        }
        long ct = this->t.read_ms();
        // run function to check encoder counts and set speed
        if(ct >= last_t + this->update_rate) {
            this->calc_speed();
            last_t = ct;
            if(reached_bottom == true) {
                reverse_delay++;
            }
        }
        // update position every 300ms       
        if(ct >= last_pud + 300) {
            this->z_pos_update();
            last_pud = ct;
        }
        // wait for 15 updates
        if(reverse_delay == 15) {
            THEKERNEL->step_ticker->motor[Z_AXIS]->set_direction(false);
            this->forward = false;
        }         
        c = this->totalPulses;        
    }    
    
    this->z_pos_update(); 
    THEKERNEL->conveyor->wait_for_idle();

    if(this->is_G84) {
        this->send_gcode("M4 S%i", this->sticky_s); 
    }else{
        this->send_gcode("M3 S%i", this->sticky_s);
    }         
    
    last_t = this->t.read_ms();
    last_pud = last_t;

    while(c > 10) {
        long ct = this->t.read_ms();
        // run function to check encoder counts and set speed
        if(ct >= last_t + this->update_rate) {
            this->calc_speed();
            last_t = ct;
        }
        // update position every 300ms
        
        if(ct >= last_pud + 300) {
            this->z_pos_update();
            last_pud = ct;
        }        
        c = this->totalPulses;        
    }
    
    this->send_gcode("M5");
    this->t.stop();
    this->t.reset();

    if(this->debug){ THEKERNEL->streams->printf("Final totalPulses = %i \n", c); }
 
    wait_ms(100);

    stop_sync();
    this->z_pos_update();
    // rapids retract at R-Plane (Initial-Z or R)
    this->send_gcode("G0 Z%1.4f", this->initial_z);
    if(this->debug){ THEKERNEL->streams->printf("Tap Completed! \n"); }
    
    THEKERNEL->conveyor->wait_for_idle();
}

void Tappingcycles::start_sync()
{     
    if(this->debug){ THEKERNEL->streams->printf("Sticky_R = %f \n", this->sticky_r); }
    if(this->debug){ THEKERNEL->streams->printf("Sticky_Z = %f \n", this->sticky_z); }
    if(this->debug){ THEKERNEL->streams->printf("Z Axis Res = %f \n", this->z_axis_res); }  
    this->target_pulses = ( this->sticky_r - this->sticky_z ) * this->z_axis_res;
    if(this->debug){ THEKERNEL->streams->printf("Target Pulses = %i \n", this->target_pulses); }
    this->ratio = this->z_axis_res / this->pulses_per_rev;    
    this->calcVal = this->timeslice * this->ratio * this->sticky_f;
    this->last_c = this->get_count();        
}

void Tappingcycles::stop_sync()
{    
    this->pulse_timer.detach();
}

void Tappingcycles::calc_speed()
{
    long c = this->get_count();
    int val = (1 / ((c - this->last_c) * this->calcVal)) * 1000000;
    this->last_c = c;
    /*
    if((val < 0) && (this->forward == true)) {
        THEKERNEL->step_ticker->motor[Z_AXIS]->set_direction(true);
        this->forward = false;
        
    }else if((val > 0) && (this->forward == false)) {
        THEKERNEL->step_ticker->motor[Z_AXIS]->set_direction(false);
        this->forward = true;        
    }
    */
    this->pulse_timer.attach_us(this, &Tappingcycles::do_step, abs(val));   
}

void Tappingcycles::do_step()
{    
    THEKERNEL->step_ticker->motor[Z_AXIS]->man_step();        
    if(this->forward == true) {
        this->totalPulses++;
    }else{
        this->totalPulses--;
    }            
}

void  Tappingcycles::z_pos_update() 
{
    int change = this->totalPulses - this->last_pulses;
    this->last_pulses = this->totalPulses;
    float pos[3];
    THEROBOT->get_axis_position(pos);
    if(this->debug){ THEKERNEL->streams->printf("Z Axis Position = %f \n", pos[2]); }        
    float cur_z = pos[2] + (change / this->z_axis_res);     
    THEROBOT->reset_axis_position(pos[0], pos[1], cur_z);        
}

long Tappingcycles::get_count() 
{
    long c;
    if(this->encoder_select == 0) {
        c = HWE->read_enc();
    }else{
        c = SWE0->read_enc();
    }
    return c;
}
void Tappingcycles::load_vals()
{
    if(this->encoder_select == 0) {
        this->pulses_per_rev = HWE->get_count_per_rev();
    }else{
        this->pulses_per_rev = SWE0->get_count_per_rev();
    }
    this->z_axis_res = THEKERNEL->step_ticker->motor[Z_AXIS]->get_steps_per_mm();    
    this->timeslice = 1000 / this->update_rate;
    this->vals_loaded = true;
}

void Tappingcycles::on_gcode_received(void* argument)
{
    // received gcode
    Gcode *gcode = static_cast<Gcode *>(argument);

    int code;
    // no "G" in gcode    
    if (gcode->has_g) {
        // "G" value
        code = gcode->g;        
    }else{        
        return;
    } 
    // cycle start
    if ((code == 84) || (code == 74)) {
        if(code == 84) {
            this->is_G84 = true;
        }else{
            this->is_G84 = false;
        }
        
        // reset sticky values
        this->reset_sticky();
        // wait for any moves left and current position is update
        THEKERNEL->conveyor->wait_for_idle();
        // get actual position from robot
        float pos[3];
        THEROBOT->get_axis_position(pos);
        // convert to WCS
        Robot::wcs_t wpos= THEROBOT->mcs2wcs(pos);
        // backup Z position as Initial-Z value
        this->initial_z = std::get<Z_AXIS>(wpos); // must use the work coordinate position
        // set retract type
        this->retract_type = RETRACT_TO_Z;               
        this->update_sticky(gcode);
        this->cycle_started = true;
        if(!this->vals_loaded) {this->load_vals();}
        this->tap_hole(gcode);

     // Untested. Xx.x Yx.x is being intercepted by something else.   
    }else if ( (code == 79 ) && (this->cycle_started) ) {
        this->tap_hole(gcode);

    }else if (code == 80) {
        this->cycle_started = false;
    }
}
