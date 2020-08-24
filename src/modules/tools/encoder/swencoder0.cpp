/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "swencoder0.h"
#include "Module.h"
#include "Kernel.h"
#include "nuts_bolts.h"
#include "Config.h"
#include "StreamOutputPool.h"
#include "SerialMessage.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StepTicker.h"
#include "Block.h"
#include "SlowTicker.h"
#include "Robot.h"
#include "utils.h"
#include "Pin.h"
#include "Gcode.h"
#include "PwmOut.h" // mbed.h lib
#include "PublicDataRequest.h"

#include <algorithm>

#define swencoder0_checksum                     CHECKSUM("swencoder0")
#define enable_checksum                         CHECKSUM("enable")
#define cpr_checksum                            CHECKSUM("cpr")
#define chan_A_checksum                         CHECKSUM("chan_A")
#define chan_B_checksum                         CHECKSUM("chan_B")
#define chan_I_checksum                         CHECKSUM("chan_I")
#define inv_checksum                            CHECKSUM("inv")
#define debug_checksum                          CHECKSUM("debug")

SwEncoder0* SwEncoder0::instance;

SwEncoder0::SwEncoder0()
{
    
}

void SwEncoder0::on_module_loaded()
{
    if( !THEKERNEL->config->value( swencoder0_checksum, enable_checksum )->by_default(false)->as_bool() ) {
        // as not needed free up resource
        delete this;
        return;
    }

    this->inv = THEKERNEL->config->value( swencoder0_checksum, inv_checksum )->by_default(false)->as_bool();
    this->debug = THEKERNEL->config->value( swencoder0_checksum, debug_checksum )->by_default(false)->as_bool();
    this->decoding_type = 4;
    
    // Get the pin for channel A interrupt
    {
        Pin *smoothie_pin = new Pin();
        if(this->inv == false) {
            smoothie_pin->from_string(THEKERNEL->config->value(swencoder0_checksum, chan_A_checksum)->by_default("nc")->as_string());
        }else{
            smoothie_pin->from_string(THEKERNEL->config->value(swencoder0_checksum, chan_B_checksum)->by_default("nc")->as_string());
        }       
        
        smoothie_pin->as_input();
        if (smoothie_pin->port_number == 0 || smoothie_pin->port_number == 2) {
            PinName pinname = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);
            chan_A = new mbed::InterruptIn(pinname);
            chan_A->rise(this, &SwEncoder0::encode);
            NVIC_SetPriority (SysTick_IRQn, 5);
            //NVIC_SetPriority(EINT3_IRQn, 12);
            chan_A->fall(this, &SwEncoder0::encode);
            NVIC_SetPriority (SysTick_IRQn, 5);                                   
        } else {
            THEKERNEL->streams->printf("Error: Channel A has to be on P0 or P2.\n");
            delete this;
            return;
        }
        delete smoothie_pin;
    }

    // Get the pin for channel B interrupt
    {        
        Pin *smoothie_pin = new Pin();
        if(this->inv == false) {
            smoothie_pin->from_string(THEKERNEL->config->value(swencoder0_checksum, chan_B_checksum)->by_default("nc")->as_string());
        }else{
            smoothie_pin->from_string(THEKERNEL->config->value(swencoder0_checksum, chan_A_checksum)->by_default("nc")->as_string());            
        }    
        smoothie_pin->as_input();
        if (smoothie_pin->port_number == 0 || smoothie_pin->port_number == 2) {
            PinName pinname = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);
            chan_B = new mbed::InterruptIn(pinname);
            if(this->decoding_type != 2) {
                chan_B->rise(this, &SwEncoder0::encode);
                NVIC_SetPriority (SysTick_IRQn, 5);
                chan_B->fall(this, &SwEncoder0::encode);
                NVIC_SetPriority (SysTick_IRQn, 5);
            }
                                  
        } else {
            THEKERNEL->streams->printf("Error: Channel B has to be on P0 or P2.\n");
            delete this;
            return;
        }
        delete smoothie_pin;
    } 
    /*
    // Get the pin for channel I interrupt (optional)
    {
        Pin *smoothie_pin = new Pin();
        smoothie_pin->from_string(THEKERNEL->config->value(swencoder0_checksum, chan_I_checksum)->by_default("nc")->as_string());
        smoothie_pin->as_input();
        if (smoothie_pin->port_number == 0 || smoothie_pin->port_number == 2) {
            PinName pinname = port_pin((PortName)smoothie_pin->port_number, smoothie_pin->pin);
            chan_I = new mbed::InterruptIn(pinname);            
                chan_I->rise(this, &SwEncoder0::on_index);
                NVIC_SetPriority (SysTick_IRQn, 5); 
                this->has_I = true;                                
        } else {
            THEKERNEL->streams->printf("Error: Channel I has to be on P0 or P2.\n");
            this->has_I = false;            
        }
        delete smoothie_pin;
    }  
    */ 
    

    // get count per rev from config
    cpr = THEKERNEL->config->value(swencoder0_checksum, cpr_checksum)->by_default(360)->as_int();
    this->cpr = cpr;      

    //register for events
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA); 

    this->pulses = 0;
    this->revs = 0;
    int chanA  = this->chan_A->read();
    int chanB  = this->chan_B->read();
    this->currState = (chanA << 1) | (chanB);
    this->prevState = this->currState; 

    if(this->debug) { this->t.attach(this, &SwEncoder0::debug_out, 1.0); }   

} // End on_module_loaded 

void SwEncoder0::encode() 
{
    int change = 0;
    int chanA  = this->chan_A->read();
    int chanB  = this->chan_B->read();
 
    //2-bit state.
    this->currState = (chanA << 1) | (chanB);  
    
    if (this->decoding_type == 2) {
 
        //11->00->11->00 is counter clockwise rotation or "forward".
        if ((this->prevState == 3 && this->currState == 0) ||
                (this->prevState == 0 && this->currState == 3)) {
 
            this->pulses++;
 
        }
        //10->01->10->01 is clockwise rotation or "backward".
        else if ((this->prevState == 2 && this->currState == 1) ||
                 (this->prevState == 1 && this->currState == 2)) {
 
            this->pulses--; 
        }
 
    } else { 
        //Entered a new valid state.
        if (((this->currState ^ this->prevState) != INVALID) && (this->currState != this->prevState)) {
            //2 bit state. Right hand bit of prev XOR left hand bit of current
            //gives 0 if clockwise rotation and 1 if counter clockwise rotation.
            change = (this->prevState & PREV_MASK) ^ ((this->currState & CURR_MASK) >> 1);
 
            if (change == 0) {
                change = -1;
            } 
            this->pulses -= change;           
        } 
    } 
    this->prevState = this->currState;     
}

void SwEncoder0::on_console_line_received( void *argument )
{
    if(THEKERNEL->is_halted()) return; // if in halted state ignore any commands

    SerialMessage *msgp = static_cast<SerialMessage *>(argument);
    string possible_command = msgp->message;

    // ignore anything that is not lowercase or a letter
    if(possible_command.empty() || !islower(possible_command[0]) || !isalpha(possible_command[0])) {
        return;
    }

    string cmd = shift_parameter(possible_command);

    // Act depending on command
    if (cmd == "get_count_sw0") {
        long p = this->pulses;
        msgp->stream->printf("SW0 Count = %ld\n", p);
        return;
    }        
}

// returns instance
void SwEncoder0::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(swencoder0_checksum)) return;    
    pdr->set_data_ptr(this);
    pdr->set_taken();
}

int SwEncoder0::get_count_per_rev()
{
    return this->cpr;
}

long SwEncoder0::read_enc()
{
    long p = this->pulses;
    return p;
}

long SwEncoder0::get_revs()
{
    return this->revs;
}

void SwEncoder0::on_index()
{
    this->revs++;
}

void SwEncoder0::debug_out()
{
    long c = this->read_enc();
    THEKERNEL->streams->printf("swencoder0 = %ld \n", c);
}
/*
void SwEncoder0::disable_enc()
{
    this->chan_A->rise(NULL);
    this->chan_A->fall(NULL);
    this->chan_B->rise(NULL);
    this->chan_B->fall(NULL);
    if(this->has_I) 
        chan_I->rise(NULL);           
}

void SwEncoder0::enable_enc()
{
    this->chan_A->rise(this, &SwEncoder0::encode);
    this->chan_A->fall(this, &SwEncoder0::encode);
    this->chan_B->rise(this, &SwEncoder0::encode);
    this->chan_B->fall(this, &SwEncoder0::encode);
    if(this->has_I)    
        chan_I->rise(this, &SwEncoder0::on_index);
}
*/

