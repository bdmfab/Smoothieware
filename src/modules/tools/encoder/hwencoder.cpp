/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "hwencoder.h"
#include "Module.h"
#include "Kernel.h"
#include "nuts_bolts.h"
#include "Config.h"
#include "StreamOutputPool.h"
#include "SerialMessage.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Robot.h"
#include "utils.h"
#include "Pin.h"
#include <string.h>

#define hwencoder_checksum                     CHECKSUM("hwencoder")
#define enable_checksum                        CHECKSUM("enable")
#define cpr_checksum                           CHECKSUM("cpr")
#define debug_checksum                         CHECKSUM("debug")
#define filter_checksum                        CHECKSUM("filter")          
#define inv_checksum                           CHECKSUM("inv")

HwEncoder* HwEncoder::instance;

HwEncoder::HwEncoder()
{
    
}

void HwEncoder::on_module_loaded()
{
    if( !THEKERNEL->config->value( hwencoder_checksum, enable_checksum )->by_default(false)->as_bool() ) {
        // as not needed free up resource
        delete this;
        return;
    }

    // get info from config
    this->debug = THEKERNEL->config->value( hwencoder_checksum, debug_checksum )->by_default(false)->as_bool();
    // not implemented 
    this->inv = THEKERNEL->config->value( hwencoder_checksum, inv_checksum )->by_default(false)->as_bool();
    this->cpr = THEKERNEL->config->value( hwencoder_checksum, cpr_checksum )->by_default(360)->as_int();
    this->filter = THEKERNEL->config->value( hwencoder_checksum, filter_checksum )->by_default(200)->as_int();

    if(this->inv){
        this->hwe.DirectionInvert = QEI_DIRINV_CMPL; 
    }else{
        this->hwe.DirectionInvert = QEI_DIRINV_NONE;
    }           
    this->hwe.SignalMode = QEI_SIGNALMODE_QUAD;
    this->hwe.CaptureMode = QEI_CAPMODE_4X;
    this->hwe.InvertIndex = QEI_INVINX_NONE;   

    // Power control for peripherals
    LPC_SC->PCONP|=0x40000;
    // Pin function register 1
    LPC_SC->PCLKSEL1|=0x01;
    // Pin function register 3 
    LPC_PINCON->PINSEL3 |=0x14100 ;         
    
    QEI_Init(LPC_QEI, &this->hwe);   

    LPC_QEI->QEICON |= 0x01;     
    LPC_QEI->CMPOS0 = 0x00;    
    LPC_QEI->QEILOAD = 0x00; 
    LPC_QEI->VELCOMP = 0x00; 
    LPC_QEI->QEIIEC = 0x01; 
    LPC_QEI->QEICLR = 0x41;    
    // max position
    LPC_QEI->QEIMAXPOS = 0xFFFFFFFE;     
     
    long l_filter = this->filter;
    QEI_SetDigiFilter(LPC_QEI, l_filter);
    QEI_SetPositionComp(LPC_QEI, QEI_COMPPOS_CH_0, 0);  
    QEI_IntCmd(LPC_QEI, QEI_INTFLAG_POS0_Int, ENABLE);
    NVIC_EnableIRQ(QEI_IRQn);     
    
    if(this->debug) { this->t.attach(this, &HwEncoder::debug_out, 1.0); } 
    
    this->inverse_count = false;
} // End on_module_loaded 

void HwEncoder::invert()
{    
        QEI_IntClear(LPC_QEI, QEI_INTFLAG_POS0_Int);
        uint32_t d = QEI_GetStatus(LPC_QEI, QEI_STATUS_DIR);
        if(this->debug) { THEKERNEL->streams->printf("Direction = %ld \n", d); }
        if(d == 1) {
            this->inverse_count = true;
        }else{
            this->inverse_count = false;
        }          
}

extern "C" void QEI_IRQHandler (void)  
{
    if(QEI_GetIntStatus(LPC_QEI, QEI_INTFLAG_POS0_Int)) {
        HWE->invert();
    }        
}

void HwEncoder::debug_out()
{
    long c = this->read_enc();
    THEKERNEL->streams->printf("hwencoder = %ld \n", c);
}

int HwEncoder::get_count_per_rev()
{
    return this->cpr;
}

void HwEncoder::reset()
{             
    QEI_Reset(LPC_QEI, QEI_RESET_POS);
    QEI_Reset(LPC_QEI, QEI_RESET_IDX);    
}

long HwEncoder::read_enc()
{    
    long c = QEI_GetPosition(LPC_QEI);
    if(this->inverse_count) {
        c -= 4294967294;              
    }  
    return c;    
}
