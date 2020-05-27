/**************************************************************************/
/*! 
    @file     ELM402_emu.c
    @author   Dawid "SileliS" Bankowski (d.bankowski@gmail.com)
    
    @brief    ELM 402 software emulator.
    @section LICENSE
    Software License Agreement (BSD License)
    Copyright (c) 2015, D. Bankowski
    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/


//#include <18F4520.h>



//!!!*** DONE ***!!!///
//#include <12F683.h>   //nie sprawdzane  //BOR
//#include <12F609.h>   //nie sprawdzane  //BOR
//#include <12F615.h>   //nie sprawdzane  //BOR
//#include <12F629.h>   //nie sprawdzane  //OSCCAL BOD
//#include <12F675.h>   //nie sprawdzane  //OSCCAL BOD
//#include <12F1571.h>  //nie sprawdzane  //BOR LPBOR
//#include <12LF1571.h>     //nie sprawdzane  //BOR LPBOR
//#include <12F1572.h>    //nie sprawdzane  //BOR LPBOR
#include <12LF1572.h>      //nie sprawdzane  //BOR LPBOR

//#define _use_UP_DOWN_      //if defined output is Voltage level (UP/DOWN) or ifndef output is UART (UART is for debug only)

#define device getenv("DEVICE")

#define ELM402_debounce_period   10    //10 times
#define ELM402_debounce_delay    50    //uS 50
#define ELM_startup_time_delay   50    //mS
#define ELM402_pulse_time        200   //uS
#define ELM402_pulse_width_additional_time  1800  //uS


//#if device=="PIC18F4520"
//   #warning device
//   #warning OSCCAL exists getenv("SFR_VALID:OSCCAL")
//   #if getenv("SFR_VALID:OSCCAL")!=0
//      #warning at getenv("SFR:OSCCAL")
//   #endif
// 
//
//    #warning   RCON getenv("SFR:RCON")
//    #byte      RCON = getenv("SFR:RCON")
//    #warning   BOR getenv("SFR:RCON").0
//    #bit       BOR = RCON.0
//    
//    
//   //#device ADC=10
//   #use delay(internal=4000000)
//   #FUSES nomclr, BROWNOUT
//
//   #define rotoencoder_port_interrupt INT_RB
//   
//   /*ELM402 emulation definitions*/
//   //input definition
//   #define signal_B           PIN_B4
//   #define signal_A           PIN_B5
//  
//   #define pulseWidth         PIN_B1
//   #define outputInvert       PIN_B2
//   
//   //output definition
//   #define output_UP           PIN_C6
//   #define output_DOWN         PIN_C7
//   
//   #ifndef _use_UP_DOWN_ 
//      #WARNING "UART OUTPUT"
//      #use rs232(baud=9600,parity=N,xmit=output_UP,rcv=output_DOWN,bits=8)
//   #endif
//   /*ELM402 emulation definitions*/
//      
//#endif

#if device=="PIC12F629" || device=="PIC12F675" || device=="PIC12F1571" || device=="PIC12LF1571" || device=="PIC12F1572" || device=="PIC12LF1572"
   #warning device
   #warning OSCCAL exists getenv("SFR_VALID:OSCCAL")
   #if getenv("SFR_VALID:OSCCAL")!=0
      #warning at getenv("SFR:OSCCAL")
   #endif
   
   #warning   PCON getenv("SFR:PCON")
   #byte      PCON = getenv("SFR:PCON")
   #warning   BOD getenv("SFR:PCON").0
   #bit       BOD = PCON.0
#endif

#if device=="PIC12F683" || device=="PIC12F609" || device=="PIC12F615"
   #warning device
   #warning OSCCAL exists getenv("SFR_VALID:OSCCAL") //at getenv("SFR:OSCCAL")
   #if getenv("SFR_VALID:OSCCAL")!=0
      #warning at getenv("SFR:OSCCAL")
   #endif
  
   #warning   PCON getenv("SFR:PCON")
   #byte      PCON = getenv("SFR:PCON")
   #warning   BOR getenv("SFR:PCON").0
   #bit       BOD = PCON.0 //w tym chipie jest "BOR", ale dla u³atwienia kodu da³em BOD
#endif

#if device=="PIC12F683" || device=="PIC12F629" || device=="PIC12F675" || device=="PIC12F609" || device=="PIC12F615" || device=="PIC12F1571" || device=="PIC12LF1571" || device=="PIC12F1572" || device=="PIC12LF1572"
   #use delay(internal=4000000)
   #FUSES nomclr, BROWNOUT
   
   #define rotoencoder_port_interrupt INT_RA
   
   /*ELM402 emulation definitions*/
   //input definition
   #define signal_B           PIN_A4
   #define signal_A           PIN_A5
   
   #define pulseWidth         PIN_A3
   #define outputInvert       PIN_A2
   
   //output definition
   #define output_UP           PIN_A0
   #define output_DOWN         PIN_A1
   
   #ifndef _use_UP_DOWN_
      #WARNING "UART OUTPUT" 
      #use rs232(baud=9600,parity=N,xmit=output_UP,rcv=output_DOWN,bits=8) //,stream=PORT1)
   #endif
   /*ELM402 emulation definitions*/
#endif


int OLD_states;
int NEW_states;
const int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};
signed int Out;
int pulses;
void debounce_inputs(int16 time2debounce);      //ELM402 debounce emulation - waits till there is no voltage oscillation on signal_A and signal_B
int get_signals_state(void);     //returns rotor encoder state - inputs are signal_A and signal_B


/* Rotar encoder onPort change interrupt */
#if device=="PIC18F4520"
   #INT_RB
#endif

#if device=="PIC12F629" || device=="PIC12F675" || device=="PIC12F683" || device=="PIC12F609" || device=="PIC12F615" || device=="PIC12F1571" || device=="PIC12LF1571" || device=="PIC12F1572" || device=="PIC12LF1572"
   #INT_RA
#endif

void  onPORT_changes_isr(void) 
{
   //disable_interrupts(rotoencoder_port_interrupt);
   //OLD_states = NEW_states;
   debounce_inputs(ELM402_debounce_period);
   //pulses=pulses+1;
   NEW_states = input_state(signal_A) * 2 + input_state (signal_B);
   //printf("%d %d \n\r", input_state(Pin_B5), input_state(Pin_B4));
   //if (QEM [OLD_states * 4 + NEW_states]!=2)
   Out = Out+ QEM [OLD_states * 4 + NEW_states];
   //printf("\033[H\033[J");

   if (BOD==0)
   {
      reset_cpu();
   }
   
   /*
   #if device=="PIC12F629" || device=="PIC12F675" || device=="PIC12F1571" || device=="PIC12LF1571" || device=="PIC12F1572" || device=="PIC12LF1572"

   #endif
   
   #if device=="PIC12F683" || device=="PIC12F609" || device=="PIC12F615"
       if (BOR==0)
      {
         reset_cpu();
      }
   #endif*/
   
   pulses=pulses+1;
   if (pulses==4)
   {  
      if (Out>3)     //Rotor encoder increment condition
      {
         #ifndef _use_UP_DOWN_ 
            if (input_state(outputInvert))
            {
               printf("minus ");
            }
            else
            {
               printf("plus ");
            }
         #endif
         
         #ifdef _use_UP_DOWN_
            output_bit( output_UP, 1-input_state(outputInvert));
         #endif
         
      }
      else if (Out<-3)     //Rotor encoder decrement condition
      {
         #ifndef _use_UP_DOWN_ 
            if (input_state(outputInvert))
            {
               printf("plus ");
            }
            else
            {
               printf("minus ");
            }
         #endif 
         
         #ifdef _use_UP_DOWN_
               output_bit( output_DOWN, 1-input_state(outputInvert));
         #endif
         
         
      }
      else //if ((Out<3)&(Out>-3)) //Rotor encoder error condition (i.e. rotor contact are corrupted)
      {
         #ifndef _use_UP_DOWN_ 
            printf("error ");
         #endif
         
         #ifdef _use_UP_DOWN_
            #asm
              // nop;
            #endasm
         #endif
      }
      
   Out=0;
   pulses=0;
   delay_us(ELM402_pulse_time+input_state(pulseWidth)*ELM402_pulse_width_additional_time);   
   //#todo: clear outputs
   #ifndef _use_UP_DOWN_ 
    //  printf("\033[H\033[J"); // terminal clear
   #endif
   
   #ifdef _use_UP_DOWN_
      output_bit( output_UP, 0+input_state(outputInvert));
      output_bit( output_DOWN, 0+input_state(outputInvert));
   #endif
   }
   OLD_states = NEW_states;
   
   //clear_interrupt(rotoencoder_port_interrupt);
   
   
   enable_interrupts(rotoencoder_port_interrupt);
   
   
   
}
/* Rotar encoder onPort change interrupt */
  
#zero_ram  
void main()
{
   delay_ms(500);
   enable_interrupts(rotoencoder_port_interrupt);
   enable_interrupts(GLOBAL);
   OLD_states = NEW_states = input_state(signal_A) * 2 + input_state (signal_B);
   pulses=0;
   Out=0;
   delay_ms(ELM_startup_time_delay);
   
   /*------ Brownout reset ------*/
   /* for PIC18f4520 or PIC12f683*/
   /*#if device!="PIC12F629" || device=="PIC12F675" || device=="PIC12F1571" || device=="PIC12LF1571" || device=="PIC12F1572" || device=="PIC12LF1572"
      brownout_enable (TRUE);
      BOR=1;
   #endif*/
   /* for PIC18f4520 or PIC12f683*/
   /* for PIC12f629 PIC12F675             */
   /*#if device=="PIC12F629" || device=="PIC12F675" || device=="PIC12F1571" || device=="PIC12LF1571" || device=="PIC12F1572" || device=="PIC12LF1572"
      BOD =1;
   #endif*/
   /* for PIC12f629 PIC12F675                   
   /*------ Brownout reset ------*/
   while(TRUE)
   {
   /*#if device!="PIC12F629" || device=="PIC12F675" || device=="PIC12F1571" || device=="PIC12LF1571" || device=="PIC12F1572" || device=="PIC12LF1572"
      if (BOD==0)
         printf("BOR");
   #endif*/
  
  //printf("\033[H\033[J");
  //delay_ms(2500);
  
  sleep();
  
   }

}

void debounce_inputs(int16 time2debounce)
{

   unsigned int16 count_low_signal_A=0;
   unsigned int16 count_high_signal_A=0;
   unsigned int16 count_low_signal_B=0;
   unsigned int16 count_high_signal_B=0;
   do
   {
      delay_us(ELM402_debounce_delay);
      if (input(signal_A) == 0)
      {
         count_low_signal_A++;
         count_high_signal_A = 0;
      }   
      else
      {
         count_low_signal_A = 0;
         count_high_signal_A++;
      }   
      if (input(signal_B) == 0)
      {
         count_low_signal_B++;
         count_high_signal_B = 0;
      }   
      else
      {
         count_low_signal_B = 0;
         count_high_signal_B++;
      }
   }
   while((count_low_signal_A < time2debounce) && (count_high_signal_A < time2debounce) || (count_low_signal_B< time2debounce) && (count_high_signal_B < time2debounce));
}

