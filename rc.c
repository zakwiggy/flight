/*#######################################################################################
Decodieren eines RC Summen Signals
#######################################################################################*/
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) Holger Buss, Ingo Busker
// + only for non-profit use
// + www.MikroKopter.com
// + porting the sources to other systems or using the software on other systems (except hardware from www.mikrokopter.de) is not allowed
// + see the File "License.txt" for further Informations
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "rc.h"
#include "main.h"
// Achtung: ACT_S3D_SUMMENSIGNAL wird in der Main.h gesetzt

volatile int PPM_in[26];
volatile int PPM_diff[26];  // das diffenzierte Stick-Signal
volatile char Channels,tmpChannels = 0;
volatile unsigned char NewPpmData = 1;

//############################################################################
//zum decodieren des PPM-Signals wird Timer1 mit seiner Input
//Capture Funktion benutzt:
void rc_sum_init (void)
//############################################################################
{
 unsigned char i;
 for(i=0;i<26;i++)
  {
   PPM_in[i] = 0;
   PPM_diff[i] = 0;
  }

    AdNeutralGier = 0;
    AdNeutralRoll = 0;
    AdNeutralNick = 0;
    return;
}

#ifndef ACT_S3D_SUMMENSIGNAL
//############################################################################
//Diese Routine startet und inizialisiert den Timer für RC
ISR(TIMER1_CAPT_vect)
//############################################################################
{
if(!(EE_Parameter.ExtraConfig & CFG_SENSITIVE_RC))
 {
	static unsigned int AltICR=0;
    signed int signal = 0,tmp;
	static int index;

	signal = (unsigned int) ICR1 - AltICR;
	AltICR = ICR1;
    //Syncronisationspause? (3.52 ms < signal < 25.6 ms)
	if((signal > 1100) && (signal < 8000))
        {
        Channels = index;
        if(index >= 4)  NewPpmData = 0;  // Null bedeutet: Neue Daten
        index = 1;
        }
 	else
        {
        if(index < 13)
            {
            if((signal > 250) && (signal < 687))
                {
                signal -= 466;
                // Stabiles Signal
                if(abs(signal - PPM_in[index]) < 6) { if(SenderOkay < 200) SenderOkay += 10; else SenderOkay = 200;}
                tmp = (3 * (PPM_in[index]) + signal) / 4;
                if(tmp > signal+1) tmp--; else
                if(tmp < signal-1) tmp++;
                if(SenderOkay >= 195)  PPM_diff[index] = ((tmp - PPM_in[index]) / 3) * 3;
                else PPM_diff[index] = 0;
                PPM_in[index] = tmp;
                }
            index++;
		   if(PlatinenVersion < 20)
            {	
             if(index == 5) J3High; else J3Low;  // Servosignal an J3 anlegen
             if(index == 6) J4High; else J4Low;  // Servosignal an J4 anlegen
             if(index == 7) J5High; else J5Low;  // Servosignal an J5 anlegen
			}
        }
	}
 }
 else
 {
	static unsigned int AltICR=0;
    static int ppm_in[13];
    static int ppm_diff[13];
    static int old_ppm_in[13];
    static int old_ppm_diff[13];
    signed int signal = 0,tmp;
	static unsigned char index, okay_cnt = 0;
	signal = (unsigned int) ICR1 - AltICR;
	AltICR = ICR1;
    //Syncronisationspause? (3.52 ms < signal < 25.6 ms)
	if((signal > 1100) && (signal < 8000))
        {
        tmpChannels = index;
        if(tmpChannels >= 4 && Channels == tmpChannels)
		 {
          if(okay_cnt > 10)
		   {
		   NewPpmData = 0;  // Null bedeutet: Neue Daten
		   for(index = 0; index < 13; index++)
		    {
			 if(okay_cnt > 30)
			  {
			   old_ppm_in[index] = PPM_in[index];
			   old_ppm_diff[index] = PPM_diff[index];
			  }
		     PPM_in[index] = ppm_in[index];
		     PPM_diff[index] = ppm_diff[index];
		    }
		   }
          if(okay_cnt < 255) okay_cnt++;
		 }
         else
		  {
		   if(okay_cnt > 100) okay_cnt = 10; else okay_cnt = 0;
		   ROT_ON;
		  }
        index = 1;
        if(!MotorenEin) Channels = tmpChannels;
        }
 	else
        {
        if(index < 13)
            {
            if((signal > 250) && (signal < 687))
                {
                signal -= 466;
                // Stabiles Signal
                if((abs(signal - ppm_in[index]) < 6))
				 {
				  if(okay_cnt > 25)  SenderOkay += 10;
				  else
				  if(okay_cnt > 10)  SenderOkay += 2;
				  if(SenderOkay > 200) SenderOkay = 200;
				 }
                tmp = (3 * (ppm_in[index]) + signal) / 4;
                if(tmp > signal+1) tmp--; else
                if(tmp < signal-1) tmp++;
                if(SenderOkay >= 190)  ppm_diff[index] = ((tmp - ppm_in[index]) / 3) * 3;
                else ppm_diff[index] = 0;
                ppm_in[index] = tmp;
                }
			else ROT_ON;
		   if(PlatinenVersion < 20)
            {	
             if(index == 5) J3High; else J3Low;  // Servosignal an J3 anlegen
             if(index == 6) J4High; else J4Low;  // Servosignal an J4 anlegen
             if(index == 7) J5High; else J5Low;  // Servosignal an J5 anlegen
			}
          }
		  if(index < 20) index++;
          else
		  if(index == 20)
		  {
            unsigned char i;
            ROT_ON;
		    index = 30;
            for(i=0;i<13;i++) // restore from older data
			 {
		      PPM_in[i] = old_ppm_in[i];
		      PPM_diff[i] = 0;
//			  okay_cnt /= 2;
 	         }
		  }
	    }
 }
}

#else
//############################################################################
//Diese Routine startet und inizialisiert den Timer für RC
ISR(TIMER1_CAPT_vect)
//############################################################################

{
	static unsigned int AltICR=0;
    signed int signal = 0,tmp;
	static int index;

	signal = (unsigned int) ICR1 - AltICR;
	signal /= 2;
	AltICR = ICR1;
    //Syncronisationspause?
	if((signal > 1100*2) && (signal < 8000*2))
        {
        if(index >= 4)  NewPpmData = 0;  // Null bedeutet: Neue Daten
        index = 1;
        }
 	else
        {
        if(index < 13)
            {
            if((signal > 250) && (signal < 687*2))
                {
                signal -= 962;
                // Stabiles Signal
                if(abs(signal - PPM_in[index]) < 6) { if(SenderOkay < 200) SenderOkay += 10;}
                tmp = (3 * (PPM_in[index]) + signal) / 4;
                if(tmp > signal+1) tmp--; else
                if(tmp < signal-1) tmp++;
                if(SenderOkay >= 195)  PPM_diff[index] = ((tmp - PPM_in[index]) / 3) * 3;
                else PPM_diff[index] = 0;
                PPM_in[index] = tmp;
                }
            index++;
        }
	}
}
#endif

