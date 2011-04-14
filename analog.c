// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) Holger Buss, Ingo Busker
// + only for non-profit use
// + www.MikroKopter.com
// + porting the sources to other systems or using the software on other systems (except hardware from www.mikrokopter.de) is not allowed
// + see the File "License.txt" for further Informations
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "main.h"
#include "eeprom.h"
volatile int  Aktuell_Nick,Aktuell_Roll,Aktuell_Gier,Aktuell_ax, Aktuell_ay,Aktuell_az, UBat = 100;
volatile int  AdWertNickFilter = 0, AdWertRollFilter = 0, AdWertGierFilter = 0;
volatile int  HiResNick = 2500, HiResRoll = 2500;
volatile int  AdWertNick = 0, AdWertRoll = 0, AdWertGier = 0;
volatile int  AdWertAccRoll = 0,AdWertAccNick = 0,AdWertAccHoch = 0;
volatile char messanzahl_AccHoch = 0;
volatile long Luftdruck = 32000;
volatile long SummenHoehe = 0;
volatile int  StartLuftdruck;
volatile unsigned int  MessLuftdruck = 1023;
unsigned char DruckOffsetSetting;
signed char ExpandBaro = 0;
volatile int VarioMeter = 0;
volatile unsigned int ZaehlMessungen = 0;
unsigned char AnalogOffsetNick = 115,AnalogOffsetRoll = 115,AnalogOffsetGier = 115;
volatile unsigned char AdReady = 1;
float NeutralAccZ_float;
//#######################################################################################
//
void ADC_Init(void)
//#######################################################################################
{
    ADMUX = 0;//Referenz ist extern
    ANALOG_ON;
}

#define DESIRED_H_ADC 800

void SucheLuftruckOffset(void)
{
 unsigned int off;
 ExpandBaro = 0;

#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
 {
  unsigned char off2;
  OCR0A = 150;
  off2 = GetParamByte(PID_PRESSURE_OFFSET);
  if(off2 < 230) off2 += 10;
  OCR0B = off2;
  Delay_ms_Mess(100);
  if(MessLuftdruck > DESIRED_H_ADC) off2 = 240;
  for(; off2 >= 5; off2 -= 5)
   {
   OCR0B = off2;
   Delay_ms_Mess(50);
   printf("*");
   if(MessLuftdruck > DESIRED_H_ADC) break;
   }
   SetParamByte(PID_PRESSURE_OFFSET, off2);
  if(off2 >= 15) off = 140; else off = 0;
  for(; off < 250;off++)
   {
   OCR0A = off;
   Delay_ms_Mess(50);
   printf(".");
   if(MessLuftdruck < DESIRED_H_ADC) break;
   }
   DruckOffsetSetting = off;
 }
#else
  off = GetParamByte(PID_PRESSURE_OFFSET);
  if(off > 20) off -= 10;
  OCR0A = off;
  Delay_ms_Mess(100);
  if(MessLuftdruck < DESIRED_H_ADC) off = 0;
  for(; off < 250;off++)
   {
   OCR0A = off;
   Delay_ms_Mess(50);
   printf(".");
   if(MessLuftdruck < DESIRED_H_ADC) break;
   }
   DruckOffsetSetting = off;
   SetParamByte(PID_PRESSURE_OFFSET, off);
#endif
 if((EE_Parameter.GlobalConfig & CFG_HOEHENREGELUNG) && (DruckOffsetSetting < 10 || DruckOffsetSetting >= 245)) VersionInfo.HardwareError[0] |= FC_ERROR0_PRESSURE;
 OCR0A = off;
 Delay_ms_Mess(300);
}


void SucheGyroOffset(void)
{
 unsigned char i, ready = 0;
 int timeout;
 timeout = SetDelay(2000);
 for(i=140; i != 0; i--)
  {
   if(ready == 3 && i > 10) i = 9;
   ready = 0;
   if(AdWertNick < 1020) AnalogOffsetNick--; else if(AdWertNick > 1030) AnalogOffsetNick++; else ready++;
   if(AdWertRoll < 1020) AnalogOffsetRoll--; else if(AdWertRoll > 1030) AnalogOffsetRoll++; else ready++;
   if(AdWertGier < 1020) AnalogOffsetGier--; else if(AdWertGier > 1030) AnalogOffsetGier++; else ready++;
   I2C_Start(TWI_STATE_GYRO_OFFSET_TX);
   if(AnalogOffsetNick < 10)  { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_NICK; AnalogOffsetNick = 10;}; if(AnalogOffsetNick > 245) { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_NICK; AnalogOffsetNick = 245;};
   if(AnalogOffsetRoll < 10)  { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_ROLL; AnalogOffsetRoll = 10;}; if(AnalogOffsetRoll > 245) { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_ROLL; AnalogOffsetRoll = 245;};
   if(AnalogOffsetGier < 10)  { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_YAW;  AnalogOffsetGier = 10;}; if(AnalogOffsetGier > 245) { VersionInfo.HardwareError[0] |= FC_ERROR0_GYRO_YAW;  AnalogOffsetGier = 245;};
   while(twi_state) if(CheckDelay(timeout)) {printf("\n\r DAC or I2C ERROR! Check I2C, 3Vref, DAC and BL-Ctrl"); break;}
   AdReady = 0;
   ANALOG_ON;
   while(!AdReady);
   if(i<10) Delay_ms_Mess(10);
  }
   Delay_ms_Mess(70);
}

/*
0  n
1  r
2     g
3     y
4     x
5  n
6  r
7     u
8     z
9     L
10 n
11 r
12    g
13    y
14    x
15 n
16 r
17    L
*/

//#######################################################################################
//
ISR(ADC_vect)
//#######################################################################################
{
    static unsigned char kanal=0,state = 0;
	static signed char subcount = 0;
    static signed int gier1, roll1, nick1, nick_filter, roll_filter;
	static signed int accy, accx;
	static long tmpLuftdruck = 0;
	static char messanzahl_Druck = 0;
    switch(state++)
        {
        case 0:
            nick1 = ADC;
            kanal = AD_ROLL;
            break;
        case 1:
            roll1 = ADC;
		    kanal = AD_GIER;
            break;
        case 2:
            gier1 = ADC;
            kanal = AD_ACC_Y;
            break;
        case 3:
            Aktuell_ay = NeutralAccY - ADC;
            accy = Aktuell_ay;
		    kanal = AD_ACC_X;
            break;
        case 4:
            Aktuell_ax = ADC - NeutralAccX;
            accx =  Aktuell_ax;
            kanal = AD_NICK;
            break;
        case 5:
            nick1 += ADC;
            kanal = AD_ROLL;
            break;
        case 6:
            roll1 += ADC;
            kanal = AD_UBAT;
            break;
        case 7:
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
            if(EE_Parameter.ExtraConfig & CFG_3_3V_REFERENCE) UBat = (3 * UBat + (11 * ADC) / 30) / 4; // there were some single FC2.1 with 3.3V reference
			else   
#endif
			UBat = (3 * UBat + ADC / 3) / 4;
		    kanal = AD_ACC_Z;
            break;
       case 8:
            AdWertAccHoch =  (signed int) ADC - NeutralAccZ;
            if(AdWertAccHoch > 1)
             {
              if(NeutralAccZ < 750)
               {
                subcount += 5;
                if(modell_fliegt < 500) subcount += 10;
               }
              if(subcount > 100) { NeutralAccZ++; subcount -= 100;}
             }
             else if(AdWertAccHoch < -1)
             {
              if(NeutralAccZ > 550)
                {
                 subcount -= 5;
                 if(modell_fliegt < 500) subcount -= 10;
                 if(subcount < -100) { NeutralAccZ--; subcount += 100;}
                }
             }
            messanzahl_AccHoch = 1;
            Aktuell_az = ADC;
            Mess_Integral_Hoch += AdWertAccHoch;      // Integrieren
            Mess_Integral_Hoch -= Mess_Integral_Hoch / 1024; // dämfen
 	        kanal = AD_DRUCK;
            break;
   // "case 9:" fehlt hier absichtlich
        case 10:
            nick1 += ADC;
            kanal = AD_ROLL;
            break;
        case 11:
            roll1 += ADC;
		    kanal = AD_GIER;
            break;
        case 12:
            if(PlatinenVersion == 10)  AdWertGier = (ADC + gier1 + 1) / 2;
            else
            if(PlatinenVersion >= 20)  AdWertGier = 2047 - (ADC + gier1);
			else 					   AdWertGier = (ADC + gier1);
            kanal = AD_ACC_Y;
            break;
        case 13:
            Aktuell_ay = NeutralAccY - ADC;
            AdWertAccRoll = (Aktuell_ay + accy);
            kanal = AD_ACC_X;
            break;
        case 14:
            Aktuell_ax = ADC - NeutralAccX;
            AdWertAccNick =  (Aktuell_ax + accx);
            kanal = AD_NICK;
            break;
        case 15:
            nick1 += ADC;
            if(PlatinenVersion == 10) nick1 *= 2; else nick1 *= 4;
            AdWertNick = nick1 / 8;
            nick_filter = (nick_filter + nick1) / 2;
            HiResNick = nick_filter - AdNeutralNick;
            AdWertNickFilter = (AdWertNickFilter + HiResNick) / 2;
            kanal = AD_ROLL;
            break;
        case 16:
            roll1 += ADC;
            if(PlatinenVersion == 10) roll1 *= 2; else roll1 *= 4;
            AdWertRoll = roll1 / 8;
            roll_filter = (roll_filter + roll1) / 2;
            HiResRoll = roll_filter - AdNeutralRoll;
            AdWertRollFilter = (AdWertRollFilter + HiResRoll) / 2;
 	        kanal = AD_DRUCK;
            break;
        case 17:
            state = 0;
			AdReady = 1;
            ZaehlMessungen++;
            // "break" fehlt hier absichtlich
        case 9:
        	MessLuftdruck = ADC;
            tmpLuftdruck += MessLuftdruck;
            if(++messanzahl_Druck >= 18)
            {
				Luftdruck = (7 * Luftdruck + tmpLuftdruck - (18 * 523) * (long)ExpandBaro + 4) / 8;  // -523.19 counts per 10 counts offset step
				HoehenWert = StartLuftdruck - Luftdruck;
				SummenHoehe -= SummenHoehe/SM_FILTER;
				SummenHoehe += HoehenWert;
				VarioMeter = (31 * VarioMeter + 8 * (int)(HoehenWert - SummenHoehe/SM_FILTER))/32;
                tmpLuftdruck /= 2;
                messanzahl_Druck = 18/2;
            }
            kanal = AD_NICK;
            break;
        default:
            kanal = 0; state = 0; kanal = AD_NICK;
            break;
        }
    ADMUX = kanal;
    if(state != 0) ANALOG_ON;
}

