// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) Holger Buss, Ingo Busker
// + only for non-profit use
// + www.MikroKopter.com
// + porting the sources to other systems or using the software on other systems (except hardware from www.mikrokopter.de) is not allowed
// + see the File "License.txt" for further Informations
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "main.h"
#include "eeprom.h"

char DisplayBuff[80] = "Hello World";
unsigned char DispPtr = 0;

unsigned char MaxMenue = 17;
unsigned char MenuePunkt = 0;
unsigned char RemoteKeys = 0;

#define KEY1    0x01
#define KEY2    0x02
#define KEY3    0x04
#define KEY4    0x08
#define KEY5    0x10

void LcdClear(void)
{
 unsigned char i;
 for(i=0;i<80;i++) DisplayBuff[i] = ' ';
}

void Menu_Putchar(char c)
{
	DisplayBuff[DispPtr++] = c;
}

void Menu(void)
 {
  if(RemoteKeys & KEY1) { if(MenuePunkt) MenuePunkt--; else MenuePunkt = MaxMenue;}
  if(RemoteKeys & KEY2) { if(MenuePunkt == MaxMenue) MenuePunkt = 0; else MenuePunkt++;}
  if((RemoteKeys & KEY1) && (RemoteKeys & KEY2)) MenuePunkt = 0;
  LcdClear();
  if(MenuePunkt < 10) {LCD_printfxy(17,0,"[%i]",MenuePunkt);}
  else {LCD_printfxy(16,0,"[%i]",MenuePunkt);};

  switch(MenuePunkt)
   {
    case 0:
           LCD_printfxy(0,0,"+ MikroKopter +");
           LCD_printfxy(0,1,"HW:V%d.%d SW:%d.%d%c",PlatinenVersion/10,PlatinenVersion%10, VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH +'a');
           LCD_printfxy(0,2,"Setting:%d %s", GetActiveParamSet(),Mixer.Name);
		   if(VersionInfo.HardwareError[0]) LCD_printfxy(0,3,"Hardware Error 1:%d !!",VersionInfo.HardwareError[0])
		   else
           if(MissingMotor) LCD_printfxy(0,3,"Missing BL-Ctrl:%d!!",MissingMotor)
		   else
           if(VersionInfo.HardwareError[1] & FC_ERROR1_MIXER) LCD_printfxy(0,3,"Mixer Error!")
		   else
//		   if(VersionInfo.HardwareError[1]) LCD_printfxy(0,3,"Error 2:%d !!",VersionInfo.HardwareError[1])
//		   else
           if(I2CTimeout < 6) LCD_printfxy(0,3,"I2C ERROR!!!")
           break;
    case 1:
          if(EE_Parameter.GlobalConfig & CFG_HOEHENREGELUNG)
           {
           LCD_printfxy(0,0,"Height:  %5i",(int)(HoehenWert/5));
           LCD_printfxy(0,1,"Setpoint:%5i",(int)(SollHoehe/5));
           LCD_printfxy(0,2,"Pressure:%5i",MessLuftdruck);
           LCD_printfxy(0,3,"Offset:  %5i",OCR0A);
           }
           else
           {
           LCD_printfxy(0,0,"Height control");
           LCD_printfxy(0,1,"DISABLED");
           LCD_printfxy(0,2,"Height control");
           LCD_printfxy(0,3,"DISABLED");
           }

           break;
    case 2:
           LCD_printfxy(0,0,"act. bearing");
           LCD_printfxy(0,1,"Nick:      %5i",IntegralNick/1024);
           LCD_printfxy(0,2,"Roll:      %5i",IntegralRoll/1024);
           LCD_printfxy(0,3,"Compass:   %5i",KompassValue);
           break;
    case 3:
           LCD_printfxy(0,0,"K1:%4i  K2:%4i ",PPM_in[1],PPM_in[2]);
           LCD_printfxy(0,1,"K3:%4i  K4:%4i ",PPM_in[3],PPM_in[4]);
           LCD_printfxy(0,2,"K5:%4i  K6:%4i ",PPM_in[5],PPM_in[6]);
           LCD_printfxy(0,3,"K7:%4i  K8:%4i ",PPM_in[7],PPM_in[8]);
           break;
    case 4:
           LCD_printfxy(0,0,"Ni:%4i  Ro:%4i ",PPM_in[EE_Parameter.Kanalbelegung[K_NICK]],PPM_in[EE_Parameter.Kanalbelegung[K_ROLL]]);
           LCD_printfxy(0,1,"Gs:%4i  Gi:%4i ",PPM_in[EE_Parameter.Kanalbelegung[K_GAS]]+120,PPM_in[EE_Parameter.Kanalbelegung[K_GIER]]);
           LCD_printfxy(0,2,"P1:%4i  P2:%4i ",PPM_in[EE_Parameter.Kanalbelegung[K_POTI1]]+127,PPM_in[EE_Parameter.Kanalbelegung[K_POTI2]]+127);
           LCD_printfxy(0,3,"P3:%4i  P4:%4i ",PPM_in[EE_Parameter.Kanalbelegung[K_POTI3]]+127,PPM_in[EE_Parameter.Kanalbelegung[K_POTI4]]+127);
           break;
    case 5:
           LCD_printfxy(0,0,"Gyro - Sensor");
          if(PlatinenVersion == 10)
          {
           LCD_printfxy(0,1,"Nick%4i (%3i.%i)",AdWertNick - AdNeutralNick/8, AdNeutralNick/8, AdNeutralNick%8);
           LCD_printfxy(0,2,"Roll%4i (%3i.%i)",AdWertRoll - AdNeutralRoll/8, AdNeutralRoll/8, AdNeutralRoll%8);
           LCD_printfxy(0,3,"Gier%4i (%3i)",AdNeutralGier - AdWertGier, AdNeutralGier);
          }
          else
          if((PlatinenVersion == 11) || (PlatinenVersion >= 20))
          {
           LCD_printfxy(0,1,"Nick %4i (%3i.%x)",AdWertNick - AdNeutralNick/8, AdNeutralNick/16, (AdNeutralNick%16)/2);
           LCD_printfxy(0,2,"Roll %4i (%3i.%x)",AdWertRoll - AdNeutralRoll/8, AdNeutralRoll/16, (AdNeutralRoll%16)/2);
           LCD_printfxy(0,3,"Yaw  %4i (%3i)",AdNeutralGier - AdWertGier, AdNeutralGier/2);
          }
          else
          if(PlatinenVersion == 13)
          {
           LCD_printfxy(0,1,"Nick %4i (%3i)(%3i)",AdWertNick - AdNeutralNick/8, AdNeutralNick/16,AnalogOffsetNick);
           LCD_printfxy(0,2,"Roll %4i (%3i)(%3i)",AdWertRoll - AdNeutralRoll/8, AdNeutralRoll/16,AnalogOffsetRoll);
           LCD_printfxy(0,3,"Yaw  %4i (%3i)(%3i)",AdNeutralGier - AdWertGier, AdNeutralGier/2,AnalogOffsetGier);
          }
           break;
    case 6:
           LCD_printfxy(0,0,"ACC - Sensor");
           LCD_printfxy(0,1,"Nick %4i (%3i)",AdWertAccNick,NeutralAccX);
           LCD_printfxy(0,2,"Roll %4i (%3i)",AdWertAccRoll,NeutralAccY);
           LCD_printfxy(0,3,"Z    %4i (%3i)",AdWertAccHoch,(int)NeutralAccZ);
           break;
    case 7:
			LCD_printfxy(0,0,"Voltage:   %3i.%1iV",UBat/10, UBat%10);
			LCD_printfxy(0,1,"Current:   %3i.%1iA",Capacity.ActualCurrent/10, Capacity.ActualCurrent%10);
			LCD_printfxy(0,2,"Power:     %4iW",Capacity.ActualPower);
			LCD_printfxy(0,3,"Discharge: %5imAh", Capacity.UsedCapacity);
    	   break;
    case 8:
           LCD_printfxy(0,0,"Receiver");
           LCD_printfxy(0,1,"RC-RSSI:    %4i", PPM_in[0]);
           LCD_printfxy(0,2,"RC-Quality: %4i", SenderOkay);
           LCD_printfxy(0,3,"RC-Channels:%4i", Channels-1);
           break;
    case 9:
           LCD_printfxy(0,0,"Compass");
           LCD_printfxy(0,1,"Heading:  %5i",KompassRichtung);
           LCD_printfxy(0,2,"Value:    %5i",KompassValue);
           LCD_printfxy(0,3,"Soll:     %5i",KompassSollWert);
           break;
    case 10:
           LCD_printfxy(0,0,"Poti1:  %3i",Poti[0]);
           LCD_printfxy(0,1,"Poti2:  %3i",Poti[1]);
           LCD_printfxy(0,2,"Poti3:  %3i",Poti[2]);
           LCD_printfxy(0,3,"Poti4:  %3i",Poti[3]);
           break;
    case 11:
           LCD_printfxy(0,0,"Poti5:  %3i",Poti[4]);
           LCD_printfxy(0,1,"Poti6:  %3i",Poti[5]);
           LCD_printfxy(0,2,"Poti7:  %3i",Poti[6]);
           LCD_printfxy(0,3,"Poti8:  %3i",Poti[7]);
           break;
    case 12:
           LCD_printfxy(0,0,"Servo  " );
           LCD_printfxy(0,1,"Setpoint  %3i",Parameter_ServoNickControl);
           LCD_printfxy(0,2,"Position: %3i",ServoNickValue/4);
           LCD_printfxy(0,3,"Range:%3i-%3i",EE_Parameter.ServoNickMin,EE_Parameter.ServoNickMax);
           break;
    case 13:
           LCD_printfxy(0,0,"ExternControl  " );
           LCD_printfxy(0,1,"Ni:%4i  Ro:%4i ",ExternControl.Nick,ExternControl.Roll);
           LCD_printfxy(0,2,"Gs:%4i  Gi:%4i ",ExternControl.Gas,ExternControl.Gier);
           LCD_printfxy(0,3,"Hi:%4i  Cf:%4i ",ExternControl.Hight,ExternControl.Config);
           break;
    case 14:
           LCD_printfxy(0,0,"BL-Ctrl Errors " );
		   LCD_printfxy(0,1,"%3d %3d %3d %3d ",Motor[0].State & MOTOR_STATE_ERROR_MASK,Motor[1].State & MOTOR_STATE_ERROR_MASK,Motor[2].State & MOTOR_STATE_ERROR_MASK,Motor[3].State & MOTOR_STATE_ERROR_MASK);
		   LCD_printfxy(0,2,"%3d %3d %3d %3d ",Motor[4].State & MOTOR_STATE_ERROR_MASK,Motor[5].State & MOTOR_STATE_ERROR_MASK,Motor[6].State & MOTOR_STATE_ERROR_MASK,Motor[7].State & MOTOR_STATE_ERROR_MASK);
		   LCD_printfxy(0,3,"%3d %3d %3d %3d ",Motor[8].State & MOTOR_STATE_ERROR_MASK,Motor[9].State & MOTOR_STATE_ERROR_MASK,Motor[10].State & MOTOR_STATE_ERROR_MASK,Motor[11].State & MOTOR_STATE_ERROR_MASK);
           break;
    case 15:
           LCD_printfxy(0,0,"BL Temperature" );
		   LCD_printfxy(0,1,"%3i %3i %3i %3i ",Motor[0].Temperature,Motor[1].Temperature,Motor[2].Temperature,Motor[3].Temperature);
		   LCD_printfxy(0,2,"%3i %3i %3i %3i ",Motor[4].Temperature,Motor[5].Temperature,Motor[6].Temperature,Motor[7].Temperature);
		   LCD_printfxy(0,3,"%3i %3i %3i %3i ",Motor[8].Temperature,Motor[9].Temperature,Motor[10].Temperature,Motor[11].Temperature);
           break;
    case 16:
           LCD_printfxy(0,0,"BL-Ctrl found " );
		   LCD_printfxy(0,1," %c   %c   %c   %c ",'-' + 4 * (Motor[0].State>>7),'-' + 5 * (Motor[1].State>>7),'-' + 6 * (Motor[2].State>>7),'-' + 7 * (Motor[3].State>>7));
		   LCD_printfxy(0,2," %c   %c   %c   %c ",'-' + 8 * (Motor[4].State>>7),'-' + 9 * (Motor[5].State>>7),'-' + 10 * (Motor[6].State>>7),'-' + 11 * (Motor[7].State>>7));
		   LCD_printfxy(0,3," %c   -   -   - ",'-' + 12 * (Motor[8].State>>7));
		   if(Motor[9].State>>7)  LCD_printfxy(4,3,"10");
		   if(Motor[10].State>>7) LCD_printfxy(8,3,"11");
		   if(Motor[11].State>>7) LCD_printfxy(12,3,"12");
           break;
    case 17:
           LCD_printfxy(0,0,"Flight-Time  " );
           LCD_printfxy(0,1,"Total:%5umin",FlugMinutenGesamt);
           LCD_printfxy(0,2,"Act:  %5umin",FlugMinuten);
           LCD_printfxy(13,3,"(reset)");
		   if(RemoteKeys & KEY4)
             {
               FlugMinuten = 0;
               SetParamWord(PID_FLIGHT_MINUTES, FlugMinuten);
			 }
           break;
    default:
           if(MenuePunkt == MaxMenue) MaxMenue--;
    	   MenuePunkt = 0;
           break;
    }
    RemoteKeys = 0;
}
