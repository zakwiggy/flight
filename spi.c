// ######################## SPI - FlightCtrl ###################
#include "main.h"
#include "eeprom.h"


//struct str_ToNaviCtrl_Version   ToNaviCtrl_Version;
//struct str_FromNaviCtrl_Version   FromNaviCtrl_Version;
struct str_ToNaviCtrl   ToNaviCtrl;
struct str_FromNaviCtrl   FromNaviCtrl;
struct str_FromNaviCtrl_Value FromNaviCtrl_Value;
struct str_SPI_VersionInfo NC_Version;
struct str_GPSInfo GPSInfo;

unsigned char              SPI_BufferIndex;
unsigned char              SPI_RxBufferIndex;
signed char FromNC_Rotate_C = 32, FromNC_Rotate_S = 0;

volatile unsigned char     SPI_Buffer[sizeof(FromNaviCtrl)];
unsigned char *SPI_TX_Buffer;

unsigned char SPITransferCompleted, SPI_ChkSum;
unsigned char SPI_RxDataValid,NaviDataOkay = 250;

unsigned char SPI_CommandSequence[] = {SPI_FCCMD_STICK, SPI_FCCMD_USER, SPI_FCCMD_PARAMETER1, SPI_FCCMD_STICK, SPI_FCCMD_MISC, SPI_FCCMD_VERSION, SPI_FCCMD_STICK, SPI_FCCMD_SERVOS, SPI_FCCMD_ACCU};
unsigned char SPI_CommandCounter = 0;
unsigned char NC_ErrorCode = 0;
signed int POI_KameraNick = 0; // in 0,1°
vector16_t MagVec = {0,0,0};

#ifdef USE_SPI_COMMUNICATION

//------------------------------------------------------
void SPI_MasterInit(void)
{
  DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK);    // Set MOSI and SCK output, all others input
  SLAVE_SELECT_DDR_PORT |= (1 << SPI_SLAVE_SELECT);

  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(0<<SPR0)|(0<<SPIE);   // Enable SPI, Master, set clock rate fck/64
  SPSR = 0;//(1<<SPI2X);

  SLAVE_SELECT_PORT |=  (1 << SPI_SLAVE_SELECT);
  SPITransferCompleted = 1;

  //SPDR = 0x00;  // dummy write

  ToNaviCtrl.Sync1 = 0xAA;
  ToNaviCtrl.Sync2 = 0x83;

  ToNaviCtrl.Command = SPI_FCCMD_USER;
  ToNaviCtrl.IntegralNick = 0;
  ToNaviCtrl.IntegralRoll = 0;
  FromNaviCtrl_Value.SerialDataOkay = 0;
  SPI_RxDataValid = 0;

}

//------------------------------------------------------
void SPI_StartTransmitPacket(void)
{
   //if ((SLAVE_SELECT_PORT & (1 << SPI_SLAVE_SELECT)) == 0) return;    // transfer of prev. packet not completed
   if (!SPITransferCompleted) return;
//   _delay_us(30);

   SLAVE_SELECT_PORT &=  ~(1 << SPI_SLAVE_SELECT);  // SelectSlave
   SPI_TX_Buffer = (unsigned char *) &ToNaviCtrl;

   ToNaviCtrl.Command = SPI_CommandSequence[SPI_CommandCounter++];
   if (SPI_CommandCounter >= sizeof(SPI_CommandSequence)) SPI_CommandCounter = 0;

   SPITransferCompleted = 0;
   UpdateSPI_Buffer();                              // update buffer

   SPI_BufferIndex = 1;
  //ebugOut.Analog[16]++;
   // -- Debug-Output ---
   //----
   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); 	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); 	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); 	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
   ToNaviCtrl.Chksum = ToNaviCtrl.Sync1;
   SPDR = ToNaviCtrl.Sync1;                  // Start transmission
//     SLAVE_SELECT_PORT |=  (1 << SPI_SLAVE_SELECT);   // DeselectSlave

}

//------------------------------------------------------
//SIGNAL(SIG_SPI)
void SPI_TransmitByte(void)
{
   static unsigned char SPI_RXState = 0;
   unsigned char rxdata;
   static unsigned char rxchksum;

   if (SPITransferCompleted) return;
   if (!(SPSR & (1 << SPIF))) return;
  SendSPI = 4;

//   _delay_us(30);
  SLAVE_SELECT_PORT |=  (1 << SPI_SLAVE_SELECT);   // DeselectSlave

  rxdata = SPDR;
  switch ( SPI_RXState)
  {
  case 0:

			SPI_RxBufferIndex = 0;
			rxchksum = rxdata;
			if (rxdata == 0x81 )  { SPI_RXState  = 1;  }   // 1. Syncbyte ok

   	   break;

   case 1:
 		    if (rxdata == 0x55) { rxchksum += rxdata; SPI_RXState  = 2;  }   // 2. Syncbyte ok
	         else SPI_RXState  = 0;
   	   break;

   case 2:
		   SPI_Buffer[SPI_RxBufferIndex++]= rxdata;             // get data
           //DebugOut.Analog[19]++;
           if (SPI_RxBufferIndex >= sizeof(FromNaviCtrl))
   		   {

      		if (rxdata == rxchksum)
			{
	          unsigned char *ptr = (unsigned char *)&FromNaviCtrl;
   			  memcpy(ptr, (unsigned char *) SPI_Buffer,  sizeof(SPI_Buffer));
			  SPI_RxDataValid = 1;
			}
			else
			 {
			  SPI_RxDataValid = 0;
			 }


			SPI_RXState  = 0;
   		   }
		  else rxchksum += rxdata;
	break;

  }

   if (SPI_BufferIndex < sizeof(ToNaviCtrl))
     {
 	   SLAVE_SELECT_PORT &=  ~(1 << SPI_SLAVE_SELECT);  // SelectSlave
 	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); 	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); 	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");
	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");	   asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop"); asm volatile ("nop");

	   SPDR = SPI_TX_Buffer[SPI_BufferIndex];
	   ToNaviCtrl.Chksum += SPI_TX_Buffer[SPI_BufferIndex];
	//   SLAVE_SELECT_PORT |=  (1 << SPI_SLAVE_SELECT);   // DeselectSlave

 	 }
	 else SPITransferCompleted = 1;

	 SPI_BufferIndex++;
}


//------------------------------------------------------
void UpdateSPI_Buffer(void)
{
  signed int tmp;
  ToNaviCtrl.IntegralNick = (int) (IntegralNick / (long)(EE_Parameter.GyroAccFaktor * 4));
  ToNaviCtrl.IntegralRoll = (int) (IntegralRoll / (long)(EE_Parameter.GyroAccFaktor * 4));
  ToNaviCtrl.GyroCompass = (10 * ErsatzKompass) / GIER_GRAD_FAKTOR;
  ToNaviCtrl.GyroGier = (signed int) AdNeutralGier - AdWertGier;
  ToNaviCtrl.AccNick = ((int) ACC_AMPLIFY * (NaviAccNick / NaviCntAcc))/4;
  ToNaviCtrl.AccRoll = ((int) ACC_AMPLIFY * (NaviAccRoll / NaviCntAcc))/4;
  NaviCntAcc = 0; NaviAccNick = 0; NaviAccRoll = 0;
//  ToNaviCtrl.User8 = Parameter_UserParam8;
//  ToNaviCtrl.CalState = WinkelOut.CalcState;
   switch(ToNaviCtrl.Command)  //
   {
	 case SPI_FCCMD_USER:
				ToNaviCtrl.Param.Byte[0] = Parameter_UserParam1;
				ToNaviCtrl.Param.Byte[1] = Parameter_UserParam2;
				ToNaviCtrl.Param.Byte[2] = Parameter_UserParam3;
				ToNaviCtrl.Param.Byte[3] = Parameter_UserParam4;
				ToNaviCtrl.Param.Byte[4] = Parameter_UserParam5;
				ToNaviCtrl.Param.Byte[5] = Parameter_UserParam6;
				ToNaviCtrl.Param.Byte[6] = Parameter_UserParam7;
				ToNaviCtrl.Param.Byte[7] = Parameter_UserParam8;
				ToNaviCtrl.Param.Byte[8] = FC_StatusFlags;
                FC_StatusFlags &= ~(FC_STATUS_CALIBRATE | FC_STATUS_START);
	            ToNaviCtrl.Param.Byte[9] = GetActiveParamSet();
				ToNaviCtrl.Param.Byte[10] = ControlHeading;
				ToNaviCtrl.Param.Byte[11] = FC_StatusFlags2;
        break;

     case SPI_FCCMD_ACCU:
     			ToNaviCtrl.Param.Int[0] = Capacity.ActualCurrent; // 0.1A
     			ToNaviCtrl.Param.Int[1] = Capacity.UsedCapacity; // mAh
     			ToNaviCtrl.Param.Byte[4] = (unsigned char) UBat; // 0.1V
     			ToNaviCtrl.Param.Byte[5] = (unsigned char) BattLowVoltageWarning; //0.1V
				ToNaviCtrl.Param.Byte[6] = VarioCharacter;
     	break;

	 case SPI_FCCMD_PARAMETER1:
				ToNaviCtrl.Param.Byte[0] = EE_Parameter.NaviGpsModeControl;     // Parameters for the Naviboard
				ToNaviCtrl.Param.Byte[1] = EE_Parameter.NaviGpsGain;
				ToNaviCtrl.Param.Byte[2] = EE_Parameter.NaviGpsP;
				ToNaviCtrl.Param.Byte[3] = EE_Parameter.NaviGpsI;
				ToNaviCtrl.Param.Byte[4] = EE_Parameter.NaviGpsD;
				ToNaviCtrl.Param.Byte[5] = EE_Parameter.NaviGpsACC;
				ToNaviCtrl.Param.Byte[6] = EE_Parameter.NaviGpsMinSat;
                ToNaviCtrl.Param.Byte[7] = EE_Parameter.NaviStickThreshold;
                ToNaviCtrl.Param.Byte[8] = EE_Parameter.NaviOperatingRadius;
                ToNaviCtrl.Param.Byte[9] = EE_Parameter.NaviWindCorrection;
                ToNaviCtrl.Param.Byte[10] = EE_Parameter.NaviSpeedCompensation;
				ToNaviCtrl.Param.Byte[11] = EE_Parameter.NaviAngleLimitation;
	    break;

	 case SPI_FCCMD_STICK:
              cli();
                tmp = PPM_in[EE_Parameter.Kanalbelegung[K_GAS]];  if(tmp > 127) tmp = 127; else if(tmp < -127) tmp = -127;
				ToNaviCtrl.Param.Byte[0] = (char) tmp;
                tmp = PPM_in[EE_Parameter.Kanalbelegung[K_GIER]]; if(tmp > 127) tmp = 127; else if(tmp < -127) tmp = -127;
				ToNaviCtrl.Param.Byte[1] = (char) tmp;
                tmp = PPM_in[EE_Parameter.Kanalbelegung[K_ROLL]]; if(tmp > 127) tmp = 127; else if(tmp < -127) tmp = -127;
				ToNaviCtrl.Param.Byte[2] = (char) tmp;
                tmp = PPM_in[EE_Parameter.Kanalbelegung[K_NICK]]; if(tmp > 127) tmp = 127; else if(tmp < -127) tmp = -127;
              sei();
				ToNaviCtrl.Param.Byte[3] = (char) tmp;
				ToNaviCtrl.Param.Byte[4] = (unsigned char) Poti[0];
				ToNaviCtrl.Param.Byte[5] = (unsigned char) Poti[1];
				ToNaviCtrl.Param.Byte[6] = (unsigned char) Poti[2];
	            ToNaviCtrl.Param.Byte[7] = (unsigned char) Poti[3];
				ToNaviCtrl.Param.Byte[8] = (unsigned char) Poti[4];
				ToNaviCtrl.Param.Byte[9] = (unsigned char) Poti[5];
				ToNaviCtrl.Param.Byte[10] = (unsigned char) Poti[6];
				ToNaviCtrl.Param.Byte[11] = (unsigned char) Poti[7];
			break;
		case SPI_FCCMD_MISC:
			if(WinkelOut.CalcState > 5)
			{
				WinkelOut.CalcState = 0;
				ToNaviCtrl.Param.Byte[0] = 5;
			}
			else ToNaviCtrl.Param.Byte[0] = WinkelOut.CalcState;
			ToNaviCtrl.Param.Byte[1] = EE_Parameter.NaviPH_LoginTime;
			ToNaviCtrl.Param.Int[1] = (int)(HoehenWert/5);
			ToNaviCtrl.Param.Int[2] = (int)(SollHoehe/5);
			ToNaviCtrl.Param.Byte[6] = EE_Parameter.NaviGpsPLimit;
			ToNaviCtrl.Param.Byte[7] = EE_Parameter.NaviGpsILimit;
			ToNaviCtrl.Param.Byte[8] = EE_Parameter.NaviGpsDLimit;
            ToNaviCtrl.Param.Byte[9] = (unsigned char) SenderOkay;
            ToNaviCtrl.Param.Byte[10] = (unsigned char) PPM_in[0];
			ToNaviCtrl.Param.Byte[11] = DebugOut.Analog[7] / 4; //GasMischanteil
			break;
		case SPI_FCCMD_VERSION:
			ToNaviCtrl.Param.Byte[0] = VERSION_MAJOR;
			ToNaviCtrl.Param.Byte[1] = VERSION_MINOR;
			ToNaviCtrl.Param.Byte[2] = VERSION_PATCH;
			ToNaviCtrl.Param.Byte[3] = NC_SPI_COMPATIBLE;
			ToNaviCtrl.Param.Byte[4] = PlatinenVersion;
			ToNaviCtrl.Param.Byte[5] = VersionInfo.HardwareError[0];
			ToNaviCtrl.Param.Byte[6] = VersionInfo.HardwareError[1];
			ToNaviCtrl.Param.Byte[7] = VersionInfo.HardwareError[2];
			ToNaviCtrl.Param.Byte[8] = VersionInfo.HardwareError[3];
			ToNaviCtrl.Param.Byte[9] = VersionInfo.HardwareError[4];
 	    	break;
 	    case SPI_FCCMD_SERVOS:
 	    	ToNaviCtrl.Param.Byte[0] = EE_Parameter.ServoNickRefresh;     // Parameters for the Servo Control
			ToNaviCtrl.Param.Byte[1] = EE_Parameter.ServoCompInvert;
			ToNaviCtrl.Param.Byte[2] = Parameter_ServoNickControl;
			ToNaviCtrl.Param.Byte[3] = EE_Parameter.ServoNickComp;
			ToNaviCtrl.Param.Byte[4] = EE_Parameter.ServoNickMin;
			ToNaviCtrl.Param.Byte[5] = EE_Parameter.ServoNickMax;
			ToNaviCtrl.Param.Byte[6] = Parameter_ServoRollControl;
			ToNaviCtrl.Param.Byte[7] = EE_Parameter.ServoRollComp;
			ToNaviCtrl.Param.Byte[8] = EE_Parameter.ServoRollMin;
			ToNaviCtrl.Param.Byte[9] = EE_Parameter.ServoRollMax;
 	    	break;
	}

  if(SPI_RxDataValid)
  {
	NaviDataOkay = 250;
	CalculateCompassTimer = 1;
	if(abs(FromNaviCtrl.GPS_Nick) < 512 && abs(FromNaviCtrl.GPS_Roll) < 512 && (EE_Parameter.GlobalConfig & CFG_GPS_AKTIV))
	{
		GPS_Nick = FromNaviCtrl.GPS_Nick;
		GPS_Roll = FromNaviCtrl.GPS_Roll;
	}

	// update compass readings
	MagVec.x = FromNaviCtrl.MagVecX;
	MagVec.y = FromNaviCtrl.MagVecY;
	MagVec.z = FromNaviCtrl.MagVecZ;

	if(FromNaviCtrl.CompassValue <= 360)   KompassValue = FromNaviCtrl.CompassValue;
    KompassRichtung = ((540 + KompassValue - KompassSollWert) % 360) - 180;

    if(FromNaviCtrl.BeepTime > beeptime /*&& !WinkelOut.CalcState*/) beeptime = FromNaviCtrl.BeepTime;

	  switch (FromNaviCtrl.Command)
	  {
	    case SPI_NCCMD_KALMAN:
			FromNaviCtrl_Value.Kalman_K = FromNaviCtrl.Param.sByte[0];
			FromNaviCtrl_Value.Kalman_MaxFusion = FromNaviCtrl.Param.sByte[1];
			FromNaviCtrl_Value.Kalman_MaxDrift = FromNaviCtrl.Param.sByte[2];
			FromNaviCtrl_Value.SerialDataOkay = FromNaviCtrl.Param.Byte[3];
			FromNaviCtrl_Value.GpsZ = FromNaviCtrl.Param.Byte[4];
			FromNC_Rotate_C = FromNaviCtrl.Param.Byte[5];
			FromNC_Rotate_S = FromNaviCtrl.Param.Byte[6];
            if(CareFree && FromNaviCtrl.Param.sInt[4] >= 0)
			 {
			  KompassSollWert = FromNaviCtrl.Param.sInt[4]; // bei Carefree kann NC den Kompass-Sollwinkel vorgeben
			  if(EE_Parameter.CamOrientation)  // Kamera angle is not front
			   {
			    KompassSollWert += 360 - ((unsigned int) EE_Parameter.CamOrientation * 15);
				KompassSollWert %= 360;
			   }	
			 }
			POI_KameraNick = (POI_KameraNick + FromNaviCtrl.Param.sInt[5]) / 2; // FromNaviCtrl.Param.sInt[5]; // Nickwinkel
			break;
		case SPI_NCCMD_VERSION:
			NC_Version.Major = FromNaviCtrl.Param.Byte[0];
			NC_Version.Minor = FromNaviCtrl.Param.Byte[1];
			NC_Version.Patch = FromNaviCtrl.Param.Byte[2];
			NC_Version.Compatible = FromNaviCtrl.Param.Byte[3];
			NC_Version.Hardware = FromNaviCtrl.Param.Byte[4];
			DebugOut.Status[0] |= FromNaviCtrl.Param.Byte[5];
			NC_ErrorCode = FromNaviCtrl.Param.Byte[6];
			DebugOut.Status[1] = (DebugOut.Status[1] & (0x01|0x02)) | (FromNaviCtrl.Param.Byte[6] & (0x04 | 0x08));
			break;

		case SPI_NCCMD_GPSINFO:
			GPSInfo.Flags = FromNaviCtrl.Param.Byte[0];
			GPSInfo.NumOfSats = FromNaviCtrl.Param.Byte[1];
			GPSInfo.SatFix = FromNaviCtrl.Param.Byte[2];
			GPSInfo.Speed = FromNaviCtrl.Param.Byte[3];
			GPSInfo.HomeDistance = FromNaviCtrl.Param.Int[2];
			GPSInfo.HomeBearing = FromNaviCtrl.Param.sInt[3];
			PPM_in[25] = (signed char) FromNaviCtrl.Param.Byte[8]; // WP_EVENT-Channel-Value
            FromNC_AltitudeSpeed = FromNaviCtrl.Param.Byte[9];
 	        FromNC_AltitudeSetpoint = (long) FromNaviCtrl.Param.sInt[5] * 10;  // in cm
			break;
// 0 = 0,1
// 1 = 2,3
// 2 = 4,5
// 3 = 6,7
// 4 = 8,9
// 5 = 10,11
		default:
			break;
	  }
  }
  else
  {
//    KompassValue = 0;
//    KompassRichtung = 0;
	GPS_Nick = 0;
    GPS_Roll = 0;
  }
}

#endif


