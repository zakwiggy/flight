// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) Holger Buss, Ingo Busker
// + Nur für den privaten Gebrauch / NON-COMMERCIAL USE ONLY
// + www.MikroKopter.com
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Es gilt für das gesamte Projekt (Hardware, Software, Binärfiles, Sourcecode und Dokumentation),
// + dass eine Nutzung (auch auszugsweise) nur für den privaten und nicht-kommerziellen Gebrauch zulässig ist.
// + Sollten direkte oder indirekte kommerzielle Absichten verfolgt werden, ist mit uns (info@mikrokopter.de) Kontakt
// + bzgl. der Nutzungsbedingungen aufzunehmen.
// + Eine kommerzielle Nutzung ist z.B.Verkauf von MikroKoptern, Bestückung und Verkauf von Platinen oder Bausätzen,
// + Verkauf von Luftbildaufnahmen, usw.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Werden Teile des Quellcodes (mit oder ohne Modifikation) weiterverwendet oder veröffentlicht,
// + unterliegen sie auch diesen Nutzungsbedingungen und diese Nutzungsbedingungen incl. Copyright müssen dann beiliegen
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Sollte die Software (auch auszugesweise) oder sonstige Informationen des MikroKopter-Projekts
// + auf anderen Webseiten oder Medien veröffentlicht werden, muss unsere Webseite "http://www.mikrokopter.de"
// + eindeutig als Ursprung verlinkt und genannt werden
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Keine Gewähr auf Fehlerfreiheit, Vollständigkeit oder Funktion
// + Benutzung auf eigene Gefahr
// + Wir übernehmen keinerlei Haftung für direkte oder indirekte Personen- oder Sachschäden
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Die Portierung oder Nutzung der Software (oder Teile davon) auf andere Systeme (ausser der Hardware von www.mikrokopter.de) ist nur
// + mit unserer Zustimmung zulässig
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Die Funktion printf_P() unterliegt ihrer eigenen Lizenz und ist hiervon nicht betroffen
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Redistributions of source code (with or without modifications) must retain the above copyright notice,
// + this list of conditions and the following disclaimer.
// +   * Neither the name of the copyright holders nor the names of contributors may be used to endorse or promote products derived
// +     from this software without specific prior written permission.
// +   * The use of this project (hardware, software, binary files, sources and documentation) is only permittet
// +     for non-commercial use (directly or indirectly)
// +     Commercial use (for excample: selling of MikroKopters, selling of PCBs, assembly, ...) is only permitted
// +     with our written permission
// +   * If sources or documentations are redistributet on other webpages, out webpage (http://www.MikroKopter.de) must be
// +     clearly linked as origin
// +   * porting the sources to other systems or using the software on other systems (except hardware from www.mikrokopter.de) is not allowed
// +  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// +  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// +  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// +  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// +  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// +  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// +  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// +  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// +  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// +  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// +  POSSIBILITY OF SUCH DAMAGE.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "main.h"


unsigned char PlatinenVersion = 10;
unsigned char SendVersionToNavi = 1;
unsigned char BattLowVoltageWarning = 94;
unsigned int FlugMinuten = 0,FlugMinutenGesamt = 0;
unsigned int FlugSekunden = 0;

unsigned char FoundMotors = 0;
unsigned char JetiBeep = 0; // to allow any Morse-Beeping of the Jeti-Box

void CalMk3Mag(void)
{
 static unsigned char stick = 1;
 if(PPM_in[EE_Parameter.Kanalbelegung[K_NICK]] > -20) stick = 0;
 if((PPM_in[EE_Parameter.Kanalbelegung[K_NICK]] < -70) && !stick)
  {
   stick = 1;
   WinkelOut.CalcState++;
   if(WinkelOut.CalcState > 4)
    {
//     WinkelOut.CalcState = 0; // in Uart.c
     beeptime = 1000;
    }
   else Piep(WinkelOut.CalcState,150);
  }
  DebugOut.Analog[19] = WinkelOut.CalcState;
}


void LipoDetection(unsigned char print)
{
	#define MAX_CELL_VOLTAGE 43 // max cell volatage for LiPO
	unsigned int timer, cells;
	if(print) printf("\n\rBatt:");
	if(EE_Parameter.UnterspannungsWarnung < 50) // automatische Zellenerkennung
	{
		timer = SetDelay(500);
		if(print) while (!CheckDelay(timer));
		// up to 6s LiPo, less than 2s is technical impossible
		for(cells = 2; cells < 7; cells++)
		{
			if(UBat < cells * MAX_CELL_VOLTAGE) break;
		}

		BattLowVoltageWarning = cells * EE_Parameter.UnterspannungsWarnung;
		if(print)
		{
			Piep(cells, 200);
			printf(" %d Cells ", cells);
		}
	}
	else BattLowVoltageWarning = EE_Parameter.UnterspannungsWarnung;
	if(print) printf(" Low warning level: %d.%d",BattLowVoltageWarning/10,BattLowVoltageWarning%10);
}

//############################################################################
//Hauptprogramm
int main (void)
//############################################################################
{
	unsigned int timer,i,timer2 = 0, timerPolling;

    DDRB  = 0x00;
    PORTB = 0x00;
    for(timer = 0; timer < 1000; timer++); // verzögern
#if (defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__))
	PlatinenVersion = 21;
#else
	if(PINB & 0x01)
     {
      if(PINB & 0x02) PlatinenVersion = 13;
       else           PlatinenVersion = 11;
     }
    else
     {
      if(PINB & 0x02) PlatinenVersion = 20;
       else           PlatinenVersion = 10;
     }
#endif
    DDRC  = 0x81; // SCL
    DDRC  |=0x40; // HEF4017 Reset
    PORTC = 0xff; // Pullup SDA
    DDRB  = 0x1B; // LEDs und Druckoffset
    PORTB = 0x01; // LED_Rot
    DDRD  = 0x3E; // Speaker & TXD & J3 J4 J5
	PORTD = 0x47; // LED
    HEF4017R_ON;
    MCUSR &=~(1<<WDRF);
    WDTCSR |= (1<<WDCE)|(1<<WDE);
    WDTCSR = 0;

    beeptime = 2500;
	StickGier = 0; PPM_in[K_GAS] = 0; StickRoll = 0; StickNick = 0;
    if(PlatinenVersion >= 20)  GIER_GRAD_FAKTOR = 1220; else GIER_GRAD_FAKTOR = 1291; // unterschiedlich für ME und ENC
    ROT_OFF;

    Timer_Init();
	TIMER2_Init();
	UART_Init();
    rc_sum_init();
   	ADC_Init();
	I2C_Init(1);
	SPI_MasterInit();
	Capacity_Init();
	LIBFC_Init();
	GRN_ON;
    sei();
	ParamSet_Init();


// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Check connected BL-Ctrls
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Check connected BL-Ctrls
	BLFlags |= BLFLAG_READ_VERSION;
	motor_read = 0;  // read the first I2C-Data
	SendMotorData();
	timer = SetDelay(500);
	while(!(BLFlags & BLFLAG_TX_COMPLETE) && !CheckDelay(timer)); //wait for complete transfer

    printf("\n\rFound BL-Ctrl: ");
    timer = SetDelay(4000);
	for(i=0; i < MAX_MOTORS; i++)
	{
		SendMotorData();
		while(!(BLFlags & BLFLAG_TX_COMPLETE)  && !CheckDelay(timer)); //wait for complete transfer
		if(Mixer.Motor[i][0] > 0) // wait max 4 sec for the BL-Ctrls to wake up
		{
			while(!CheckDelay(timer) && !(Motor[i].State & MOTOR_STATE_PRESENT_MASK) )
			{
				SendMotorData();
				while(!(BLFlags & BLFLAG_TX_COMPLETE) && !CheckDelay(timer)); //wait for complete transfer
			}
		}
		if(Motor[i].State & MOTOR_STATE_PRESENT_MASK)
		{
			printf("%d",i+1);
			FoundMotors++;
//			if(Motor[i].Version & MOTOR_STATE_NEW_PROTOCOL_MASK) printf("(new) ");
		}
	}
	for(i=0; i < MAX_MOTORS; i++)
	{
		if(!(Motor[i].State & MOTOR_STATE_PRESENT_MASK) && Mixer.Motor[i][0] > 0)
		{
			printf("\n\r\n\r!! MISSING BL-CTRL: %d !!",i+1);
			ServoActive = 2; // just in case the FC would be used as camera-stabilizer
		}
		Motor[i].State &= ~MOTOR_STATE_ERROR_MASK; // clear error counter
	}
 	printf("\n\r===================================");

    if(RequiredMotors < FoundMotors) VersionInfo.HardwareError[1] |= FC_ERROR1_MIXER;

	//if(EE_Parameter.GlobalConfig & CFG_HOEHENREGELUNG)
	{
		printf("\n\rCalibrating pressure sensor..");
		timer = SetDelay(1000);
		SucheLuftruckOffset();
		while (!CheckDelay(timer));
		printf("OK\n\r");
	}

	SetNeutral(0);

	ROT_OFF;

    beeptime = 2000;
    ExternControl.Digital[0] = 0x55;


	FlugMinuten = (unsigned int)GetParamByte(PID_FLIGHT_MINUTES) * 256 + (unsigned int)GetParamByte(PID_FLIGHT_MINUTES + 1);
	FlugMinutenGesamt = (unsigned int)GetParamByte(PID_FLIGHT_MINUTES_TOTAL) * 256 + (unsigned int)GetParamByte(PID_FLIGHT_MINUTES_TOTAL + 1);

	if((FlugMinutenGesamt == 0xFFFF) || (FlugMinuten == 0xFFFF))
	{
		FlugMinuten = 0;
		FlugMinutenGesamt = 0;
	}
    printf("\n\rFlight-time %u min  Total:%u min", FlugMinuten, FlugMinutenGesamt);

	printf("\n\rControl: ");
	if (EE_Parameter.GlobalConfig & CFG_HEADING_HOLD) printf("HeadingHold");
	else printf("Normal (ACC-Mode)");

    LcdClear();
    I2CTimeout = 5000;
    WinkelOut.Orientation = 1;
    LipoDetection(1);

	LIBFC_ReceiverInit(EE_Parameter.Receiver);

	printf("\n\r===================================\n\r");
	//SpektrumBinding();
    timer = SetDelay(2000);
	timerPolling = SetDelay(250);

	Debug(ANSI_CLEAR "FC-Start!\n\rFlugzeit: %d min", FlugMinutenGesamt);  	// Note: this won't waste flash memory, if #DEBUG is not active
    DebugOut.Status[0] = 0x01 | 0x02;
	JetiBeep = 0;
	while (1)
	{
	
	if (JetiUpdateModeActive) while (1);
	
	if(CheckDelay(timerPolling))
	{
	  timerPolling = SetDelay(100);
	  LIBFC_Polling();
	}
	if(UpdateMotor && AdReady)      // ReglerIntervall
            {
  		    UpdateMotor=0;
            if(WinkelOut.CalcState) CalMk3Mag();
            else  MotorRegler();
			SendMotorData();
            ROT_OFF;
            if(SenderOkay)  { SenderOkay--; VersionInfo.HardwareError[1] &= ~FC_ERROR1_PPM; }
			else
			{
				TIMSK1 |= _BV(ICIE1); // enable PPM-Input
				PPM_in[0] = 0; // set RSSI to zero on data timeout
				VersionInfo.HardwareError[1] |= FC_ERROR1_PPM;
			}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//if(HoehenReglerAktiv && NaviDataOkay && SenderOkay < 160 && SenderOkay > 10 && FromNaviCtrl_Value.SerialDataOkay > 220) SenderOkay = 160;
//if(HoehenReglerAktiv && NaviDataOkay && SenderOkay < 101 && SenderOkay > 10 && FromNaviCtrl_Value.SerialDataOkay > 1) SenderOkay = 101;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            if(!--I2CTimeout || MissingMotor)
                {
                  if(!I2CTimeout)
				   {
				    I2C_Reset();
                    I2CTimeout = 5;
					DebugOut.Analog[28]++; // I2C-Error
					VersionInfo.HardwareError[1] |= FC_ERROR1_I2C;
					DebugOut.Status[1] |= 0x02; // BL-Error-Status
  				   }
                  if((BeepMuster == 0xffff) && MotorenEin)
                   {
                    beeptime = 10000;
                    BeepMuster = 0x0080;
                   }
                }
            else
                {
                 ROT_OFF;
				 if(!beeptime)
				  {
				   VersionInfo.HardwareError[1] &= ~FC_ERROR1_I2C;
				  }
                }
          if(!UpdateMotor)
		   {
			if(CalculateServoSignals) CalculateServo();
			DatenUebertragung();
			BearbeiteRxDaten();
			if(CheckDelay(timer))
			{
				static unsigned char second;
				timer += 20; // 20 ms interval
				if(MissingMotor)
				 {
				  VersionInfo.HardwareError[1] |= FC_ERROR1_BL_MISSING;
				  DebugOut.Status[1] |= 0x02; // BL-Error-Status
				 }
				 else
				 {
				   VersionInfo.HardwareError[1] &= ~FC_ERROR1_BL_MISSING;
				   if(I2CTimeout > 6) DebugOut.Status[1] &= ~0x02; // BL-Error-Status
				 }

			    if(I2CTimeout > 6) VersionInfo.HardwareError[1] &= ~FC_ERROR1_I2C;

				if(PcZugriff) PcZugriff--;
				else
				{
					ExternControl.Config = 0;
					ExternStickNick = 0;
					ExternStickRoll = 0;
					ExternStickGier = 0;
					if(BeepMuster == 0xffff && SenderOkay == 0)
					{
						beeptime = 15000;
						BeepMuster = 0x0c00;
					}
				}
				if(NaviDataOkay > 200)
				{
					NaviDataOkay--;
					VersionInfo.HardwareError[1] &= ~FC_ERROR1_SPI_RX;
				}
				else
				{
					if(NC_Version.Compatible)
					 {
						VersionInfo.HardwareError[1] |= FC_ERROR1_SPI_RX;
                       if(BeepMuster == 0xffff && MotorenEin)
						{
							beeptime = 15000;
							BeepMuster = 0xA800;
						}
					 }
					GPS_Nick = 0;
					GPS_Roll = 0;
					//if(!beeptime)
                    FromNaviCtrl.CompassValue = -1;
                    NaviDataOkay = 0;
				}
			   if(UBat < BattLowVoltageWarning)
				{
					FC_StatusFlags |= FC_STATUS_LOWBAT;
					if(BeepMuster == 0xffff)
					{
						beeptime = 6000;
						BeepMuster = 0x0300;
					}
				}
				else if(!beeptime) FC_StatusFlags &= ~FC_STATUS_LOWBAT;

				SPI_StartTransmitPacket();
				SendSPI = 4;
				if(!MotorenEin) timer2 = 1450; // 0,5 Minuten aufrunden
				else
                if(++second == 49)
				 {
				   second = 0;
				   FlugSekunden++;
				 }
				if(++timer2 == 2930)  // eine Minute
				 {
				   timer2 = 0;
               	   FlugMinuten++;
	               FlugMinutenGesamt++;
                   SetParamByte(PID_FLIGHT_MINUTES,FlugMinuten / 256);
                   SetParamByte(PID_FLIGHT_MINUTES+1,FlugMinuten % 256);
                   SetParamByte(PID_FLIGHT_MINUTES_TOTAL,FlugMinutenGesamt / 256);
                   SetParamByte(PID_FLIGHT_MINUTES_TOTAL+1,FlugMinutenGesamt % 256);
				   timer = SetDelay(20); // falls "timer += 20;" mal nicht geht
	  		     }
			}
           LED_Update();
           Capacity_Update();
           } //else DebugOut.Analog[26]++;
          }
     if(!SendSPI) { SPI_TransmitByte(); }
    }
 return (1);
}


