// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) Holger Buss, Ingo Busker
// + only for non-profit use
// + www.MikroKopter.com
// + see the File "License.txt" for further Informations
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <stdarg.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "main.h"
#include "uart.h"
#include "libfc.h"
#include "eeprom.h"

#define FC_ADDRESS 1
#define NC_ADDRESS 2
#define MK3MAG_ADDRESS 3
#define BL_CTRL_ADDRESS 5

#define ABO_TIMEOUT 4000 // disable abo after 4 seconds
#define MAX_SENDE_BUFF     170
#define MAX_EMPFANGS_BUFF  170

#define BLPARAM_REVISION 1
#define MASK_SET_PWM_SCALING		0x01
#define MASK_SET_CURRENT_LIMIT		0x02
#define MASK_SET_TEMP_LIMIT			0x04
#define MASK_SET_CURRENT_SCALING	0x08
#define MASK_SET_BITCONFIG			0x10
#define MASK_RESET_CAPCOUNTER		0x20
#define MASK_SET_DEFAULT_PARAMS		0x40
#define MASK_SET_SAVE_EEPROM	 	0x80

typedef struct
{
	unsigned char Revision;			// revision of parameter structure
	unsigned char Address;			// target address
	unsigned char PwmScaling;		// maximum value of pwm setpoint
	unsigned char CurrentLimit;		// current limit in 1A steps
	unsigned char TemperatureLimit;	// in °C
	unsigned char CurrentScaling;	// scaling factor for current measurement
	unsigned char BitConfig;		// see defines above
	unsigned char SetMask;			// filter for active paramters
	unsigned char Checksum;			// checksum for parameter sturcture
}  __attribute__((packed)) BLParameter_t;


unsigned char GetExternalControl = 0,DebugDisplayAnforderung1 = 0, DebugDisplayAnforderung = 0,DebugDataAnforderung = 0,GetVersionAnforderung = 0, GetPPMChannelAnforderung = 0;
unsigned char DisplayLine = 0;
unsigned volatile char SioTmp = 0;
unsigned volatile char NeuerDatensatzEmpfangen = 0;
unsigned volatile char NeueKoordinateEmpfangen = 0;
unsigned volatile char UebertragungAbgeschlossen = 1;
unsigned volatile char CntCrcError = 0;
unsigned volatile char AnzahlEmpfangsBytes = 0;
unsigned volatile char TxdBuffer[MAX_SENDE_BUFF];
unsigned volatile char RxdBuffer[MAX_EMPFANGS_BUFF];

unsigned char *pRxData = 0;
unsigned char RxDataLen = 0;
unsigned volatile char PC_DebugTimeout = 0;
unsigned volatile char PC_MotortestActive = 0;
unsigned char DebugTextAnforderung = 255;

unsigned char PcZugriff = 100;
unsigned char MotorTest[16];
unsigned char MeineSlaveAdresse = 1; // Flight-Ctrl
unsigned char ConfirmFrame;
struct str_DebugOut    DebugOut;
struct str_ExternControl  ExternControl;
struct str_VersionInfo VersionInfo;
struct str_WinkelOut WinkelOut;
struct str_Data3D Data3D;

int Display_Timer, Debug_Timer,Kompass_Timer,Timer3D;
unsigned int DebugDataIntervall = 0, Intervall3D = 0, Display_Interval = 0;
unsigned int AboTimeOut = 0;
unsigned volatile char JetiUpdateModeActive = 0;

const unsigned char ANALOG_TEXT[32][16] PROGMEM =
{
   //1234567890123456
    "AngleNick       ", //0
    "AngleRoll       ",
    "AccNick         ",
    "AccRoll         ",
    "YawGyro         ",
    "Height Value    ", //5
    "AccZ            ",
    "Gas             ",
    "Compass Value   ",
    "Voltage [0.1V]  ",
    "Receiver Level  ", //10
    "Gyro Compass    ",
    "Motor 1         ",
    "Motor 2         ",
    "Motor 3         ",
    "Motor 4         ", //15
    "16              ",
    "17              ",
    "18              ",
    "19              ",
    "Servo           ", //20
    "Hovergas        ",
    "Current [0.1A]  ",
    "Capacity [mAh]  ",
    "Hight Setpoint  ",
    "25              ", //25
    "26              ",
    "Compass Setpoint",
    "I2C-Error       ",
    "BL Limit        ",
    "GPS_Nick        ", //30
    "GPS_Roll        "
};
    
	
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++ Sende-Part der Datenübertragung
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ISR(USART0_TX_vect)
{
 static unsigned int ptr = 0;
 unsigned char tmp_tx;

 if(!UebertragungAbgeschlossen)
  {
   ptr++;                    // die [0] wurde schon gesendet
   tmp_tx = TxdBuffer[ptr];
   if((tmp_tx == '\r') || (ptr == MAX_SENDE_BUFF))
    {
     ptr = 0;
     UebertragungAbgeschlossen = 1;
    }
   UDR0 = tmp_tx;
  }
  else ptr = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++ Empfangs-Part der Datenübertragung, incl. CRC-Auswertung
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ISR(USART0_RX_vect)
{
 static unsigned int crc;
 static unsigned char crc1,crc2,buf_ptr;
 static unsigned char UartState = 0;
 unsigned char CrcOkay = 0;

 if (JetiUpdateModeActive == 1) {   UDR1 = UDR0; return; }
 if (JetiUpdateModeActive == 2) {   RxdBuffer[0] = UDR0; return; }

 SioTmp = UDR0;

 if(buf_ptr >= MAX_SENDE_BUFF)    UartState = 0;
 if(SioTmp == '\r' && UartState == 2)
  {
   UartState = 0;
   crc -= RxdBuffer[buf_ptr-2];
   crc -= RxdBuffer[buf_ptr-1];
   crc %= 4096;
   crc1 = '=' + crc / 64;
   crc2 = '=' + crc % 64;
   CrcOkay = 0;
   if((crc1 == RxdBuffer[buf_ptr-2]) && (crc2 == RxdBuffer[buf_ptr-1])) CrcOkay = 1; else { CrcOkay = 0; CntCrcError++;};
   if(!NeuerDatensatzEmpfangen && CrcOkay) // Datensatz schon verarbeitet
    {
     NeuerDatensatzEmpfangen = 1;
	 AnzahlEmpfangsBytes = buf_ptr + 1;
     RxdBuffer[buf_ptr] = '\r';
	 if(RxdBuffer[2] == 'R')
	  {
	   LcdClear();
	   wdt_enable(WDTO_250MS); // Reset-Commando
	   ServoActive = 0;
	  }

	}
  }
  else
  switch(UartState)
  {
   case 0:
          if(SioTmp == '#' && !NeuerDatensatzEmpfangen) UartState = 1;  // Startzeichen und Daten schon verarbeitet
		  buf_ptr = 0;
		  RxdBuffer[buf_ptr++] = SioTmp;
		  crc = SioTmp;
          break;
   case 1: // Adresse auswerten
		  UartState++;
		  RxdBuffer[buf_ptr++] = SioTmp;
		  crc += SioTmp;
		  break;
   case 2: //  Eingangsdaten sammeln
		  RxdBuffer[buf_ptr] = SioTmp;
		  if(buf_ptr < MAX_EMPFANGS_BUFF) buf_ptr++;
		  else UartState = 0;
		  crc += SioTmp;
		  break;
   default:
          UartState = 0;
          break;
  }
}


// --------------------------------------------------------------------------
void AddCRC(unsigned int wieviele)
{
 unsigned int tmpCRC = 0,i;
 for(i = 0; i < wieviele;i++)
  {
   tmpCRC += TxdBuffer[i];
  }
   tmpCRC %= 4096;
   TxdBuffer[i++] = '=' + tmpCRC / 64;
   TxdBuffer[i++] = '=' + tmpCRC % 64;
   TxdBuffer[i++] = '\r';
  UebertragungAbgeschlossen = 0;
  UDR0 = TxdBuffer[0];
}



// --------------------------------------------------------------------------
void SendOutData(unsigned char cmd,unsigned char address, unsigned char BufferAnzahl, ...) //unsigned char *snd, unsigned char len)
{
 va_list ap;
 unsigned int pt = 0;
 unsigned char a,b,c;
 unsigned char ptr = 0;

 unsigned char *snd = 0;
 int len = 0;

 TxdBuffer[pt++] = '#';				// Startzeichen
 TxdBuffer[pt++] = 'a' + address;		// Adresse (a=0; b=1,...)
 TxdBuffer[pt++] = cmd;				// Commando

 va_start(ap, BufferAnzahl);
 if(BufferAnzahl)
 {
 		snd = va_arg(ap, unsigned char*);
 		len = va_arg(ap, int);
 		ptr = 0;
 		BufferAnzahl--;
 }
 while(len)
  {
	if(len)
	{
	   a = snd[ptr++];
	   len--;
	   if((!len) && BufferAnzahl)
	   	{
			snd = va_arg(ap, unsigned char*);
			len = va_arg(ap, int);
			ptr = 0;
			BufferAnzahl--;
		}
	}
	else a = 0;
	if(len)
	{
		b = snd[ptr++];
		len--;
		if((!len) && BufferAnzahl)
		{
			snd = va_arg(ap, unsigned char*);
			len = va_arg(ap, int);
			ptr = 0;
			BufferAnzahl--;
		}
	}
	else b = 0;
	if(len)
	{
		c = snd[ptr++];
		len--;
		if((!len) && BufferAnzahl)
		{
			snd = va_arg(ap, unsigned char*);
			len = va_arg(ap, int);
			ptr = 0;
			BufferAnzahl--;
		}
	}
	else c = 0;
   TxdBuffer[pt++] = '=' + (a >> 2);
   TxdBuffer[pt++] = '=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4));
   TxdBuffer[pt++] = '=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6));
   TxdBuffer[pt++] = '=' + ( c & 0x3f);
  }
 va_end(ap);
 AddCRC(pt);
}

// --------------------------------------------------------------------------
void Decode64(void)  // die daten werden im rx buffer dekodiert, das geht nur, weil aus 4 byte immer 3 gemacht werden.
{
 unsigned char a,b,c,d;
 unsigned char x,y,z;
 unsigned char ptrIn = 3; // start at begin of data block
 unsigned char ptrOut = 3;
 unsigned char len = AnzahlEmpfangsBytes - 6; // von der Gesamtbytezahl eines Frames gehen 3 Bytes des Headers  ('#',Addr, Cmd) und 3 Bytes des Footers (CRC1, CRC2, '\r') ab.

 while(len)
  {
   a = RxdBuffer[ptrIn++] - '=';
   b = RxdBuffer[ptrIn++] - '=';
   c = RxdBuffer[ptrIn++] - '=';
   d = RxdBuffer[ptrIn++] - '=';

   x = (a << 2) | (b >> 4);
   y = ((b & 0x0f) << 4) | (c >> 2);
   z = ((c & 0x03) << 6) | d;

   if(len--) RxdBuffer[ptrOut++] = x; else break;
   if(len--) RxdBuffer[ptrOut++] = y; else break;
   if(len--) RxdBuffer[ptrOut++] = z;	else break;
  }
  	pRxData = (unsigned char*)&RxdBuffer[3]; // decodierte Daten beginnen beim 4. Byte
	RxDataLen = ptrOut - 3;  // wie viele Bytes wurden dekodiert?

}

// --------------------------------------------------------------------------
void BearbeiteRxDaten(void)
{
 if(!NeuerDatensatzEmpfangen) return;

	unsigned char tempchar1, tempchar2;
	Decode64(); // dekodiere datenblock im Empfangsbuffer
	switch(RxdBuffer[1]-'a') // check for Slave Address
	{
		case FC_ADDRESS: // FC special commands
		switch(RxdBuffer[2])
		{
			case 'K':// Kompasswert
					memcpy((unsigned char *)&KompassValue , (unsigned char *)pRxData, sizeof(KompassValue));
					KompassRichtung = ((540 + KompassValue - KompassSollWert) % 360) - 180;
					break;
			case 't':// Motortest
			        if(AnzahlEmpfangsBytes > 20) memcpy(&MotorTest[0], (unsigned char *)pRxData, sizeof(MotorTest));
			        else memcpy(&MotorTest[0], (unsigned char *)pRxData, 4);
					PC_MotortestActive = 240;
					//while(!UebertragungAbgeschlossen);
					//SendOutData('T', MeineSlaveAdresse, 0);
					PcZugriff = 255;
					break;

			case 'n':// "Get Mixer
					while(!UebertragungAbgeschlossen);
              	    SendOutData('N', FC_ADDRESS, 1, (unsigned char *) &Mixer, sizeof(Mixer) - 1);
					Debug("Mixer lesen");
					break;

			case 'm':// "Write Mixer
                    if(pRxData[0] == EEMIXER_REVISION)
					{
                       memcpy(&Mixer, (unsigned char *)pRxData, sizeof(Mixer) - 1);
                       MixerTable_WriteToEEProm();
					   tempchar1 = 1;
					   VersionInfo.HardwareError[1] &= ~FC_ERROR1_MIXER;
					}
                    else
                    {
						tempchar1 = 0;
					}
					while(!UebertragungAbgeschlossen);
					SendOutData('M', FC_ADDRESS, 1, &tempchar1, sizeof(tempchar1));
					break;

			case 'p': // get PPM Channels
					GetPPMChannelAnforderung = 1;
					PcZugriff = 255;
					break;

			case 'q':// "Get"-Anforderung für Settings
					// Bei Get werden die vom PC einstellbaren Werte vom PC zurückgelesen
					if((10 <= pRxData[0]) && (pRxData[0] < 20))
					{
						tempchar1 = pRxData[0] - 10;
						if(tempchar1< 1) tempchar1 = 1; // limit to 1
						else if(tempchar1 > 5) tempchar1 = 5; // limit to 5
						SetDefaultParameter(tempchar1, 1);
					}
					else if((20 <= pRxData[0]) && (pRxData[0] < 30))
					{
						tempchar1 = pRxData[0] - 20;
						if(tempchar1< 1) tempchar1 = 1; // limit to 1
						else if(tempchar1 > 5) tempchar1 = 5; // limit to 5
						SetDefaultParameter(tempchar1, 0);
					}
					else
					{
						tempchar1 = pRxData[0];
						if(tempchar1 == 0xFF)
						{
							tempchar1 = GetActiveParamSet();
						}
						if(tempchar1< 1) tempchar1 = 1; // limit to 1
						else if(tempchar1 > 5) tempchar1 = 5; // limit to 5
						// load requested parameter set
						ParamSet_ReadFromEEProm(tempchar1);
					}
					while(!UebertragungAbgeschlossen);
					SendOutData('Q', FC_ADDRESS, 2, &tempchar1, sizeof(tempchar1), (unsigned char *) &EE_Parameter, sizeof(EE_Parameter) - 1);
					Debug("Lese Setting %d", tempchar1);

					break;

			case 's': // Parametersatz speichern
					if((1 <= pRxData[0]) && (pRxData[0] <= 5) && (pRxData[1] == EEPARAM_REVISION) && MotorenEin == 0) // check for setting to be in range
					{
						memcpy(&EE_Parameter, (uint8_t*)&pRxData[1], sizeof(EE_Parameter) - 1);
						ParamSet_WriteToEEProm(pRxData[0]);
						Umschlag180Nick = (long) EE_Parameter.WinkelUmschlagNick * 2500L;
						Umschlag180Roll = (long) EE_Parameter.WinkelUmschlagRoll * 2500L;
						tempchar1 = GetActiveParamSet();
					}
					else
					{
						tempchar1 = 0; // mark in response an invlid setting
					}
					while(!UebertragungAbgeschlossen);
					SendOutData('S', FC_ADDRESS, 1, &tempchar1, sizeof(tempchar1));
					if(!MotorenEin) Piep(tempchar1,110);
					LipoDetection(0);
					LIBFC_ReceiverInit(EE_Parameter.Receiver);
					break;
			case 'f': // auf anderen Parametersatz umschalten
			        if((1 <= pRxData[0]) && (pRxData[0] <= 5)) ParamSet_ReadFromEEProm(pRxData[0]);
					tempchar1 = GetActiveParamSet();
					while(!UebertragungAbgeschlossen);
					SendOutData('F', FC_ADDRESS, 1, &tempchar1, sizeof(tempchar1));
					if(!MotorenEin) Piep(tempchar1,110);
					LipoDetection(0);
					LIBFC_ReceiverInit(EE_Parameter.Receiver);
					break;
			case 'y':// serial Potis
					PPM_in[13] = (signed char) pRxData[0]; PPM_in[14] = (signed char) pRxData[1]; PPM_in[15] = (signed char) pRxData[2]; PPM_in[16] = (signed char) pRxData[3];
					PPM_in[17] = (signed char) pRxData[4]; PPM_in[18] = (signed char) pRxData[5]; PPM_in[19] = (signed char) pRxData[6]; PPM_in[20] = (signed char) pRxData[7];
					PPM_in[21] = (signed char) pRxData[8]; PPM_in[22] = (signed char) pRxData[9]; PPM_in[23] = (signed char) pRxData[10]; PPM_in[24] = (signed char) pRxData[11];
					break;

			case 'u': // request BL parameter
				Debug("Reading BL %d", pRxData[0]);
				// try to read BL configuration
				tempchar2 = I2C_ReadBLConfig(pRxData[0]);
				if(tempchar2 == BLCONFIG_SUCCESS) tempchar1 = 1;
				else tempchar1 = 0;
				while(!UebertragungAbgeschlossen); // wait for previous frame to be sent
				SendOutData('U', FC_ADDRESS, 4, &tempchar1, sizeof(tempchar1), &tempchar2, sizeof(tempchar2), &pRxData[0], 1, &BLConfig, sizeof(BLConfig_t));
				break;

			case 'w': // write BL parameter
				Debug("Writing BL %d", pRxData[0]);
				if(RxDataLen >= 1+sizeof(BLConfig_t))
				{
					memcpy(&BLConfig, (uint8_t*)(&pRxData[1]), sizeof(BLConfig_t));
					tempchar2 = I2C_WriteBLConfig(pRxData[0]);
					if(tempchar2 == BLCONFIG_SUCCESS) tempchar1 = 1;
					else tempchar1 = 0; // indicate error
					while(!UebertragungAbgeschlossen); // wait for previous frame to be sent
					SendOutData('W', FC_ADDRESS,2, &tempchar1, sizeof(tempchar1), &tempchar2, sizeof(tempchar2));
				}
				break;
			case 'j':
				tempchar1 = LIBFC_GetCPUType();
				if((tempchar1 == CPU_ATMEGA644P) || (tempchar1 == CPU_ATMEGA1284P))
				{
					uint16_t ubrr = (uint16_t) ((uint32_t) F_CPU/ (8 * 38400L) - 1);

					cli();

					// UART0 & UART1 disable RX and TX-Interrupt
					UCSR0B &= ~((1 << RXCIE0)|(1 << TXCIE0));
					UCSR1B &= ~((1 << RXCIE1)|(1 << TXCIE1));

					// UART0 & UART1 disable receiver and transmitter
					UCSR0B &= ~((1 << TXEN0) | (1 << RXEN0));
					UCSR1B &= ~((1 << TXEN1) | (1 << RXEN1));

					// UART0 & UART1 flush receive buffer explicit
					while ( UCSR1A & (1<<RXC1) ) UDR1;
					while ( UCSR0A & (1<<RXC0) ) UDR0;


					if (pRxData[0] == 0)
					{
						JetiUpdateModeActive = 1;

						// UART0 & UART1 set baudrate
						UBRR1H = (uint8_t)(ubrr>>8);
						UBRR1L = (uint8_t)ubrr;
						UBRR0H = UBRR1H;
						UBRR0L = UBRR1L;
						// UART1 no parity
						UCSR1C &= ~(1 << UPM11);
						UCSR1C &= ~(1 << UPM10);
						// UART1 8-bit
						UCSR1B &= ~(1 << UCSZ12);
						UCSR1C |= (1 << UCSZ11);
						UCSR1C |= (1 << UCSZ10);
					}
					else JetiUpdateModeActive = 2;

					// UART0 & UART1 1 stop bit
					UCSR1C &= ~(1 << USBS1);
					UCSR0C &= ~(1 << USBS0);
					// UART1 clear 9th bit
					UCSR1B &= ~(1<<TXB81);
					// enable receiver and transmitter for UART0 and UART1
					UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
					UCSR1B |= (1 << TXEN1) | (1 << RXEN1);
					// enable RX-Interrupt for UART0 and UART1
					UCSR0B |= (1 << RXCIE0);
					UCSR1B |= (1 << RXCIE1);
					// disable other Interrupts
					TIMSK0 = 0;
					TIMSK1 = 0;
					TIMSK2 = 0;

					sei();
				}
				break;

		} // case FC_ADDRESS:

		default: // any Slave Address

		switch(RxdBuffer[2])
		{
			// 't' comand placed here only for compatibility to BL
			case 't':// Motortest
			        if(AnzahlEmpfangsBytes >= sizeof(MotorTest)) memcpy(&MotorTest[0], (unsigned char *)pRxData, sizeof(MotorTest));
			        else memcpy(&MotorTest[0], (unsigned char *)pRxData, 4);
					while(!UebertragungAbgeschlossen);
					SendOutData('T', MeineSlaveAdresse, 0);
					PC_MotortestActive = 250;
					PcZugriff = 255;
					AboTimeOut = SetDelay(ABO_TIMEOUT);
					break;
			// 'K' comand placed here only for compatibility to old MK3MAG software, that does not send the right Slave Address
			case 'K':// Kompasswert
					memcpy((unsigned char *)&KompassValue , (unsigned char *)pRxData, sizeof(KompassValue));
					KompassRichtung = ((540 + KompassValue - KompassSollWert) % 360) - 180;
					break;
			case 'a':// Texte der Analogwerte
					DebugTextAnforderung = pRxData[0];
					if (DebugTextAnforderung > 31) DebugTextAnforderung = 31;
					PcZugriff = 255;
					break;
			case 'b':
					memcpy((unsigned char *)&ExternControl, (unsigned char *)pRxData, sizeof(ExternControl));
					ConfirmFrame = ExternControl.Frame;
					PcZugriff = 255;
					break;
			case 'c': // Poll the 3D-Data
                    if(!Intervall3D) { if(pRxData[0]) Timer3D = SetDelay(pRxData[0] * 10);}
					Intervall3D = pRxData[0] * 10;
					AboTimeOut = SetDelay(ABO_TIMEOUT);
					break;
			case 'd': // Poll the debug data
					PcZugriff = 255;
					DebugDataIntervall = (unsigned int)pRxData[0] * 10;
					if(DebugDataIntervall > 0) DebugDataAnforderung = 1;
					AboTimeOut = SetDelay(ABO_TIMEOUT);
					break;

			case 'h':// x-1 Displayzeilen
			        PcZugriff = 255;
			        if((pRxData[0] & 0x80) == 0x00) // old format
					{
						DisplayLine = 2;
						Display_Interval = 0;
					}
					else // new format
					{
						RemoteKeys |= ~pRxData[0];
						Display_Interval = (unsigned int)pRxData[1] * 10;
						DisplayLine = 4;
						AboTimeOut = SetDelay(ABO_TIMEOUT);
					}
					DebugDisplayAnforderung = 1;
					break;

			case 'l':// x-1 Displayzeilen
			        PcZugriff = 255;
					MenuePunkt = pRxData[0];
					DebugDisplayAnforderung1 = 1;
					break;
			case 'v': // Version-Anforderung und Ausbaustufe
					GetVersionAnforderung = 1;
					break;

			case 'g'://
					GetExternalControl = 1;
					break;

			default:
				//unsupported command received
				break;
		}
		break; // default:
	}
 	NeuerDatensatzEmpfangen = 0;
 	pRxData = 0;
	RxDataLen = 0;
}

//############################################################################
//Routine für die Serielle Ausgabe
void uart_putchar (char c)
//############################################################################
{
	//Warten solange bis Zeichen gesendet wurde
	loop_until_bit_is_set(UCSR0A, UDRE0);
	//Ausgabe des Zeichens
	UDR0 = c;
}


//############################################################################
//INstallation der Seriellen Schnittstelle
void UART_Init (void)
//############################################################################
{
	unsigned int ubrr = (unsigned int) ((unsigned long) F_CPU/(8 * USART0_BAUD) - 1);

	//Enable TXEN im Register UCR TX-Data Enable & RX Enable
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    // UART Double Speed (U2X)
	UCSR0A |= (1 << U2X0);
	// RX-Interrupt Freigabe
	UCSR0B |= (1 << RXCIE0);
	// TX-Interrupt Freigabe
	UCSR0B |= (1 << TXCIE0);
	// USART0 Baud Rate Register
	// set clock divider
	UBRR0H = (uint8_t)(ubrr >> 8);
	UBRR0L = (uint8_t)ubrr;

	Debug_Timer = SetDelay(DebugDataIntervall);
	Kompass_Timer = SetDelay(220);

	VersionInfo.SWMajor = VERSION_MAJOR;
	VersionInfo.SWMinor = VERSION_MINOR;
	VersionInfo.SWPatch = VERSION_PATCH;
	VersionInfo.ProtoMajor  = VERSION_SERIAL_MAJOR;
	VersionInfo.ProtoMinor  = VERSION_SERIAL_MINOR;

	pRxData = 0;
	RxDataLen = 0;
}

//---------------------------------------------------------------------------------------------
void DatenUebertragung(void)
{
	if(!UebertragungAbgeschlossen) return;

 	if(CheckDelay(AboTimeOut))
	{
		Display_Interval = 0;
		DebugDataIntervall = 0;
		Intervall3D = 0;
	}

	if(((Display_Interval>0 && CheckDelay(Display_Timer)) || DebugDisplayAnforderung) && UebertragungAbgeschlossen)
	{
		if(DisplayLine > 3)// new format
		{
			Menu();
			SendOutData('H', FC_ADDRESS, 1, (uint8_t *)DisplayBuff, 80);
		}
		else // old format
		{
			LCD_printfxy(0,0,"!!! INCOMPATIBLE !!!");
			SendOutData('H', FC_ADDRESS, 2, &DisplayLine, sizeof(DisplayLine), (uint8_t *)DisplayBuff, 20);
			if(DisplayLine++ > 3) DisplayLine = 0;
		}
		Display_Timer = SetDelay(Display_Interval);
		DebugDisplayAnforderung = 0;
	}
	if(DebugDisplayAnforderung1 && UebertragungAbgeschlossen)
	{
		Menu();
		SendOutData('L', FC_ADDRESS, 3, &MenuePunkt, sizeof(MenuePunkt), &MaxMenue, sizeof(MaxMenue), DisplayBuff, sizeof(DisplayBuff));
		DebugDisplayAnforderung1 = 0;
	}
	if(GetVersionAnforderung && UebertragungAbgeschlossen)
	{
		SendOutData('V', FC_ADDRESS, 1, (unsigned char *) &VersionInfo, sizeof(VersionInfo));
		GetVersionAnforderung = 0;
		Debug_OK("Version gesendet");
	}

	if(GetExternalControl && UebertragungAbgeschlossen) 	      // Bei Get werden die vom PC einstellbaren Werte vom PC zurückgelesen
	{
		SendOutData('G',MeineSlaveAdresse, 1, (unsigned char *) &ExternControl, sizeof(ExternControl));
		GetExternalControl = 0;
	}
    if((CheckDelay(Kompass_Timer)) && UebertragungAbgeschlossen)
    	 {
		  WinkelOut.Winkel[0] = (int) (IntegralNick / (EE_Parameter.GyroAccFaktor * 4));  // etwa in 0.1 Grad
		  WinkelOut.Winkel[1] = (int) (IntegralRoll / (EE_Parameter.GyroAccFaktor * 4));  // etwa in 0.1 Grad
		  WinkelOut.UserParameter[0] = Parameter_UserParam1;
		  WinkelOut.UserParameter[1] = Parameter_UserParam2;
    	  SendOutData('k', MK3MAG_ADDRESS, 1, (unsigned char *) &WinkelOut,sizeof(WinkelOut));
          if(WinkelOut.CalcState > 4)  WinkelOut.CalcState = 6; // wird dann in SPI auf Null gesetzt
       	  Kompass_Timer = SetDelay(99);
    	 }
    if(((DebugDataIntervall>0 && CheckDelay(Debug_Timer)) || DebugDataAnforderung) && UebertragungAbgeschlossen)
    	 {
		  CopyDebugValues();
          SendOutData('D', FC_ADDRESS, 1, (unsigned char *) &DebugOut,sizeof(DebugOut));
       	  DebugDataAnforderung = 0;
          if(DebugDataIntervall>0) Debug_Timer = SetDelay(DebugDataIntervall);
    	 }
    if(Intervall3D > 0 && CheckDelay(Timer3D) && UebertragungAbgeschlossen)
    	 {
		  Data3D.Winkel[0] = (int) (IntegralNick / (EE_Parameter.GyroAccFaktor * 4));  // etwa in 0.1 Grad
		  Data3D.Winkel[1] = (int) (IntegralRoll / (EE_Parameter.GyroAccFaktor * 4));  // etwa in 0.1 Grad
          Data3D.Winkel[2] = (int) ((10 * ErsatzKompass) / GIER_GRAD_FAKTOR);
		  Data3D.Centroid[0] = SummeNick >> 9;
		  Data3D.Centroid[1] = SummeRoll >> 9;
		  Data3D.Centroid[2] = Mess_Integral_Gier >> 9;
    	  SendOutData('C', FC_ADDRESS, 1, (unsigned char *) &Data3D,sizeof(Data3D));
          Timer3D = SetDelay(Intervall3D);
    	 }
    if(DebugTextAnforderung != 255) // Texte für die Analogdaten
     {
		unsigned char label[16]; // local sram buffer
		memcpy_P(label, ANALOG_TEXT[DebugTextAnforderung], 16); // read lable from flash to sra
      SendOutData('A', FC_ADDRESS, 2, (unsigned char *)&DebugTextAnforderung, sizeof(DebugTextAnforderung),label, 16);
      DebugTextAnforderung = 255;
	 }
     if(ConfirmFrame && UebertragungAbgeschlossen)   // Datensatz bestätigen
	 {
		SendOutData('B', FC_ADDRESS, 1, (uint8_t*)&ConfirmFrame, sizeof(ConfirmFrame));
      	ConfirmFrame = 0;
     }

     if(GetPPMChannelAnforderung && UebertragungAbgeschlossen)
     {
		 SendOutData('P', FC_ADDRESS, 1, (unsigned char *) &PPM_in, sizeof(PPM_in));
		 GetPPMChannelAnforderung = 0;
	 }

#ifdef DEBUG															// only include functions if DEBUG is defined
     if(SendDebugOutput && UebertragungAbgeschlossen)
     {
		 SendOutData('0', FC_ADDRESS, 1, (unsigned char *) &tDebug, sizeof(tDebug));
		 SendDebugOutput = 0;
	 }
#endif

}


