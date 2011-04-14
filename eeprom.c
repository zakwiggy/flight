// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) Holger Buss, Ingo Busker
// + Nur für den privaten Gebrauch
// + porting the sources to other systems or using the software on other systems (except hardware from www.mikrokopter.de) is not allowed
// + www.MikroKopter.com
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Es gilt für das gesamte Projekt (Hardware, Software, Binärfiles, Sourcecode und Dokumentation),
// + dass eine Nutzung (auch auszugsweise) nur für den privaten (nicht-kommerziellen) Gebrauch zulässig ist.
// + Sollten direkte oder indirekte kommerzielle Absichten verfolgt werden, ist mit uns (info@mikrokopter.de) Kontakt
// + bzgl. der Nutzungsbedingungen aufzunehmen.
// + Eine kommerzielle Nutzung ist z.B.Verkauf von MikroKoptern, Bestückung und Verkauf von Platinen oder Bausätzen,
// + Verkauf von Luftbildaufnahmen, usw.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Werden Teile des Quellcodes (mit oder ohne Modifikation) weiterverwendet oder veröffentlicht,
// + unterliegen sie auch diesen Nutzungsbedingungen und diese Nutzungsbedingungen incl. Copyright müssen dann beiliegen
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Sollte die Software (auch auszugesweise) oder sonstige Informationen des MikroKopter-Projekts
// + auf anderen Webseiten oder sonstigen Medien veröffentlicht werden, muss unsere Webseite "http://www.mikrokopter.de"
// + eindeutig als Ursprung verlinkt werden
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Keine Gewähr auf Fehlerfreiheit, Vollständigkeit oder Funktion
// + Benutzung auf eigene Gefahr
// + Wir übernehmen keinerlei Haftung für direkte oder indirekte Personen- oder Sachschäden
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Die Portierung der Software (oder Teile davon) auf andere Systeme (ausser der Hardware von www.mikrokopter.de) ist nur
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
// +   * porting to systems other than hardware from www.mikrokopter.de is not allowed
// +  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// +  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// +  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// +  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// +  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// +  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// +  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// +  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN// +  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// +  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// +  POSSIBILITY OF SUCH DAMAGE.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#ifndef EEMEM
#define EEMEM __attribute__ ((section (".eeprom")))
#endif


#include <avr/eeprom.h>
#include <string.h>
#include "eeprom.h"
#include "uart.h"
#include "led.h"
#include "main.h"
#include "fc.h"
#include "twimaster.h"

paramset_t		EE_Parameter;
MixerTable_t	Mixer;
uint8_t RequiredMotors;


uint8_t RAM_Checksum(uint8_t* pBuffer, uint16_t len)
{
	uint8_t crc = 0xAA;
	uint16_t i;

	for(i=0; i<len; i++)
	{
		crc += pBuffer[i];
	}
	return crc;
}

uint8_t EEProm_Checksum(uint16_t EEAddr, uint16_t len)
{
	uint8_t crc = 0xAA;
	uint16_t off;

	for(off=0; off<len; off++)
	{
		crc += eeprom_read_byte((uint8_t*)(EEAddr + off));;
	}
	return crc;
}

void ParamSet_DefaultStickMapping(void)
{
	EE_Parameter.Kanalbelegung[K_GAS]   = 1;
	EE_Parameter.Kanalbelegung[K_ROLL]  = 2;
	EE_Parameter.Kanalbelegung[K_NICK]  = 3;
	EE_Parameter.Kanalbelegung[K_GIER]  = 4;
	EE_Parameter.Kanalbelegung[K_POTI1] = 5;
	EE_Parameter.Kanalbelegung[K_POTI2] = 6;
	EE_Parameter.Kanalbelegung[K_POTI3] = 7;
	EE_Parameter.Kanalbelegung[K_POTI4] = 8;
	EE_Parameter.Kanalbelegung[K_POTI5] = 9;
	EE_Parameter.Kanalbelegung[K_POTI6] = 10;
	EE_Parameter.Kanalbelegung[K_POTI7] = 11;
	EE_Parameter.Kanalbelegung[K_POTI8] = 12;
}


/***************************************************/
/*    Default Values for parameter set 1           */
/***************************************************/
void CommonDefaults(void)
{
	EE_Parameter.Revision = EEPARAM_REVISION;

	if(PlatinenVersion >= 20)
	{
		EE_Parameter.Gyro_D = 10;
		EE_Parameter.Driftkomp = 0;
		EE_Parameter.GyroAccFaktor = 27;
		EE_Parameter.WinkelUmschlagNick = 78;
		EE_Parameter.WinkelUmschlagRoll = 78;
	}
	else
	{
		EE_Parameter.Gyro_D = 3;
		EE_Parameter.Driftkomp = 32;
		EE_Parameter.GyroAccFaktor = 30;
		EE_Parameter.WinkelUmschlagNick = 85;
		EE_Parameter.WinkelUmschlagRoll = 85;
	}
	EE_Parameter.GlobalConfig = CFG_ACHSENKOPPLUNG_AKTIV | CFG_KOMPASS_AKTIV | CFG_GPS_AKTIV | CFG_HOEHEN_SCHALTER;
	EE_Parameter.ExtraConfig = /*CFG2_HEIGHT_LIMIT |*/ CFG2_VARIO_BEEP;
	EE_Parameter.Receiver = RECEIVER_JETI;
	EE_Parameter.MotorSafetySwitch = 0; 
	EE_Parameter.ExternalControl = 0;

	EE_Parameter.Gas_Min = 8;             // Wert : 0-32
	EE_Parameter.Gas_Max = 230;           // Wert : 33-247
	EE_Parameter.KompassWirkung = 64;    // Wert : 0-247

	EE_Parameter.Hoehe_MinGas = 30;
	EE_Parameter.MaxHoehe     = 255;         // Wert : 0-247   255 -> Poti1
	EE_Parameter.Hoehe_P      = 15;          // Wert : 0-32
	EE_Parameter.Luftdruck_D  = 30;          // Wert : 0-247
	EE_Parameter.Hoehe_ACC_Wirkung = 0;     // Wert : 0-247
	EE_Parameter.Hoehe_HoverBand = 8;     	  // Wert : 0-247
	EE_Parameter.Hoehe_GPS_Z = 64;           // Wert : 0-247
	EE_Parameter.Hoehe_StickNeutralPoint = 0;// Wert : 0-247 (0 = Hover-Estimation)
	EE_Parameter.Hoehe_Verstaerkung = 15;    // Wert : 0-50

	EE_Parameter.UserParam1 =   0;           // zur freien Verwendung
	EE_Parameter.UserParam2 =   0;           // zur freien Verwendung
	EE_Parameter.UserParam3 =   0;           // zur freien Verwendung
	EE_Parameter.UserParam4 =   0;           // zur freien Verwendung
	EE_Parameter.UserParam5 =   0;           // zur freien Verwendung
	EE_Parameter.UserParam6 =   0;           // zur freien Verwendung
	EE_Parameter.UserParam7 = 0;             // zur freien Verwendung
	EE_Parameter.UserParam8 = 0;             // zur freien Verwendung

	EE_Parameter.ServoNickControl = 128;     // Wert : 0-247     // Stellung des Servos
	EE_Parameter.ServoNickComp = 50;         // Wert : 0-247     // Einfluss Gyro/Servo
	EE_Parameter.ServoCompInvert = 2;        // Wert : 0-247     // Richtung Einfluss Gyro/Servo
	EE_Parameter.ServoNickMin = 15;          // Wert : 0-247     // Anschlag
	EE_Parameter.ServoNickMax = 230;         // Wert : 0-247     // Anschlag
	EE_Parameter.ServoNickRefresh = 4;
	EE_Parameter.Servo3 = 125;
	EE_Parameter.Servo4 = 125;
	EE_Parameter.Servo5 = 125;
	EE_Parameter.ServoRollControl = 128;     // Wert : 0-247     // Stellung des Servos
	EE_Parameter.ServoRollComp = 85;         // Wert : 0-247     // Einfluss Gyro/Servo
	EE_Parameter.ServoRollMin = 70;          // Wert : 0-247     // Anschlag
	EE_Parameter.ServoRollMax = 220;         // Wert : 0-247     // Anschlag
	EE_Parameter.ServoManualControlSpeed = 60;
	EE_Parameter.CamOrientation = 0;         // Wert : 0-24 -> 0-360 -> 15° steps

	EE_Parameter.J16Bitmask = 95;
	EE_Parameter.J17Bitmask = 243;
	EE_Parameter.WARN_J16_Bitmask = 0xAA;
	EE_Parameter.WARN_J17_Bitmask = 0xAA;
	EE_Parameter.J16Timing = 20;
	EE_Parameter.J17Timing = 20;

	EE_Parameter.LoopGasLimit = 50;
	EE_Parameter.LoopThreshold = 90;         // Wert: 0-247  Schwelle für Stickausschlag
	EE_Parameter.LoopHysterese = 50;
	EE_Parameter.BitConfig = 0;              // Bitcodiert: 0x01=oben, 0x02=unten, 0x04=links, 0x08=rechts / wird getrennt behandelt

	EE_Parameter.NaviGpsModeControl = 254; // 254 -> Poti 2
	EE_Parameter.NaviGpsGain = 100;
	EE_Parameter.NaviGpsP = 90;
	EE_Parameter.NaviGpsI = 90;
	EE_Parameter.NaviGpsD = 90;
	EE_Parameter.NaviGpsPLimit = 75;
	EE_Parameter.NaviGpsILimit = 75;
	EE_Parameter.NaviGpsDLimit = 75;
	EE_Parameter.NaviGpsACC = 0;
	EE_Parameter.NaviGpsMinSat = 6;
	EE_Parameter.NaviStickThreshold = 8;
	EE_Parameter.NaviWindCorrection = 90;
	EE_Parameter.NaviSpeedCompensation = 30;
	EE_Parameter.NaviOperatingRadius = 245;
	EE_Parameter.NaviAngleLimitation = 100;
	EE_Parameter.NaviPH_LoginTime = 2;
	EE_Parameter.OrientationAngle = 0;
	EE_Parameter.OrientationModeControl = 0;
	EE_Parameter.UnterspannungsWarnung = 33; // Wert : 0-247 ( Automatische Zellenerkennung bei < 50)
	EE_Parameter.NotGas = 45;                // Wert : 0-247     // Gaswert bei Empangsverlust
	EE_Parameter.NotGasZeit = 90;            // Wert : 0-247     // Zeit bis auf NotGas geschaltet wird, wg. Rx-Problemen
}

void ParamSet_DefaultSet1(void) // sport
{
	CommonDefaults();
	EE_Parameter.Stick_P = 14;            // Wert : 1-20
	EE_Parameter.Stick_D = 16;            // Wert : 0-20
	EE_Parameter.Gier_P = 12;             // Wert : 1-20
	EE_Parameter.Gyro_P = 80;             // Wert : 0-247
	EE_Parameter.Gyro_I = 150;            // Wert : 0-247
	EE_Parameter.Gyro_Gier_P = 80;        // Wert : 0-247
	EE_Parameter.Gyro_Gier_I = 150;       // Wert : 0-247
	EE_Parameter.Gyro_Stability = 6; 	  // Wert : 1-8
	EE_Parameter.I_Faktor = 32;
	EE_Parameter.AchsKopplung1 = 90;
	EE_Parameter.AchsKopplung2 = 80;
	EE_Parameter.CouplingYawCorrection = 1;
	EE_Parameter.GyroAccAbgleich = 16;        // 1/k;
	EE_Parameter.DynamicStability = 100;
	memcpy(EE_Parameter.Name, "Sport\0", 12);
	EE_Parameter.crc = RAM_Checksum((uint8_t*)(&EE_Parameter), sizeof(EE_Parameter)-1);
}


/***************************************************/
/*    Default Values for parameter set 2           */
/***************************************************/
void ParamSet_DefaultSet2(void) // normal
{
	CommonDefaults();
	EE_Parameter.Stick_P = 10;               // Wert : 1-20
	EE_Parameter.Stick_D = 16;               // Wert : 0-20
	EE_Parameter.Gier_P = 6;                 // Wert : 1-20
	EE_Parameter.Gyro_P = 90;                // Wert : 0-247
	EE_Parameter.Gyro_I = 120;               // Wert : 0-247
	EE_Parameter.Gyro_Gier_P = 90;           // Wert : 0-247
	EE_Parameter.Gyro_Gier_I = 120;          // Wert : 0-247
	EE_Parameter.Gyro_Stability = 6; 	  	  // Wert : 1-8
	EE_Parameter.I_Faktor = 32;
	EE_Parameter.AchsKopplung1 = 90;
	EE_Parameter.AchsKopplung2 = 80;
	EE_Parameter.CouplingYawCorrection = 60;
	EE_Parameter.GyroAccAbgleich = 32;        // 1/k
	EE_Parameter.DynamicStability = 75;
	memcpy(EE_Parameter.Name, "Normal\0", 12);
	EE_Parameter.crc = RAM_Checksum((uint8_t*)(&EE_Parameter), sizeof(EE_Parameter)-1);
}


/***************************************************/
/*    Default Values for parameter set 3           */
/***************************************************/
void ParamSet_DefaultSet3(void) // beginner
{
	CommonDefaults();
	EE_Parameter.Stick_P = 8;                // Wert : 1-20
	EE_Parameter.Stick_D = 16;               // Wert : 0-20
	EE_Parameter.Gier_P  = 6;                // Wert : 1-20
	EE_Parameter.Gyro_P = 100;               // Wert : 0-247
	EE_Parameter.Gyro_I = 120;               // Wert : 0-247
	EE_Parameter.Gyro_Gier_P = 100;          // Wert : 0-247
	EE_Parameter.Gyro_Gier_I = 120;          // Wert : 0-247
	EE_Parameter.Gyro_Stability = 6; 	  	  // Wert : 1-8
	EE_Parameter.I_Faktor = 16;
	EE_Parameter.AchsKopplung1 = 90;
	EE_Parameter.AchsKopplung2 = 80;
	EE_Parameter.CouplingYawCorrection = 70;
	EE_Parameter.GyroAccAbgleich = 32;        // 1/k
	EE_Parameter.DynamicStability = 70;
	memcpy(EE_Parameter.Name, "Beginner\0", 12);
	EE_Parameter.crc = RAM_Checksum((uint8_t*)(&EE_Parameter), sizeof(EE_Parameter)-1);
}

/***************************************************/
/*       Read Parameter from EEPROM as byte        */
/***************************************************/
uint8_t GetParamByte(uint16_t param_id)
{
	return eeprom_read_byte((uint8_t*)(EEPROM_ADR_PARAM_BEGIN + param_id));
}

/***************************************************/
/*       Write Parameter to EEPROM as byte         */
/***************************************************/
void SetParamByte(uint16_t param_id, uint8_t value)
{
	eeprom_write_byte((uint8_t*)(EEPROM_ADR_PARAM_BEGIN + param_id), value);
}

/***************************************************/
/*       Read Parameter from EEPROM as word        */
/***************************************************/
uint16_t GetParamWord(uint16_t param_id)
{
	return eeprom_read_word((uint16_t *)(EEPROM_ADR_PARAM_BEGIN + param_id));
}

/***************************************************/
/*       Write Parameter to EEPROM as word         */
/***************************************************/
void SetParamWord(uint16_t param_id, uint16_t value)
{
	eeprom_write_word((uint16_t*)(EEPROM_ADR_PARAM_BEGIN + param_id), value);
}

/***************************************************/
/*       Read Parameter Set from EEPROM            */
/***************************************************/
// number [1..5]
uint8_t ParamSet_ReadFromEEProm(uint8_t setnumber)
{
	uint8_t crc;
	uint16_t eeaddr;

	// range the setnumber
	if((1 > setnumber) || (setnumber > 5)) setnumber = 3;

	// calculate eeprom addr
	eeaddr = EEPROM_ADR_PARAMSET + PARAMSET_STRUCT_LEN * (setnumber - 1);

	// calculate checksum from eeprom
	crc = EEProm_Checksum(eeaddr, PARAMSET_STRUCT_LEN - 1);

	// check crc
	if(crc != eeprom_read_byte((uint8_t*)(eeaddr + PARAMSET_STRUCT_LEN - 1))) return 0;

	// check revision
	if(eeprom_read_byte((uint8_t*)(eeaddr)) != EEPARAM_REVISION) return 0;

	// read paramset from eeprom
	eeprom_read_block((void *) &EE_Parameter, (void*)(EEPROM_ADR_PARAMSET + PARAMSET_STRUCT_LEN * (setnumber - 1)), PARAMSET_STRUCT_LEN);
	LED_Init();
	return 1;
}

/***************************************************/
/*        Write Parameter Set to EEPROM            */
/***************************************************/
// number [1..5]
uint8_t ParamSet_WriteToEEProm(uint8_t setnumber)
{
	uint8_t crc;

	if(EE_Parameter.Revision == EEPARAM_REVISION) // write only the right revision to eeprom
	{
		if(setnumber > 5) setnumber = 5;
		if(setnumber < 1) return 0;

		// update checksum
		EE_Parameter.crc = RAM_Checksum((uint8_t*)(&EE_Parameter), sizeof(EE_Parameter)-1);

		// write paramset to eeprom
		eeprom_write_block((void *) &EE_Parameter, (void*)(EEPROM_ADR_PARAMSET + PARAMSET_STRUCT_LEN * (setnumber - 1)), PARAMSET_STRUCT_LEN);

		// backup channel settings to separate block in eeprom
		eeprom_write_block( (void*)(EE_Parameter.Kanalbelegung), (void*)(EEPROM_ADR_CHANNELS), sizeof(EE_Parameter.Kanalbelegung));

		// write crc of channel block to eeprom
		crc = RAM_Checksum((uint8_t*)(EE_Parameter.Kanalbelegung), sizeof(EE_Parameter.Kanalbelegung));
		eeprom_write_byte((uint8_t*)(EEPROM_ADR_CHANNELS + sizeof(EE_Parameter.Kanalbelegung)), crc);

		// update active settings number
		SetActiveParamSet(setnumber);
		LED_Init();
		return 1;
	}
	// wrong revision
	return 0;
}

/***************************************************/
/*          Read MixerTable from EEPROM            */
/***************************************************/
uint8_t MixerTable_ReadFromEEProm(void)
{
	uint8_t crc;

	// calculate checksum in eeprom
	crc = EEProm_Checksum(EEPROM_ADR_MIXERTABLE, sizeof(Mixer) - 1);

	// check crc
	if( crc != eeprom_read_byte((uint8_t*)(EEPROM_ADR_MIXERTABLE + sizeof(Mixer) - 1)) ) return 0;

	// check revision
	if(eeprom_read_byte((uint8_t*)(EEPROM_ADR_MIXERTABLE)) != EEMIXER_REVISION) return 0;

	// read mixer table
	eeprom_read_block((void *) &Mixer, (void*)(EEPROM_ADR_MIXERTABLE), sizeof(Mixer));
	return 1;
}

/***************************************************/
/*          Write Mixer Table to EEPROM            */
/***************************************************/
uint8_t MixerTable_WriteToEEProm(void)
{
	if(Mixer.Revision == EEMIXER_REVISION)
	{
		// update crc
		Mixer.crc = RAM_Checksum((uint8_t*)(&Mixer), sizeof(Mixer) - 1);

		// write to eeprom
		eeprom_write_block((void *) &Mixer, (void*)(EEPROM_ADR_MIXERTABLE), sizeof(Mixer));
		return 1;
	}
	else return 0;
}

/***************************************************/
/*    Default Values for Mixer Table               */
/***************************************************/
void MixerTable_Default(void) // Quadro
{
	uint8_t i;

	Mixer.Revision = EEMIXER_REVISION;
	// clear mixer table
	for(i = 0; i < 16; i++)
	{
		Mixer.Motor[i][MIX_GAS]  = 0;
		Mixer.Motor[i][MIX_NICK] = 0;
		Mixer.Motor[i][MIX_ROLL] = 0;
		Mixer.Motor[i][MIX_YAW]  = 0;
	}
	// default = Quadro
	Mixer.Motor[0][MIX_GAS] = 64; Mixer.Motor[0][MIX_NICK] = +64; Mixer.Motor[0][MIX_ROLL] =   0; Mixer.Motor[0][MIX_YAW] = +64;
	Mixer.Motor[1][MIX_GAS] = 64; Mixer.Motor[1][MIX_NICK] = -64; Mixer.Motor[1][MIX_ROLL] =   0; Mixer.Motor[1][MIX_YAW] = +64;
	Mixer.Motor[2][MIX_GAS] = 64; Mixer.Motor[2][MIX_NICK] =   0; Mixer.Motor[2][MIX_ROLL] = -64; Mixer.Motor[2][MIX_YAW] = -64;
	Mixer.Motor[3][MIX_GAS] = 64; Mixer.Motor[3][MIX_NICK] =   0; Mixer.Motor[3][MIX_ROLL] = +64; Mixer.Motor[3][MIX_YAW] = -64;
	memcpy(Mixer.Name, "Quadro\0", 7);
	Mixer.crc = Mixer.crc = RAM_Checksum((uint8_t*)(&Mixer), sizeof(Mixer) - 1);
}

/***************************************************/
/*       Get active parameter set                  */
/***************************************************/
uint8_t GetActiveParamSet(void)
{
	uint8_t setnumber;
	setnumber = eeprom_read_byte((uint8_t*)(EEPROM_ADR_PARAM_BEGIN + PID_ACTIVE_SET));
	if(setnumber > 5)
	{
		setnumber = 3;
		eeprom_write_byte((void*)(EEPROM_ADR_PARAM_BEGIN+PID_ACTIVE_SET), setnumber);
	}
	return(setnumber);
}

/***************************************************/
/*       Set active parameter set                  */
/***************************************************/
void SetActiveParamSet(uint8_t setnumber)
{
	if(setnumber > 5) setnumber = 5;
	if(setnumber < 1) setnumber = 1;
	eeprom_write_byte((uint8_t*)(EEPROM_ADR_PARAM_BEGIN + PID_ACTIVE_SET), setnumber);
}

/***************************************************/
/*       Set default parameter set                 */
/***************************************************/
void SetDefaultParameter(uint8_t set, uint8_t restore_channels)
{

	if(set > 5) set = 5;
	else if(set < 1) set = 1;

	switch(set)
	{
		case 1:
			ParamSet_DefaultSet1(); // Fill ParamSet Structure to default parameter set 1 (Sport)
			break;
		case 2:
			ParamSet_DefaultSet2(); // Kamera
			break;
		case 3:
			ParamSet_DefaultSet3(); // Beginner
			break;
		default:
			ParamSet_DefaultSet3(); // Beginner
			break;
	}
	if(restore_channels)
	{
		uint8_t crc;
		// 1st check for a valid channel backup in eeprom
		crc = EEProm_Checksum(EEPROM_ADR_CHANNELS, sizeof(EE_Parameter.Kanalbelegung));
		if(crc == eeprom_read_byte((uint8_t*)(EEPROM_ADR_CHANNELS + sizeof(EE_Parameter.Kanalbelegung))) )
		{
			eeprom_read_block((void *)EE_Parameter.Kanalbelegung, (void*)(EEPROM_ADR_CHANNELS), sizeof(EE_Parameter.Kanalbelegung));
		}
		else ParamSet_DefaultStickMapping();
	}
	else ParamSet_DefaultStickMapping();
	ParamSet_WriteToEEProm(set);
}

/***************************************************/
/*       Initialize EEPROM Parameter Sets          */
/***************************************************/
void ParamSet_Init(void)
{
	uint8_t channel_backup  = 0, bad_params = 0, ee_default = 0,i;


	if(EEPARAM_REVISION != GetParamByte(PID_EE_REVISION) )
	{
		ee_default = 1; // software update or forced by mktool
		SetParamByte(PID_EE_REVISION, EEPARAM_REVISION);
	}


	// 1st check for a valid channel backup in eeprom
	i = EEProm_Checksum(EEPROM_ADR_CHANNELS, sizeof(EE_Parameter.Kanalbelegung));
	if(i == eeprom_read_byte((uint8_t*)(EEPROM_ADR_CHANNELS + sizeof(EE_Parameter.Kanalbelegung))) ) channel_backup = 1;


	// parameter check

	// check all 5 parameter settings
	for (i = 1;i < 6; i++)
	{
		if(ee_default || !ParamSet_ReadFromEEProm(i)) // could not read paramset from eeprom
		{
			bad_params = 1;
			printf("\n\rGenerating default Parameter Set %d",i);
			switch(i)
			{
				case 1:
					ParamSet_DefaultSet1(); // Fill ParamSet Structure to default parameter set 1 (Sport)
					break;
				case 2:
					ParamSet_DefaultSet2(); // Kamera
					break;
				case 3:
					ParamSet_DefaultSet3(); // Beginner
					break;
				default:
					ParamSet_DefaultSet3(); // Kamera
					break;
			}
			if(channel_backup) // if we have an channel mapping backup in eeprom
			{	// restore it from eeprom
				eeprom_read_block((void *)EE_Parameter.Kanalbelegung, (void*)(EEPROM_ADR_CHANNELS), sizeof(EE_Parameter.Kanalbelegung));
			}
			else
			{	// use default mapping
				ParamSet_DefaultStickMapping();
			}
			ParamSet_WriteToEEProm(i);
		}
	}
	if(bad_params) // at least one of the parameter settings were invalid
	{
		// default-Setting is parameter set 3
		SetActiveParamSet(3);
	}


	// read active parameter set to ParamSet stucture
	i = GetActiveParamSet();
	ParamSet_ReadFromEEProm(i);
	printf("\n\rUsing Parameter Set %d", i);

	// load mixer table
	if(ee_default || !MixerTable_ReadFromEEProm() )
	{
		printf("\n\rGenerating default Mixer Table");
		MixerTable_Default(); // Quadro
		MixerTable_WriteToEEProm();
	}
	// determine motornumber
	RequiredMotors = 0;
	for(i = 0; i < 16; i++)
	{
		if(Mixer.Motor[i][MIX_GAS] > 0) RequiredMotors++;
	}

	printf("\n\rMixer-Config: '%s' (%u Motors)",Mixer.Name, RequiredMotors);
	printf("\n\r==============================");
}
