// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) Holger Buss, Ingo Busker
// + Nur für den privaten Gebrauch
// + www.MikroKopter.com
// + porting the sources to other systems or using the software on other systems (except hardware from www.mikrokopter.de) is not allowed
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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include "eeprom.h"
#include "twimaster.h"
#include "fc.h"
#include "analog.h"
#include "uart.h"
#include "timer0.h"

volatile uint8_t twi_state	= TWI_STATE_MOTOR_TX;
volatile uint8_t dac_channel 	= 0;
volatile uint8_t motor_write 	= 0;
volatile uint8_t motor_read  	= 0;
volatile uint8_t I2C_TransferActive = 0;

volatile uint16_t I2CTimeout = 100;

uint8_t MissingMotor  = 0;

volatile uint8_t BLFlags = 0;

MotorData_t Motor[MAX_MOTORS];

// bit mask for witch BL the configuration should be sent
volatile uint16_t BLConfig_WriteMask = 0;
// bit mask for witch BL the configuration should be read
volatile uint16_t BLConfig_ReadMask = 0;
// buffer for BL Configuration
BLConfig_t BLConfig;

#define I2C_WriteByte(byte) {TWDR = byte; TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);}
#define I2C_ReceiveByte() {TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE) | (1<<TWEA);}
#define I2C_ReceiveLastByte() {TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);}

#define SCL_CLOCK  200000L
#define I2C_TIMEOUT 30000
#define TWI_BASE_ADDRESS 0x52

/**************************************************/
/*   Initialize I2C (TWI)                         */
/**************************************************/

void I2C_Init(char clear)
{
	uint8_t i;
	uint8_t sreg = SREG;
	cli();

	// SDA is INPUT
	DDRC  &= ~(1<<DDC1);
	// SCL is output
	DDRC |= (1<<DDC0);
	// pull up SDA
	PORTC |= (1<<PORTC0)|(1<<PORTC1);

	// TWI Status Register
	// prescaler 1 (TWPS1 = 0, TWPS0 = 0)
	TWSR &= ~((1<<TWPS1)|(1<<TWPS0));

	// set TWI Bit Rate Register
	TWBR = ((F_CPU/SCL_CLOCK)-16)/2;

	twi_state 		= TWI_STATE_MOTOR_TX;
	motor_write 	= 0;
	motor_read 		= 0;

	if(clear) for(i=0; i < MAX_MOTORS; i++)
	{
		Motor[i].Version	= 0;
		Motor[i].SetPoint	= 0;
		Motor[i].SetPointLowerBits	= 0;
		Motor[i].State		= 0;
		Motor[i].ReadMode	= BL_READMODE_STATUS;
		Motor[i].Current	= 0;
		Motor[i].MaxPWM		= 0;
		Motor[i].Temperature = 0;
	}
    sei();
	SREG = sreg;
}

void I2C_Reset(void)
{
	// stop i2c bus
	I2C_Stop(TWI_STATE_MOTOR_TX);
	TWCR = (1<<TWINT); // reset to original state incl. interrupt flag reset
	TWAMR = 0;
	TWAR = 0;
	TWDR = 0;
	TWSR = 0;
	TWBR = 0;
    I2C_TransferActive = 0;
	I2C_Init(0);
	I2C_WriteByte(0);
	BLFlags |= BLFLAG_READ_VERSION;
}

/****************************************/
/*        I2C ISR                       */
/****************************************/
ISR (TWI_vect)
{
	static uint8_t missing_motor = 0, motor_read_temperature = 0;
	static uint8_t *pBuff = 0;
	static uint8_t BuffLen = 0;

    switch (twi_state++)
	{
		// Master Transmit
        case 0: // TWI_STATE_MOTOR_TX
            I2C_TransferActive = 1;
			// skip motor if not used in mixer
			while((Mixer.Motor[motor_write][MIX_GAS] <= 0) && (motor_write < MAX_MOTORS)) motor_write++;
			if(motor_write >= MAX_MOTORS) // writing finished, read now
			{
				BLConfig_WriteMask = 0; // reset configuration bitmask
				motor_write = 0; // reset motor write counter for next cycle
				twi_state = TWI_STATE_MOTOR_RX;
				I2C_WriteByte(TWI_BASE_ADDRESS + TW_READ + (motor_read<<1) ); // select slave address in rx mode
			}
			else I2C_WriteByte(TWI_BASE_ADDRESS + TW_WRITE + (motor_write<<1) ); // select slave address in tx mode
			break;
        case 1: // Send Data to Slave
			I2C_WriteByte(Motor[motor_write].SetPoint); // transmit setpoint
			// if old version has been detected
			if(!(Motor[motor_write].Version & MOTOR_STATE_NEW_PROTOCOL_MASK))
			{
				twi_state = 4; //jump over sending more data
			}
			// the new version has been detected
			else if(!( (Motor[motor_write].SetPointLowerBits && (RequiredMotors < 7)) || BLConfig_WriteMask || BLConfig_ReadMask )  )
			{	// or LowerBits are zero and no BlConfig should be sent (saves round trip time)
				twi_state = 4; //jump over sending more data
			}
			break;
       	case 2: // lower bits of setpoint (higher resolution)
			if ((0x0001<<motor_write) & BLConfig_ReadMask)
			{
				Motor[motor_write].ReadMode = BL_READMODE_CONFIG; // configuration request
			}
			else
			{
				Motor[motor_write].ReadMode = BL_READMODE_STATUS; // normal status request
			}
			// send read mode and the lower bits of setpoint
       		I2C_WriteByte((Motor[motor_write].ReadMode<<3)|(Motor[motor_write].SetPointLowerBits & 0x07));
			// configuration tranmission request?
			if((0x0001<<motor_write) & BLConfig_WriteMask)
			{	// redirect tx pointer to configuration data
				pBuff = (uint8_t*)&BLConfig; // select config for motor
				BuffLen = sizeof(BLConfig_t);
			}
			else
			{	// jump to end of transmission for that motor
				twi_state = 4;
			}
			break;
		case 3: // send configuration
			I2C_WriteByte(*pBuff);
			pBuff++;
			if(--BuffLen > 0) twi_state = 3; // if there are some bytes left
			break;
        case 4: // repeat case 0-4 for all motors
			if(TWSR == TW_MT_DATA_NACK) // Data transmitted, NACK received
			{
				if(!missing_motor) missing_motor = motor_write + 1;
				if((Motor[motor_write].State & MOTOR_STATE_ERROR_MASK) < MOTOR_STATE_ERROR_MASK) Motor[motor_write].State++; // increment error counter and handle overflow
			}
			I2C_Stop(TWI_STATE_MOTOR_TX);
			I2CTimeout = 10;
			motor_write++; // next motor
			I2C_Start(TWI_STATE_MOTOR_TX); // Repeated start -> switch slave or switch Master Transmit -> Master Receive
			break;
       // Master Receive Data
        case 5: // TWI_STATE_MOTOR_RX
			if(TWSR != TW_MR_SLA_ACK) //  SLA+R transmitted but no ACK received
			{	// no response from the addressed slave received
				Motor[motor_read].State &= ~MOTOR_STATE_PRESENT_MASK; // clear present bit
				if(++motor_read >= MAX_MOTORS)
				{	// all motors read
					motor_read = 0;			// restart from beginning
					BLConfig_ReadMask = 0; 	// reset read configuration bitmask
					if(++motor_read_temperature >= MAX_MOTORS)
					{
						motor_read_temperature = 0;
						BLFlags &= ~BLFLAG_READ_VERSION;
					}
				}
				BLFlags |= BLFLAG_TX_COMPLETE;
				I2C_Stop(TWI_STATE_MOTOR_TX);
				I2C_TransferActive = 0;
			}
			else
			{	// motor successfully addressed
				Motor[motor_read].State |= MOTOR_STATE_PRESENT_MASK; // set present bit
				if(Motor[motor_read].Version & MOTOR_STATE_NEW_PROTOCOL_MASK)
				{
					// new BL found
					switch(Motor[motor_read].ReadMode)
					{
						case BL_READMODE_CONFIG:
							pBuff = (uint8_t*)&BLConfig;
							BuffLen = sizeof(BLConfig_t);
							break;

						case BL_READMODE_STATUS:
							pBuff = (uint8_t*)&(Motor[motor_read].Current);
							if(motor_read == motor_read_temperature) BuffLen = 3; // read Current, MaxPwm & Temp
							else BuffLen = 1;// read Current only
							break;
					}
				}
				else // old BL version
				{
					pBuff = (uint8_t*)&(Motor[motor_read].Current);
					if((BLFlags & BLFLAG_READ_VERSION) || (motor_read == motor_read_temperature)) BuffLen = 2; // Current & MaxPwm
					else BuffLen = 1; // read Current only
				}
				if(BuffLen == 1)
				{
					I2C_ReceiveLastByte(); 	// read last byte
				}
				else
				{
					I2C_ReceiveByte(); 		// read next byte
				}
			}
			MissingMotor = missing_motor;
			missing_motor = 0;
			break;
		case 6: // receive bytes
			*pBuff = TWDR;
			pBuff++;
			BuffLen--;
			if(BuffLen>1)
			{
				I2C_ReceiveByte(); // read next byte
			}
			else if (BuffLen == 1)
			{
				I2C_ReceiveLastByte(); 	// read last byte
			}
			else // nothing left
			{
				if(BLFlags & BLFLAG_READ_VERSION)
				{
					if(!(FC_StatusFlags & FC_STATUS_MOTOR_RUN) && (Motor[motor_read].MaxPWM == 250) ) Motor[motor_read].Version |= MOTOR_STATE_NEW_PROTOCOL_MASK;
					else Motor[motor_read].Version = 0;
				}
				if(++motor_read >= MAX_MOTORS)
				{
					motor_read = 0;			// restart from beginning
					BLConfig_ReadMask = 0;	// reset read configuration bitmask
					if(++motor_read_temperature >= MAX_MOTORS)
					{
						motor_read_temperature = 0;
						BLFlags &= ~BLFLAG_READ_VERSION;
					}
				}
				I2C_Stop(TWI_STATE_MOTOR_TX);
				BLFlags |= BLFLAG_TX_COMPLETE;
                I2C_TransferActive = 0;
				return;
			}
			twi_state = 6; // if there are some bytes left
			break;

		// writing Gyro-Offsets
		case 18:
			I2C_WriteByte(0x98); // Address the DAC
			break;

		case 19:
			I2C_WriteByte(0x10 + (dac_channel * 2)); // Select DAC Channel (0x10 = A, 0x12 = B, 0x14 = C)
			break;

		case 20:
			switch(dac_channel)
			{
				case 0:
						I2C_WriteByte(AnalogOffsetNick); // 1st byte for Channel A
						break;
				case 1:
						I2C_WriteByte(AnalogOffsetRoll); // 1st byte for Channel B
						break;
				case 2:
						I2C_WriteByte(AnalogOffsetGier); // 1st byte for Channel C
						break;
			}
			break;

		case 21:
			I2C_WriteByte(0x80); // 2nd byte for all channels is 0x80
			break;

		case 22:
			I2C_Stop(TWI_STATE_MOTOR_TX);
			I2C_TransferActive = 0;
			I2CTimeout = 10;
			// repeat case 18...22 until all DAC Channels are updated
			if(dac_channel < 2)
			{
				dac_channel ++; 	// jump to next channel
				I2C_Start(TWI_STATE_GYRO_OFFSET_TX); 		// start transmission for next channel
			}
			else
			{
				dac_channel = 0; // reset dac channel counter
				BLFlags |= BLFLAG_TX_COMPLETE;
			}
			break;
        default:
			I2C_Stop(TWI_STATE_MOTOR_TX);
			BLFlags |= BLFLAG_TX_COMPLETE;
			I2CTimeout = 10;
			motor_write = 0;
			motor_read = 0;
			I2C_TransferActive = 0;
			break;
	}

}


uint8_t I2C_WriteBLConfig(uint8_t motor)
{
	uint8_t i;
	uint16_t timer;

	if(MotorenEin || PC_MotortestActive) return(BLCONFIG_ERR_MOTOR_RUNNING); 	// not when motors are running!
	if(motor > MAX_MOTORS) return (BLCONFIG_ERR_MOTOR_NOT_EXIST); 			// motor does not exist!
	if(motor)
	{
		if(!(Motor[motor-1].State & MOTOR_STATE_PRESENT_MASK)) return(BLCONFIG_ERR_MOTOR_NOT_EXIST); // motor does not exist!
		if(!(Motor[motor-1].Version & MOTOR_STATE_NEW_PROTOCOL_MASK)) return(BLCONFIG_ERR_HW_NOT_COMPATIBLE); // not a new BL!
	}
	// check BL configuration to send
	if(BLConfig.Revision != BLCONFIG_REVISION) return (BLCONFIG_ERR_SW_NOT_COMPATIBLE); // bad revison
	i = RAM_Checksum((uint8_t*)&BLConfig, sizeof(BLConfig_t) - 1);
	if(i != BLConfig.crc) return(BLCONFIG_ERR_CHECKSUM); // bad checksum

	timer = SetDelay(2000);
	while(!(BLFlags & BLFLAG_TX_COMPLETE) && !CheckDelay(timer)); 	//wait for complete transfer

	// prepare the bitmask
	if(!motor) // 0 means all
	{
		BLConfig_WriteMask = 0xFF; // all motors at once with the same configuration
	}
	else //only one specific motor
	{
		BLConfig_WriteMask = 0x0001<<(motor-1);
	}
	for(i = 0; i < MAX_MOTORS; i++)
	{
		if((0x0001<<i) & BLConfig_WriteMask)
		{
			Motor[i].SetPoint = 0;
			Motor[i].SetPointLowerBits = 0;
		}
	}

	motor_write = 0;
	// needs at least MAX_MOTORS loops of 2 ms (12*2ms = 24ms)
	do
	{
		I2C_Start(TWI_STATE_MOTOR_TX); // start an i2c transmission
		while(!(BLFlags & BLFLAG_TX_COMPLETE)  && !CheckDelay(timer)); //wait for complete transfer
	}while(BLConfig_WriteMask  && !CheckDelay(timer)); // repeat until the BL config has been sent
	if(BLConfig_WriteMask) return(BLCONFIG_ERR_MOTOR_NOT_EXIST);
	return(BLCONFIG_SUCCESS);
}

uint8_t I2C_ReadBLConfig(uint8_t motor)
{
	uint8_t i;
	uint16_t timer;

	if(MotorenEin || PC_MotortestActive) return(BLCONFIG_ERR_MOTOR_RUNNING); // not when motors are running!
	if(motor > MAX_MOTORS) return (BLCONFIG_ERR_MOTOR_NOT_EXIST); 		// motor does not exist!
	if(motor == 0) return (BLCONFIG_ERR_READ_NOT_POSSIBLE);
	if(!(Motor[motor-1].State & MOTOR_STATE_PRESENT_MASK)) return(BLCONFIG_ERR_MOTOR_NOT_EXIST); // motor does not exist!
	if(!(Motor[motor-1].Version & MOTOR_STATE_NEW_PROTOCOL_MASK)) return(BLCONFIG_ERR_HW_NOT_COMPATIBLE); // not a new BL!

	timer = SetDelay(2000);
	while(!(BLFlags & BLFLAG_TX_COMPLETE) && !CheckDelay(timer)); 				//wait for complete transfer

	// prepare the bitmask
	BLConfig_ReadMask = 0x0001<<(motor-1);

	for(i = 0; i < MAX_MOTORS; i++)
	{
		if((0x0001<<i) & BLConfig_ReadMask)
		{
			Motor[i].SetPoint = 0;
			Motor[i].SetPointLowerBits = 0;
		}
	}

	motor_read = 0;
	BLConfig.Revision = 0; // bad revision
	BLConfig.crc = 0;	   // bad checksum
	// needs at least MAX_MOTORS loops of 2 ms (12*2ms = 24ms)
	do
	{
		I2C_Start(TWI_STATE_MOTOR_TX); // start an i2c transmission
		while(!(BLFlags & BLFLAG_TX_COMPLETE) && !CheckDelay(timer)); //wait for complete transfer
	}while(BLConfig_ReadMask && !CheckDelay(timer)); // repeat until the BL config has been received from all motors
	// validate result
	if(BLConfig.Revision != BLCONFIG_REVISION) return (BLCONFIG_ERR_SW_NOT_COMPATIBLE); // bad revison
	i = RAM_Checksum((uint8_t*)&BLConfig, sizeof(BLConfig_t) - 1);
	if(i != BLConfig.crc) return(BLCONFIG_ERR_CHECKSUM); // bad checksum
	return(BLCONFIG_SUCCESS);
}

