#include "main.h"
#define MULTIPLYER 4

volatile unsigned int CountMilliseconds = 0;
volatile static unsigned int tim_main;
volatile unsigned char UpdateMotor = 0;
volatile unsigned int cntKompass = 0;
volatile unsigned int beeptime = 0;
volatile unsigned char SendSPI = 0, ServoActive = 0, CalculateServoSignals = 1;
uint16_t RemainingPulse = 0;
volatile int16_t ServoNickOffset = (255 / 2) * MULTIPLYER * 16; // initial value near center positon
volatile int16_t ServoRollOffset = (255 / 2) * MULTIPLYER * 16; // initial value near center positon

unsigned int BeepMuster = 0xffff;

volatile int16_t	ServoNickValue = 0;
volatile int16_t	ServoRollValue = 0;


enum {
  STOP             = 0,
  CK               = 1,
  CK8              = 2,
  CK64             = 3,
  CK256            = 4,
  CK1024           = 5,
  T0_FALLING_EDGE  = 6,
  T0_RISING_EDGE   = 7
};


ISR(TIMER0_OVF_vect)    // 9,7kHz
{
   static unsigned char cnt_1ms = 1,cnt = 0, compass_active = 0;
   unsigned char pieper_ein = 0;
   if(SendSPI) SendSPI--;
   if(SpektrumTimer) SpektrumTimer--;
   if(!cnt--)
    {
     cnt = 9;
     CountMilliseconds++;
     cnt_1ms++;
     cnt_1ms %= 2;

     if(!cnt_1ms) UpdateMotor = 1;
	 if(!(PINC & 0x10)) compass_active = 1;

     if(beeptime)
        {
        if(beeptime > 10) beeptime -= 10; else beeptime = 0;
        if(beeptime & BeepMuster)
         {
          pieper_ein = 1;
         }
         else pieper_ein = 0;
        }
     else
      {
       pieper_ein = 0;
       BeepMuster = 0xffff;
      }
     if(pieper_ein)
        {
          if(PlatinenVersion == 10) PORTD |= (1<<2); // Speaker an PORTD.2
          else                      PORTC |= (1<<7); // Speaker an PORTC.7
        }
     else
        {
         if(PlatinenVersion == 10) PORTD &= ~(1<<2);
         else                      PORTC &= ~(1<<7);
        }
	}
 if(compass_active && !NaviDataOkay && EE_Parameter.GlobalConfig & CFG_KOMPASS_AKTIV)
 {
  if(PINC & 0x10)
   {
	if(++cntKompass > 1000) compass_active = 0;
   }
  else
   {
    if((cntKompass) && (cntKompass < 362))
    {
     cntKompass += cntKompass / 41;
     if(cntKompass > 10) KompassValue = cntKompass - 10; else KompassValue = 0;
     KompassRichtung = ((540 + KompassValue - KompassSollWert) % 360) - 180;
    }
    cntKompass = 0;
   }
 }
}


// -----------------------------------------------------------------------
unsigned int SetDelay (unsigned int t)
{
//  TIMSK0 &= ~_BV(TOIE0);
  return(CountMilliseconds + t + 1);
//  TIMSK0 |= _BV(TOIE0);
}

// -----------------------------------------------------------------------
char CheckDelay(unsigned int t)
{
//  TIMSK0 &= ~_BV(TOIE0);
  return(((t - CountMilliseconds) & 0x8000) >> 9);
//  TIMSK0 |= _BV(TOIE0);
}

// -----------------------------------------------------------------------
void Delay_ms(unsigned int w)
{
 unsigned int akt;
 akt = SetDelay(w);
 while (!CheckDelay(akt));
}

void Delay_ms_Mess(unsigned int w)
{
 unsigned int akt;
 akt = SetDelay(w);
 while (!CheckDelay(akt)) if(AdReady) {AdReady = 0; ANALOG_ON;}
}

/*****************************************************/
/*              Initialize Timer 2                   */
/*****************************************************/
// The timer 2 is used to generate the PWM at PD7 (J7)
// to control a camera servo for nick compensation.
void TIMER2_Init(void)
{
	uint8_t sreg = SREG;

	// disable all interrupts before reconfiguration
	cli();

	PORTD &= ~(1<<PORTD7); 	// set PD7 to low

	DDRC  |= (1<<DDC6);     // set PC6 as output (Reset for HEF4017)
    HEF4017R_ON;
	// Timer/Counter 2 Control Register A

	// Timer Mode is FastPWM with timer reload at OCR2A (Bits: WGM22 = 1, WGM21 = 1, WGM20 = 1)
    // PD7: Normal port operation, OC2A disconnected, (Bits: COM2A1 = 0, COM2A0 = 0)
    // PD6: Normal port operation, OC2B disconnected, (Bits: COM2B1 = 0, COM2B0 = 0)
	TCCR2A &= ~((1<<COM2A1)|(1<<COM2A0)|(1<<COM2B1)|(1<<COM2B0));
    TCCR2A |= (1<<WGM21)|(1<<WGM20);

    // Timer/Counter 2 Control Register B

	// Set clock divider for timer 2 to SYSKLOCK/32 = 20MHz / 32 = 625 kHz
	// The timer increments from 0x00 to 0xFF with an update rate of 625 kHz or 1.6 us
	// hence the timer overflow interrupt frequency is 625 kHz / 256 = 2.44 kHz or 0.4096 ms

    // divider 32 (Bits: CS022 = 0, CS21 = 1, CS20 = 1)
	TCCR2B &= ~((1<<FOC2A)|(1<<FOC2B)|(1<<CS22));
    TCCR2B |= (1<<CS21)|(1<<CS20)|(1<<WGM22);

	// Initialize the Timer/Counter 2 Register
    TCNT2 = 0;

	// Initialize the Output Compare Register A used for PWM generation on port PD7.
	OCR2A = 255;
	TCCR2A |= (1<<COM2A1); // set or clear at compare match depends on value of COM2A0

	// Timer/Counter 2 Interrupt Mask Register
	// Enable timer output compare match A Interrupt only
	TIMSK2 &= ~((1<<OCIE2B)|(1<<TOIE2));
	TIMSK2 |= (1<<OCIE2A);

    SREG = sreg;
}

//----------------------------
void Timer_Init(void)
{
    tim_main = SetDelay(10);
    TCCR0B = CK8;
    TCCR0A = (1<<COM0A1)|(1<<COM0B1)|3;//fast PWM
    OCR0A =  0;
    OCR0B = 180;
    TCNT0 = (unsigned char)-TIMER_RELOAD_VALUE;  // reload
    //OCR1  = 0x00;
    TIMSK0 |= _BV(TOIE0);
}


/*****************************************************/
/*              Control Servo Position               */
/*****************************************************/


void CalculateServo(void)
{
 signed char cosinus, sinus;
 signed long nick, roll;

	cosinus = sintab[EE_Parameter.CamOrientation + 6];
	sinus = sintab[EE_Parameter.CamOrientation];

  if(CalculateServoSignals == 1)
   {
		nick = (cosinus * IntegralNick) / 128L - (sinus * IntegralRoll) / 128L;
        nick -= POI_KameraNick * 7;
		nick = ((long)EE_Parameter.ServoNickComp * nick) / 512L;
		ServoNickOffset += ((int16_t)Parameter_ServoNickControl * (MULTIPLYER*16) - ServoNickOffset) / EE_Parameter.ServoManualControlSpeed;
		ServoNickValue = ServoNickOffset / 16; // offset (Range from 0 to 255 * 3 = 765)

		if(EE_Parameter.ServoCompInvert & 0x01)
		{	// inverting movement of servo
			ServoNickValue += nick;//(int16_t)( ( (int32_t)EE_Parameter.ServoNickComp * nick) / (256L) );
		}
		else
		{	// non inverting movement of servo
			ServoNickValue -= nick;
		}
		// limit servo value to its parameter range definition
		if(ServoNickValue < ((int16_t)EE_Parameter.ServoNickMin * MULTIPLYER) )
		{
			ServoNickValue = (int16_t)EE_Parameter.ServoNickMin * MULTIPLYER;
		}
		else
		if(ServoNickValue > ((int16_t)EE_Parameter.ServoNickMax * MULTIPLYER) )
		{
			ServoNickValue = (int16_t)EE_Parameter.ServoNickMax * MULTIPLYER;
		}
		if(PlatinenVersion < 20) CalculateServoSignals = 0; else CalculateServoSignals++;
	}
	else
	{
    	roll = (cosinus * IntegralRoll) / 128L + (sinus * IntegralNick) / 128L;
    	roll = ((long)EE_Parameter.ServoRollComp * roll) / 512L;
		ServoRollOffset += ((int16_t)Parameter_ServoRollControl * (MULTIPLYER*16) - ServoRollOffset) / EE_Parameter.ServoManualControlSpeed;
		ServoRollValue = ServoRollOffset/16; // offset (Range from 0 to 255 * 3 = 765)
		if(EE_Parameter.ServoCompInvert & 0x02)
		{	// inverting movement of servo
			ServoRollValue += roll;
		}
		else
		{	// non inverting movement of servo
			ServoRollValue -= roll;
		}
		// limit servo value to its parameter range definition
		if(ServoRollValue < ((int16_t)EE_Parameter.ServoRollMin * MULTIPLYER) )
		{
			ServoRollValue = (int16_t)EE_Parameter.ServoRollMin * MULTIPLYER;
		}
		else
		if(ServoRollValue > ((int16_t)EE_Parameter.ServoRollMax * MULTIPLYER) )
		{
			ServoRollValue = (int16_t)EE_Parameter.ServoRollMax * MULTIPLYER;
		}
		CalculateServoSignals = 0;
	}
}

ISR(TIMER2_COMPA_vect)
{
	// frame len 22.5 ms = 14063 * 1.6 us
	// stop pulse: 0.3 ms = 188 * 1.6 us
	// min servo pulse: 0.6 ms =  375 * 1.6 us
	// max servo pulse: 2.4 ms = 1500 * 1.6 us
	// resolution: 1500 - 375 = 1125 steps

	#define IRS_RUNTIME 127
	#define PPM_STOPPULSE 188
    #define PPM_FRAMELEN (1757 * EE_Parameter.ServoNickRefresh)
	#define MINSERVOPULSE 375
	#define MAXSERVOPULSE 1500
	#define SERVORANGE (MAXSERVOPULSE - MINSERVOPULSE)

	static uint8_t  PulseOutput = 0;
	static uint16_t ServoFrameTime = 0;
	static uint8_t  ServoIndex = 0;


	if(PlatinenVersion < 20)
	{
		//---------------------------
		// Nick servo state machine
		//---------------------------
		if(!PulseOutput) // pulse output complete
		{
			if(TCCR2A & (1<<COM2A0)) // we had a low pulse
			{
				TCCR2A &= ~(1<<COM2A0);// make a high pulse
				RemainingPulse  = MINSERVOPULSE + SERVORANGE/2; // center position ~ 1.5ms
				RemainingPulse += ServoNickValue - (256 / 2) * MULTIPLYER; // shift ServoNickValue to center position
				// range servo pulse width
				if(RemainingPulse > MAXSERVOPULSE )			RemainingPulse = MAXSERVOPULSE; // upper servo pulse limit
				else if(RemainingPulse < MINSERVOPULSE )	RemainingPulse = MINSERVOPULSE; // lower servo pulse limit
				// accumulate time for correct update rate
				ServoFrameTime = RemainingPulse;
			}
			else // we had a high pulse
			{
				TCCR2A |= (1<<COM2A0); // make a low pulse
				RemainingPulse = PPM_FRAMELEN - ServoFrameTime;
				CalculateServoSignals = 1;
			}
			// set pulse output active
			PulseOutput = 1;
		}
	} // EOF Nick servo state machine
	else
	{
		//-----------------------------------------------------
		// PPM state machine, onboard demultiplexed by HEF4017
		//-----------------------------------------------------
		if(!PulseOutput) // pulse output complete
		{
			if(TCCR2A & (1<<COM2A0)) // we had a low pulse
			{
				TCCR2A &= ~(1<<COM2A0);// make a high pulse

				if(ServoIndex == 0) // if we are at the sync gap
				{
					RemainingPulse = PPM_FRAMELEN - ServoFrameTime; // generate sync gap by filling time to full frame time
					ServoFrameTime = 0; // reset servo frame time
					HEF4017R_ON; // enable HEF4017 reset
				}
				else // servo channels
				{
					RemainingPulse  = MINSERVOPULSE + SERVORANGE/2; // center position ~ 1.5ms
					switch(ServoIndex) // map servo channels
					{
						case 1: // Nick Compensation Servo
							RemainingPulse += ServoNickValue - (256 / 2) * MULTIPLYER; // shift ServoNickValue to center position
							break;
 					 case 2: // Roll Compensation Servo
							RemainingPulse += ServoRollValue - (256 / 2) * MULTIPLYER; // shift ServoNickValue to center position
							break;
					 case 3:
					 		RemainingPulse += ((int16_t)Parameter_Servo3 * MULTIPLYER) - (256 / 2) * MULTIPLYER;
							break;
					 case 4:
					 		RemainingPulse += ((int16_t)Parameter_Servo4 * MULTIPLYER) - (256 / 2) * MULTIPLYER;
							break;
					 case 5:
					 		RemainingPulse += ((int16_t)Parameter_Servo5 * MULTIPLYER) - (256 / 2) * MULTIPLYER;
							break;
						default: // other servo channels
							RemainingPulse += 2 * PPM_in[ServoIndex]; // add channel value, factor of 2 because timer 1 increments 3.2µs
							break;
					}
					// range servo pulse width
					if(RemainingPulse > MAXSERVOPULSE )			RemainingPulse = MAXSERVOPULSE; // upper servo pulse limit
					else if(RemainingPulse < MINSERVOPULSE )	RemainingPulse = MINSERVOPULSE; // lower servo pulse limit
					// substract stop pulse width
					RemainingPulse -= PPM_STOPPULSE;
					// accumulate time for correct sync gap
					ServoFrameTime += RemainingPulse;
				}
			}
			else // we had a high pulse
			{
				TCCR2A |= (1<<COM2A0); // make a low pulse
				// set pulsewidth to stop pulse width
				RemainingPulse = PPM_STOPPULSE;
				// accumulate time for correct sync gap
				ServoFrameTime += RemainingPulse;
				if((ServoActive && SenderOkay > 50) || ServoActive == 2) HEF4017R_OFF; // disable HEF4017 reset
				else HEF4017R_ON;
				ServoIndex++; // change to next servo channel
				if(ServoIndex > EE_Parameter.ServoNickRefresh)
				  {
				    CalculateServoSignals = 1;
					ServoIndex = 0; // reset to the sync gap
				  }
			}
			// set pulse output active
			PulseOutput = 1;
		}
	} // EOF PPM state machine

	// General pulse output generator
	if(RemainingPulse > (255 + IRS_RUNTIME))
	{
		OCR2A = 255;
		RemainingPulse -= 255;
	}
	else
	{
		if(RemainingPulse > 255) // this is the 2nd last part
		{
			if((RemainingPulse - 255) < IRS_RUNTIME)
			{
				OCR2A = 255 - IRS_RUNTIME;
				RemainingPulse -= 255 - IRS_RUNTIME;

			}
			else // last part > ISR_RUNTIME
			{
				OCR2A = 255;
				RemainingPulse -= 255;
			}
		}
		else // this is the last part
		{
			OCR2A = RemainingPulse;
			RemainingPulse = 0;
			PulseOutput = 0; // trigger to stop pulse
		}
	} // EOF general pulse output generator
}
