 // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 // + Copyright (c) Holger Buss, Ingo Busker
 // + Thanks to Marcel Haller (Lion) for the nice idea and first implementation
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
#include "capacity.h"
#include "twimaster.h"
#include "main.h"
#include "timer0.h"
#include "analog.h"

#define CAPACITY_UPDATE_INTERVAL 10 // 10 ms
#define FC_OFFSET_CURRENT 5  // calculate with a current of 0.5A
#define BL_OFFSET_CURRENT 2  // calculate with a current of 0.2A

// global varialbles
unsigned short update_timer =  0;
Capacity_t Capacity;

// initialize capacity calculation
void Capacity_Init(void)
{
	Capacity.ActualCurrent = 0;
	Capacity.UsedCapacity = 0;
	Capacity.ActualPower = 0;
	Capacity.MinOfMaxPWM = 0;
	update_timer = SetDelay(CAPACITY_UPDATE_INTERVAL);
}


// called in main loop at a regular interval
void Capacity_Update(void)
{
	unsigned short Current, SetSum; // max value will be 255 * 12 = 3060
	static unsigned short SubCounter = 0;
	static unsigned short CurrentOffset = 0;
	static unsigned long SumCurrentOffset = 0;
	unsigned char i, NumOfMotors, MinOfMaxPWM;

	if(CheckDelay(update_timer))
	{
		update_timer += CAPACITY_UPDATE_INTERVAL; // do not use SetDelay to avoid timing leaks
		// determine sum of all present BL currents and setpoints
		Current = 0;
		SetSum = 0;
		NumOfMotors = 0;
		MinOfMaxPWM = 255;
		for(i = 0; i < MAX_MOTORS; i++)
		{
			if(Motor[i].State & MOTOR_STATE_PRESENT_MASK)
			{
				NumOfMotors++;
				Current += (unsigned int)(Motor[i].Current);
				SetSum +=  (unsigned int)(Motor[i].SetPoint);
				if(Motor[i].MaxPWM < MinOfMaxPWM) MinOfMaxPWM = Motor[i].MaxPWM;
			}
		}
		Capacity.MinOfMaxPWM = MinOfMaxPWM;

		if(SetSum == 0) // if all setpoints are 0
		{ // determine offsets of motor currents
			#define CURRENT_AVERAGE 8  // 8bit = 256 * 10 ms = 2.56s average time
			CurrentOffset = (unsigned int)(SumCurrentOffset>>CURRENT_AVERAGE);
			SumCurrentOffset -= CurrentOffset;
			SumCurrentOffset += Current;
			// after averaging set current to static offset
			Current = FC_OFFSET_CURRENT;
		}
		else // some motors are running, includes also motor test condition, where "MotorRunning" is false
		{   // subtract offset
			if(Current > CurrentOffset) Current -= CurrentOffset;
			else Current = 0;
			// add the FC and BL Offsets
			Current += FC_OFFSET_CURRENT + NumOfMotors * BL_OFFSET_CURRENT;
		}

		// update actual Current
		Capacity.ActualCurrent = Current;
		// update actual Power
		if(Current < 255)	Capacity.ActualPower = (UBat * Current) / 100; // in W higher resolution
		else				Capacity.ActualPower = (UBat * (Current/4)) / 25; // in W

		// update used capacity
		SubCounter += Current;

		// 100mA * 1ms * CAPACITY_UPDATE_INTERVAL = 1 mA * 100 ms * CAPACITY_UPDATE_INTERVAL
		// = 1mA * 0.1s * CAPACITY_UPDATE_INTERVAL = 1mA * 1min / (600 / CAPACITY_UPDATE_INTERVAL)
		// = 1mAh / (36000 / CAPACITY_UPDATE_INTERVAL)
		#define SUB_COUNTER_LIMIT (36000 / CAPACITY_UPDATE_INTERVAL)
		if(SubCounter > SUB_COUNTER_LIMIT)
		{
			Capacity.UsedCapacity++;			// we have one mAh more
			SubCounter -= SUB_COUNTER_LIMIT;	// keep the remaining sub part
		}
	} // EOF check delay update timer
}
