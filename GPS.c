// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Copyright (c) Holger Buss, Ingo Busker
// + only for non-profit use
// + www.MikroKopter.com
// + see the File "License.txt" for further Informations
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "main.h"

signed int GPS_Nick = 0;
signed int GPS_Roll = 0;
signed int GPS_Nick2 = 0;
signed int GPS_Roll2 = 0;
long GpsAktuell_X = 0;
long GpsAktuell_Y = 0;
long GpsZiel_X = 0;
long GpsZiel_Y = 0;
  
void GPS_Neutral(void)
{
 GpsZiel_X = GpsAktuell_X;
 GpsZiel_Y = GpsAktuell_Y;
}

void GPS_BerechneZielrichtung(void)
{
 GPS_Nick = 0;
 GPS_Roll = 0;
}




