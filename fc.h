/*#######################################################################################
Flight Control
#######################################################################################*/

#ifndef _FC_H
#define _FC_H
//#define GIER_GRAD_FAKTOR 1291L // Abhängigkeit zwischen GyroIntegral und Winkel
//#define GIER_GRAD_FAKTOR 1160L
extern long GIER_GRAD_FAKTOR; // Abhängigkeit zwischen GyroIntegral und Winkel
#define STICK_GAIN 4
#define ACC_AMPLIFY    6

// FC STATUS FLAGS
#define FC_STATUS_MOTOR_RUN  				0x01
#define FC_STATUS_FLY        				0x02
#define FC_STATUS_CALIBRATE  				0x04
#define FC_STATUS_START      				0x08
#define FC_STATUS_EMERGENCY_LANDING      	0x10
#define FC_STATUS_LOWBAT		      		0x20
#define FC_STATUS_VARIO_TRIM_UP      		0x40
#define FC_STATUS_VARIO_TRIM_DOWN      		0x80

// FC STATUS FLAGS2
#define FC_STATUS2_CAREFREE                 0x01
#define FC_STATUS2_ALTITUDE_CONTROL         0x02

extern volatile unsigned char FC_StatusFlags, FC_StatusFlags2;
extern void ParameterZuordnung(void);

#define Poti1 Poti[0]
#define Poti2 Poti[1]
#define Poti3 Poti[2]
#define Poti4 Poti[3]
#define Poti5 Poti[4]
#define Poti6 Poti[5]
#define Poti7 Poti[6]
#define Poti8 Poti[7]

extern unsigned char Sekunde,Minute;
extern unsigned int BaroExpandActive;
extern long IntegralNick,IntegralNick2;
extern long IntegralRoll,IntegralRoll2;
//extern int IntegralNick,IntegralNick2;
//extern int IntegralRoll,IntegralRoll2;
extern unsigned char Poti[9];

extern long Mess_IntegralNick,Mess_IntegralNick2;
extern long Mess_IntegralRoll,Mess_IntegralRoll2;
extern long IntegralAccNick,IntegralAccRoll;
extern long SummeNick,SummeRoll;
extern volatile long Mess_Integral_Hoch;
extern long Integral_Gier,Mess_Integral_Gier,Mess_Integral_Gier2;
extern int  KompassValue;
extern int  KompassSollWert;
extern int  KompassRichtung;
extern char CalculateCompassTimer;
extern unsigned char KompassFusion;
extern unsigned char ControlHeading;
extern int  TrimNick, TrimRoll;
extern long  ErsatzKompass;
extern int   ErsatzKompassInGrad; // Kompasswert in Grad
extern long HoehenWert;
extern long SollHoehe;
extern long FromNC_AltitudeSetpoint;
extern unsigned char FromNC_AltitudeSpeed;

extern unsigned char CareFree;
extern int MesswertNick,MesswertRoll,MesswertGier;
extern int AdNeutralNick,AdNeutralRoll,AdNeutralGier, Mittelwert_AccNick, Mittelwert_AccRoll;
extern unsigned int NeutralAccX, NeutralAccY;
extern unsigned char HoehenReglerAktiv;
extern int NeutralAccZ;
extern long Umschlag180Nick, Umschlag180Roll;
extern signed int ExternStickNick,ExternStickRoll,ExternStickGier;
extern unsigned char Parameter_UserParam1,Parameter_UserParam2,Parameter_UserParam3,Parameter_UserParam4,Parameter_UserParam5,Parameter_UserParam6,Parameter_UserParam7,Parameter_UserParam8;
extern int NaviAccNick,NaviAccRoll,NaviCntAcc;
extern unsigned int modell_fliegt;
extern void MotorRegler(void);
extern void SendMotorData(void);
//void CalibrierMittelwert(void);
//void Mittelwert(void);
extern void SetNeutral(unsigned char AccAdjustment);
extern void Piep(unsigned char Anzahl, unsigned int dauer);
extern void CopyDebugValues(void);

extern unsigned char h,m,s;
extern volatile unsigned char Timeout ;
extern unsigned char CosinusNickWinkel, CosinusRollWinkel;
extern int  DiffNick,DiffRoll;
//extern int  Poti1, Poti2, Poti3, Poti4;
extern volatile unsigned char SenderOkay;
extern int StickNick,StickRoll,StickGier;
extern char MotorenEin;
extern unsigned char Parameter_Servo3,Parameter_Servo4,Parameter_Servo5;
extern char VarioCharacter;
extern int HoverGas;

extern unsigned char Parameter_Luftdruck_D;
extern unsigned char Parameter_MaxHoehe;
extern unsigned char Parameter_Hoehe_P;
extern unsigned char Parameter_Hoehe_ACC_Wirkung;
extern unsigned char Parameter_KompassWirkung;
extern unsigned char Parameter_Gyro_P;
extern unsigned char Parameter_Gyro_I;
extern unsigned char Parameter_Gier_P;
extern unsigned char Parameter_ServoNickControl;
extern unsigned char Parameter_ServoRollControl;
extern unsigned char Parameter_AchsKopplung1;
extern unsigned char Parameter_AchsKopplung2;
//extern unsigned char Parameter_AchsGegenKopplung1;
extern unsigned char Parameter_J16Bitmask;             // for the J16 Output
extern unsigned char Parameter_J16Timing;              // for the J16 Output
extern unsigned char Parameter_J17Bitmask;             // for the J17 Output
extern unsigned char Parameter_J17Timing;              // for the J17 Output
extern signed char MixerTable[MAX_MOTORS][4];
extern const signed char sintab[31];
#endif //_FC_H

