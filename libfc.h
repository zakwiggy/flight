#ifndef _LIBFC_H
#define _LIBFC_H


#define CPU_UNKNOWN 	0
#define CPU_ATMEGA644	1
#define CPU_ATMEGA644P	2
#define CPU_ATMEGA1284	3
#define CPU_ATMEGA1284P	4

extern void LIBFC_Init(void);
extern void LIBFC_Polling(void);
extern void LIBFC_ReceiverInit(unsigned char rtype);

extern void LIBFC_JetiBox_Putchar(char c);
extern void LIBFC_JetiBox_SetPos(unsigned char index);
extern void LIBFC_JetiBox_Clear(void);
extern unsigned char LIBFC_GetCPUType(void);

#endif //_LIBFC_H
