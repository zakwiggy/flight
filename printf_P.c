// Die Funktion printf_P() unterliegt ihrer eigenen Lizenz und ist nicht von der Lizenz für den MikroKopter-Teil unterstellt

/* 
Copyright (C) 1993 Free Software Foundation

This file is part of the GNU IO Library.  This library is free
software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the
Free Software Foundation; either version 2, or (at your option)
any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this library; see the file COPYING.  If not, write to the Free
Software Foundation, 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

As a special exception, if you link this library with files
compiled with a GNU compiler to produce an executable, this does not cause
the resulting executable to be covered by the GNU General Public License.
This exception does not however invalidate any other reasons why
the executable file might be covered by the GNU General Public License. */

/*
 * Copyright (c) 1990 Regents of the University of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. [rescinded 22 July 1999]
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/******************************************************************************
 This file is a patched version of printf called _printf_P
 It is made to work with avr-gcc for Atmel AVR MCUs.
 There are some differences from standard printf:
 	1. There is no floating point support (with fp the code is about 8K!)
 	2. Return type is void
 	3. Format string must be in program memory (by using macro printf this is
 	   done automaticaly)
 	4. %n is not implemented (just remove the comment around it if you need it)
 	5. If LIGHTPRINTF is defined, the code is about 550 bytes smaller and the
 	   folowing specifiers are disabled :
 		space # * . - + p s o O
	6. A function void uart_sendchar(char c) is used for output. The UART must
		be initialized before using printf.

 Alexander Popov
 sasho@vip.orbitel.bg
******************************************************************************/

/*
 * Actual printf innards.
 *
 * This code is large and complicated...
 */

#include <string.h>
#ifdef __STDC__
#include <stdarg.h>
#else
#include <varargs.h>
#endif

#include "main.h"


//#define LIGHTPRINTF
char PrintZiel;


char Putchar(char zeichen)
{
}


void PRINT(const char * ptr, unsigned int len) 
{
}
  
void PRINTP(const char * ptr, unsigned int len) 
{
 for(;len;len--) Putchar(pgm_read_byte(ptr++));
}

void PAD_SP(signed char howmany) 
{
	for(;howmany>0;howmany--) Putchar(' ');
}

void PAD_0(signed char howmany) 
{
	for(;howmany>0;howmany--) Putchar('0');
}

#define	BUF		40

/*
 * Macros for converting digits to letters and vice versa
 */
#define	to_digit(c)	((c) - '0')
#define  is_digit(c)	((c)<='9' && (c)>='0')
#define	to_char(n)	((n) + '0')

/*
 * Flags used during conversion.
 */
#define	LONGINT		0x01		/* long integer */
#define	LONGDBL		0x02		/* long double; unimplemented */
#define	SHORTINT		0x04		/* short integer */
#define	ALT			0x08		/* alternate form */
#define	LADJUST		0x10		/* left adjustment */
#define	ZEROPAD		0x20		/* zero (as opposed to blank) pad */
#define	HEXPREFIX	0x40		/* add 0x or 0X prefix */

void _printf_P (char ziel,char const *fmt0, ...)      /* Works with string from FLASH */
{
}
