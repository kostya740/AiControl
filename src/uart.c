/*	uart.c
*  v. 0.9
*
* PURPOSE
*	UART interrupt handler of Ai10 controller firmware.
*
* CONTACTS
*	E-mail regarding this program should be addressed to
*		proj.ai10@gmai.com
*
* COPYRIGHT
*	This file is distributed under the terms of the GNU General Public
*	License (GPL). Copies of the GPL can be obtained from:
*		ftp://prep.ai.mit.edu/pub/gnu/GPL
*	Each contributing author retains all rights to their own work.
*
*	(C) 2011 Nik1976 and Pavel Grodek
*
* HISTORY
*
*	Created Feb. 25Th 2011
*/

#include "hardware.h"
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "uart.h"
#include "ainetcommands.h"
#include <ctype.h>

volatile uint8_t outputen;
volatile uint8_t ack_en = 0;
volatile uint8_t filt_en = 0;
volatile uint8_t slave_en = 0;
volatile uint8_t proc_en = 1;


volatile uint8_t builtin_index;
volatile uint8_t vol_level = 0xff, bal_level, fad_level, vol_level_back=0xff;
;
volatile uint8_t vol_set = 0xff, ch_input = 0;

volatile uint8_t cdc_en = 0;

volatile int8_t vol_changed = 0;
volatile int8_t bal_changed = 0;
volatile int8_t fad_changed = 0;
volatile int8_t inp_changed = 0;
volatile int8_t bass_changed = 0;
volatile int8_t track_changed = 0;
volatile uint8_t disk_changed = 0;
volatile uint8_t preset_changed = 0;
volatile uint8_t acc_changed = 0;

volatile uint8_t c,havecommand,i,j, val;
volatile uint16_t temp;
volatile char cc;


unsigned int ainet_temp[11];
uint8_t vol[VOL_LEN]={0x99,0x78,0x68,0x60,0x55,0x50,0x48,0x46,0x44,0x42,
	0x40,0x38,0x36,0x34,0x32,0x30,0x28,0x26,0x24,0x22,
	0x20,0x18,0x16,0x14,0x12,0x10,0x09,0x08,0x07,0x06,
0x05,0x04,0x03,0x02,0x01,0x00};


#define RXBUFFERSIZE 250
unsigned char rxbuffer0[RXBUFFERSIZE];
unsigned char dummy2 ='\0';

uint8_t rxbufferfirst0,rxbufferlast0; //rxbufferlen0



void uart_init(void) //UART init routine
{
	UBRRL = 25; //38400

	rxbufferlast0=0; //end of the buffer pointer
	UCSRB=_BV(TXEN)|_BV(RXEN)|_BV(RXCIE); // tx/rx enabled, rx int enabled
	UCSRC=_BV(UCSZ1)|_BV(UCSZ0); //asynchronous, 8N1

	prints("init!\r\n");
}

uint8_t get_dec(uint8_t *input_str)
{ //returns first decimal value (0..254) found in ASCII input string or 0xff if no value found
	uint8_t i=0, j=1;
	uint16_t k;
	
	while (((input_str[i]<'0')||(input_str[i]>'9'))&&(input_str[i]!='\0')) i++;
	if ((input_str[i]>='0')&&(input_str[i]<='9'))
	{
		while ((input_str[i+j]>='0')&&(input_str[i+j]<='9')) j++;
		switch (j)
		{
			case 1:
			{
				return (input_str[i]-'0');
				break;
			}
			case 2:
			{
				k=(input_str[i]-'0')*10+(input_str[i+1]-'0');
				return (k);
				break;
			}
			case 3:
			{
				k=(input_str[i]-'0')*100+(input_str[i+1]-'0')*10+(input_str[i+2]-'0');
				if (k<255) return (k);
				else return (0xff);
				break;
			}
			default:
			{
				return (0xff);
				break;
			}
			
		}
		
	} else return (0xff);
}

void prints (char *input_str)
{ //sends string to UART
	uint8_t i=0;
	
	while (input_str[i]!='\0')
	{
		while (!(UCSRA &(1<<UDRE))) ;
		UDR = input_str[i++];
	}
}


void printd (uint8_t val)
{ //sends unsigned decimal value to UART

	if (val > 99)
	{
		while (!(UCSRA &(1<<UDRE))) ;
		UDR = (val/100 + '0');
		val = val % 100;
		while (!(UCSRA &(1<<UDRE))) ;
		UDR = (val/10 + '0');
		val = val % 10;
		while (!(UCSRA &(1<<UDRE))) ;
		UDR = (val + '0');
		} else if (val >9) {
		while (!(UCSRA &(1<<UDRE))) ;
		UDR = (val/10 + '0');
		val = val % 10;
		while (!(UCSRA &(1<<UDRE))) ;
		UDR = (val + '0');
		} else {
		while (!(UCSRA &(1<<UDRE))) ;
		UDR = (val + '0');
	}
	prints("\r\n");
}



void printh (uint8_t val)
{ //sends unsigned hex value to UART


	while (!(UCSRA &(1<<UDRE))) ;
	if (val > 0x9f) UDR = (val/0x10 - 0x0a + 'a');
	else UDR = (val/0x10 + '0');
	val = val % 0x10;
	while (!(UCSRA &(1<<UDRE))) ;
	if (val > 0x09) UDR = (val - 0x0a + 'a');
	else UDR = (val + '0');
}

uint16_t get_hex(uint8_t *input_str)
{ //returns first decimal value (0..254) found in ASCII input string of 0xFF if no value found
	uint8_t i=0, j=1;
	uint16_t k;
	
	while (!(((input_str[i]>='0')&&(input_str[i]<='9'))||((input_str[i]>='a')&&(input_str[i]<='f'))||((input_str[i]>='A')&&(input_str[i]<='F'))))
	
	if ((input_str[i]>='0')&&(input_str[i]<='9'))
	{
		while ((input_str[i+j]>='0')&&(input_str[i+j]<='9')) j++;
		switch (j)
		{
			case 1:
			{
				return (input_str[i]-'0');
				break;
			}
			case 2:
			{
				k=(input_str[i]-'0')*10+(input_str[i+1]-'0');
				return (k);
				break;
			}
			case 3:
			{
				k=(input_str[i]-'0')*100+(input_str[i+1]-'0')*10+(input_str[i+2]-'0');
				if (k<255) return (k);
				else return (0xff);
				break;
			}
			default:
			{
				return (0xff);
				break;
			}
			
		}
		
	}
	return (0xff);
}

/*

int uart_putchar0(char c, FILE *stream) //sends the character to UART
{
loop_until_bit_is_set(UCSRA, UDRE);
UDR = c;
return 0;
}


void uart_putchar1(char c) //sends the character to UART
{
loop_until_bit_is_set(UCSRA, UDRE);
UDR = c;
}


int uart_getchar0(FILE *stream) //gets the character from UART
{


return 0;
}

*/



ISR(USART_RXC_vect) //UART0 rx interrupt handler
{
	//checking framing error flag
	if (bit_is_clear(UCSRA, FE))
	{ //symbol received without error
		c = UDR;
		
		
		if (c=='\r') c='\n';
		if (c=='\n')
		{ //new command is complete


			if (rxbufferlast0>0)
			{
				rxbufferfirst0=0;
				rxbuffer0[rxbufferlast0]= '\0';

				do
				{
					havecommand=1;
					cc=rxbuffer0[rxbufferfirst0++];
					switch (cc)
					{
						case reqVOLDEC:
						{
							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							if (temp<=35)
							{
								vol_changed-=temp;
							}
							else
							{
								vol_changed-=1;
								temp = 1;
							}
							if ((outputen))
							{
								prints("VOL +");
								printd(temp);
							}
							break;
						}
						case reqVOLINC:
						{
							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							
							if (temp<=35)
							{
								vol_changed+=temp;

							}
							else
							{
								vol_changed+=1;
								temp = 1;
								
							}
							if ((outputen))
							{
								prints("VOL -");
								printd(temp);
							}
							break;
						}
						case reqVOLLEV:
						{
							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							if (temp<=35)
							{
								vol_set=temp;
							}
							else
							{
								temp=0;
								while((temp<VOL_LEN-1) && vol[temp]>vol_level) temp++;
							}
							if ((outputen))
							{
								prints("VOL ");
								printd(temp);
							}
							break;
						}
						case reqBUILTIN:
						{
							builtin_index = get_dec(&rxbuffer0[rxbufferfirst0]);
							
							if (builtin_index < AINET_COMMANDS_NUM)
							{
								crc(ainet_commands[builtin_index]);
								sendcommand(builtin_index);
								if ((outputen))
								{
									prints("Sent cmd #");
									printd(builtin_index);
								}
							}
							break;
						}
						case reqSEND:
						{
							i=rxbufferfirst0;
							j=0;
							val=0;

							while ((i<=rxbufferlast0)&&(j<10))
							{
								if (isxdigit(rxbuffer0[i]))
								{
									if (isxdigit(rxbuffer0[i+1]))
									{
										if (isdigit(rxbuffer0[i])) val = ((rxbuffer0[i] - '0')<<4);
										else val = ((tolower(rxbuffer0[i]) - 'a' + 10)<<4);
										
										if (isdigit(rxbuffer0[i+1])) val+= (rxbuffer0[i+1] - '0');
										else val+= (tolower(rxbuffer0[i+1]) - 'a' + 10);
										i++;
									}
									else
									{
										if (isdigit(rxbuffer0[i])) val = (rxbuffer0[i] - '0');
										else val = (tolower(rxbuffer0[i]) - 'a' + 10);
									}
									ainet_commands[USER_COMMAND][j++] = val;
								}
								i++;
							}
							
							if (j<10)
							{
								for (i=j;i<10;i++) ainet_commands[USER_COMMAND][i] = 0;
							}
							
							crc(ainet_commands[USER_COMMAND]);
							sendcommand(USER_COMMAND);
							break;
						}
						case reqINPUT:
						{
							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							if ((temp<7)&&(temp>0))
							{ //input # from 1 fo 6
								
								ch_input = temp;
								
								if ((temp<4)&&(outputen))
								{
									
									prints("Input A");
									printd(temp);
								}
								else
								{
									prints("Input D");
									printd(temp -3);
								}
							}
							break;
						}
						case reqWOFINC:
						{

							bass_changed ++;

							if ((outputen)) prints("Sub +1");
							break;
						}

						case reqWOFDEC:
						{
							bass_changed --;
							
							if ((outputen)) prints("Sub -1");
							break;
						}
						case reqPRESET:
						{

							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							if ((temp<7)&&(temp>0))
							{ //preset # from 1 fo 6
								preset_changed = temp;
								
								if ((outputen))
								{
									prints("Preset ");
									printd(temp);
								}
							}
							
							break;
						}
						case reqSLAVE:
						{
							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							if (temp<0xff) slave_en = temp;
							if ((outputen))
							{
								prints("Slave ");
								printd(slave_en);
							}
							break;
						}
						case reqENABLE:
						{
							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							if (temp == 0) proc_en = 10;
							else if (temp < 0xff) proc_en = 11;
							if ((outputen))
							{
								prints("H70x ");
								if ((proc_en == 1)||(proc_en == 11)) prints("en");
								else prints("dis");
							}
							break;
						}
						case reqOUTPUTEN:
						{
							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							if (temp!=0xff) outputen=temp;
							if ((outputen))
							{
								prints("Out: ");
								printd(outputen);
								prints("\r\n");
							}
							break;
						}

						case reqACK:{
							
							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							if (temp!=0xff) ack_en=temp;
							if ((outputen)){
								prints("Ack: ");
								printd(ack_en);
								prints("\r\n");
							}
							break;
						}
						case reqTRACKN:
						{

							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							if (temp!=0xff) track_changed+=temp;
							else track_changed++;
							if ((outputen))
							{
								prints("Track +");
								printd(track_changed);
								prints("\r\n");
							}
							break;
						}
						case reqTRACKP:
						{
							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							if (temp!=0xff) track_changed-=temp;
							else track_changed--;
							if ((outputen))
							{
								prints("Track -");
								printd(track_changed);
								prints("\r\n");
							}
							break;
						}
						case reqDISKN:
						{
							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							if ((temp!=0xff)&&(temp<7))
							{
								disk_changed=temp;
								if ((outputen))
								{
									prints("Disk ");
									printd(disk_changed);
									prints("\r\n");
								}
							}
							else if ((temp == 60)||(temp == 70)) disk_changed=temp; //Pause/Play disc
							else
							{
								disk_changed = 21;
								if ((outputen))
								{
									prints("Next disk\r\n");
								}
							}
							break;
						}
						case reqDISKP:
						{
							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							if ((temp!=0xff)&&(temp<7))
							{
								disk_changed=temp;
								if ((outputen))
								{
									prints("Disk ");
									printd(disk_changed);
									prints("\r\n");
								}
							}
							else
							{
								disk_changed = 11;
								if ((outputen))
								{
									prints("Prev. disk\r\n");
								}
							}
							break;
						}
						case reqFILTERS:
						{
							temp = get_dec(&rxbuffer0[rxbufferfirst0]);
							if (temp!=0xff) filt_en=temp;
							if ((outputen))
							{
								prints("0x7f filter ");
								if (filt_en) prints ("en\r\n");
								else prints ("dis\r\n");
							}
							break;
						}
						case reqRESET:
						{
							resetbus();
							break;
						}
						default: havecommand = 0;
					}
				}
				while ((havecommand==0)&&(rxbufferfirst0<rxbufferlast0));
				
				rxbufferlast0=0;
				rxbufferfirst0=0;
			}
		}
		else if ((c=='\b')&&(rxbufferlast0>0)) rxbufferlast0--;  //backspace pressed
		else if (rxbufferlast0<RXBUFFERSIZE-2) 	rxbuffer0[rxbufferlast0++]=c; //new command is not complete
		else rxbufferlast0=0;  //input string is too long, buffer overflow
		
	}
	else c = UDR; //UART error handler
}
