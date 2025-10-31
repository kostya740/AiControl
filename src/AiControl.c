/*
* AiControl.c
*
* Created: 21.09.2014 11:15:46
*  Author: Константин
*/

#include "hardware.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include "uart.h"
#include "ainetcommands.h"

//number of retries in case we didn't receive any acknowledgements to our command
#define AINET_RESEND_COUNT 5

volatile uint8_t buffer[13];
volatile uint8_t buffer_out[13];
volatile uint8_t byteindex,bitindex, packet_size, overflow = 0;
volatile uint8_t dataready, ackready = 0, cdc_ack = 0;

volatile uint8_t waiting_for_ack = 0; //0 - we are not waiting, >0 - waiting for this number
volatile uint8_t resend_counter = 0;
volatile uint8_t last_sent_command = 0; //which command are we going to resend if any errors happen
volatile uint8_t ready_to_send = 1;
volatile uint8_t H701_init = 0;
volatile uint8_t H701_pwr_st;
volatile uint8_t vol_sync_cnt = 0;

uint8_t vol_state EEMEM;

void resetbus()
{

	cli();

	PORT( AINETPORT ) |= _BV( AINETOUT );
	_delay_us( 50 );
	PORT( AINETPORT ) &= ~_BV(AINETOUT);
	_delay_us( 9 );
	sei ();

}

void resendcommand( uint8_t commandindex )
{
	uint8_t i,j,type;
	unsigned char sreg_tmp;

	sreg_tmp = SREG;
	cli();

	PORT( AINETPORT ) |= _BV( AINETOUT );
	_delay_us( 31.5 );
	PORT( AINETPORT ) &= ~_BV( AINETOUT );
	_delay_us( 9 );
	for ( i=0; i < 11; i++ )
	{
		for ( j=0; j < 8; j++ )
		{
			type=( ainet_commands[commandindex][i] & _BV( 7-j ) ) >> ( 7-j );
			if ( type == 0 )
			{
				PORT( AINETPORT ) |= _BV( AINETOUT );
				_delay_us( 16 );
				PORT( AINETPORT ) &= ~_BV( AINETOUT );
				_delay_us( 4 );
			}
			else
			{
				PORT( AINETPORT ) |= _BV( AINETOUT );
				_delay_us( 8 );
				PORT( AINETPORT ) &= ~_BV( AINETOUT );
				_delay_us( 12 );
			}
		}
	}

	resend_counter++;
	waiting_for_ack = ainet_commands[last_sent_command][0]; //we've just sent a command, need ack to make sure it was received by someone

	MCUCR |= _BV( ISC10 ); //next look for rising edge of INT1
	//now set up timer interrupt for timeout
	TIFR |= _BV( TOV0 );
	TIMSK |= _BV( TOIE0 );
	bitindex = 0;
	byteindex = 11;
	for ( i=0; i < 11; i++ )
	{
		buffer[i] = ainet_commands[commandindex][i];
		buffer_out[i] = ainet_commands[commandindex][i];
	}

	dataready = 1;


	TCNT0 = 0;
	
	//clear up all pending interrupts which could've happened during sending
	GICR |= _BV( INT1 );
	TIFR |= _BV( TOV0 );
	SREG = sreg_tmp;
}

void sendcommand(uint8_t commandindex)
{
	resend_counter=0;
	last_sent_command=commandindex;
	while (!ready_to_send);
	resendcommand (commandindex);
}

void SetACC( uint8_t status )
{
	if ( status ) //have ACC on - start the processor
	{
		PORT( ACCPORT ) |= _BV( ACCOUT );
		vol_set = eeprom_read_byte( &vol_state ); //set up volume level after startup

	}
	else
	{
		if ( vol_level_back != 0xFF ) //unmute when ACC off
		{
			vol_level = vol[vol_level_back];
			vol_level_back = 0xFF;
		}

		eeprom_write_byte ( &vol_state, vol_level ); //save up the volume level
		//	eeprom_write_byte (&cdc_state, cdc_en); //save up cd changer select state

		PORT( ACCPORT ) &= ~_BV( ACCOUT ); //ACC off
	}
}

ISR( INT1_vect )
{
	
	uint8_t t,j;
	t = TCNT0;
	ready_to_send = 0;
	if ( MCUCR & _BV( ISC10 ) )
	{
		TCNT0 = 0;
		MCUCR &= ~_BV( ISC10 ); //next look for falling edge of INT1
		TIMSK &= ~_BV( TOIE0 ); //no more timer interrupts for now
		//TODO: some kind of timer should be used to still work even if INT1 pin is always high
		//TODO: don't forget to reset ready_to_send there
	}
	else
	{
		MCUCR |= _BV( ISC10 ); //next look for rising edge of INT1
		if ( t < 24 ) //logic 1 on AiNet
		{
			TCNT0 = 255 - ( 2 * 26 ); //26us timeout
			buffer[byteindex] |= _BV( 7 - bitindex );
			if ( ++bitindex > 7 )
			{
				bitindex = 0;
				byteindex++;
			}
		}
		else if ( t < 48 ) //logic 0 on AiNet
		{
			TCNT0=255 - ( 2 * 18 ); //18us timeout
			buffer[byteindex] &= ~_BV( 7 - bitindex );
			if( ++bitindex > 7 )
			{
				bitindex = 0;
				byteindex++;
			}
		}
		else //start of frame
		{
			byteindex = 0;
			bitindex = 0;
			//      TCNT0=255-(2*40); //40us timeout - one full bit, just in case
			waiting_for_ack = 0;
			TCNT0 = 0;
		}
		
		if ( ack_en == 0x0f ) ack_en = 1;
		
		if ( ( byteindex==12 ) && ( bitindex==0 ) )
		{
			//		if (buffer[11]!=0x7f)
			ackready = buffer[11];
			byteindex = 0;
			bitindex = 0;
		}

		if( ( byteindex==11 ) && ( bitindex==0 ) ) //packet without ack is 11 bytes
		{
			//        packet_size = byteindex;
			
			//		if (buffer[0]!=0x7f) {
			for( j=0; j < 11; j++ )
			{
				buffer_out[j] = buffer[j]; //copy input buffer content to output buffer
			}
			if ( dataready == 1 ) overflow = 1;
			dataready = 1;
			//		}
			
			if ( ( ( buffer[0]==0x02 ) || ( buffer[0]==0x50 ) ) && ( ack_en ) )
			{

				//do nothing - don't change TCNT0 - it's good already
			}
			else
			{
				TCNT0 = 0; //wait as long as possible - about 128us - hoping to catch ack into buffer
			}

			if ( ( buffer[0]==0x02 ) && ( buffer[1]==0x40 ) && ( buffer[2]==0x90 ) && ( buffer[3] == 0x67 ) && ( buffer[4] == 0x50 ) ) //Skip ack to enable H701 standalone mode
			{
				
				if ( ack_en == 1 )
				{
					ack_en = 0x0f; //skip ack for HU detection to enable stand alone mode
					H701_init = 1; //init H701 to enable presets switching
				}
				if ( slave_en ) ack_en = 1;
			}
			
			if ( ( buffer[0] == 0x02 ) && ( buffer[1] == 0x32 ) && ( buffer[2] == 0x90 ) && ( buffer[3] == 0x67 ) && ( buffer[4] == 0x50 ) ) cdc_ack = 2;



			//      byteindex=0;
			//      bitindex=0;
			//      dataready=1;
			//      TIMSK0&=~_BV(TOIE0); //no more timer interrupts for now - the packet is over
			//      if (waiting_for_ack==buffer_out[11]) {
			//        waiting_for_ack=0;
			//      } else if (resend_counter<AINET_RESEND_COUNT) {
			//        _delay_ms(1); //TODO: use a timer or something instead of this delay
			//        resendcommand(last_sent_command);
			//      } else {
			//        waiting_for_ack=0;
			//      }
			//      ready_to_send = 1;
			waiting_for_ack = buffer_out[0];
		}
		//now set up timer interrupt for timeout
		TIFR |= _BV( TOV0 );
		TIMSK |= _BV( TOIE0 );
		//	}
	}
}

ISR( TIMER0_OVF_vect )
{
	uint8_t j, type;

	packet_size = byteindex;

	TIMSK &= ~_BV( TOIE0 ); //no more timer interrupts for now
	//we do not process the packet here, but we have to (urgently) send ack byte
	//if the packet was for us. we repeat the first byte of the packet if we
	//are responsible for the target device

	if( ( ( buffer[0] == 0x02 ) || ( buffer[0] == 0x50 ) || ( buffer[0] == 0x42 ) ) && ( ack_en == 1 ) )
	{
		//ok, first byte is 00000010 or 01010000 - target device id 2 or 50 - it's us - send it back now
		
		if ( cdc_ack == 2 ) cdc_ack = 1;

		for ( j=0; j < 8; j++ )
		{
			type = ( buffer[0] & _BV( 7 - j ) ) >> ( 7 - j );
			if ( type == 0 )
			{
				PORT( AINETPORT ) |= _BV(  AINETOUT);
				_delay_us( 16 );
				PORT( AINETPORT ) &= ~_BV(  AINETOUT);
				_delay_us( 5 );
			}
			else
			{
				PORT( AINETPORT ) |= _BV( AINETOUT );
				_delay_us( 8 );
				PORT( AINETPORT ) &= ~_BV( AINETOUT );
				_delay_us( 13 );
			}
		}
		ackready = buffer[0];
	}

	waiting_for_ack = 0;
	
	ready_to_send = 1;
	// clear up INT1 interrupt

	//clear up all pending interrupts which could happened during sending
	GICR |= _BV( INT1 );
	TIFR |= _BV( TOV0 );
}

static void ioinit( void )
{
	uint8_t i;
	
	cli();

	for ( i=0; i < 10; i++ ) _delay_ms (10 );
	
	ack_en = 1;
	outputen = 1;
	slave_en = 0;
	
	uart_init();

	DDR( AINETPORT ) &= ~_BV( AINET );
	DDR( AINETPORT ) |= _BV( AINETOUT );
	PORT( AINETPORT ) &= ~_BV( AINET );
	PORT( AINETPORT ) &= ~_BV( AINETOUT );
	
	DDR( ACCPORT ) &= ~( _BV( ACCIN ) ); //|_BV(12)|_BV(13));
	DDR( ACCPORT ) |= ( _BV( ACCOUT ) );
	PORT( ACCPORT ) &= ~( _BV( ACCIN ) | _BV( ACCOUT ) );
	
	TCCR0 = _BV( CS01 ); //timer0 - 8 bit, clock/8
	

	//  wdt_enable(WDTO_500MS);
	//  wdt_reset();
	
	set_sleep_mode( SLEEP_MODE_IDLE );
	//  prints("Init! \r\n");
	
	sei();
}

int main(void)
{
	uint8_t i,j;
	uint8_t k,vol_step;

	dataready=0;
	byteindex=0;
	bitindex=0;
	ioinit();
	vol_set = 0xFF;
	
	while(1)
	{
		wdt_reset();
		
		if (dataready)
		{
			for (i=0;i<AINET_WATCH_NUM;i++)
			{
				j = 0;
				while (ainet_watch[i][j] == buffer_out[j]) j++;
				
				if (j>=ainet_watch[i][10] - 1) //matching packet found!
				{
					switch (i)
					{
						case 0: //volume/bal/fader level broadcast
						{
							if (vol_sync_cnt == 0) //get current volume level only if
							{
								//					vol_level = buffer_out[3];
								//					bal_level = buffer_out[4];
								//					fad_level = buffer_out[5];
							}
							
							if (vol_level == 0xff) vol_level = buffer_out[3];
							else if (vol_level == buffer_out[3]) vol_sync_cnt = 0;
							else vol_sync_cnt--;
							
							if ((ack_en)&&(H701_init == 2))
							{
								H701_init = 1;
							}
							break;
						}
						case 1:
						{
							//				H701_init = 2;
							//				_delay_us(50);
							//				ack_en = 1;
							break;
						}
						case 2:
						{
							//				if (ack_en) sendcommand(14);
							break;
						}
						case 3: //volume set ack
						{
							if ((buffer_out[5] == 0x20)&&(buffer_out[6] == 0xff))
							{
								vol_level = buffer_out[4];
								vol_sync_cnt = 5;
							}
							break;
						}
					}
				}
			}
			if ((outputen))
			{
				if (!((filt_en)&&(ackready == 0x7f)))
				{
					prints("\r\n");
					for (i=0;i<11;i++)
					{
						printh(buffer_out[i]);
						prints(" ");
					}
				}
			}
			
			if (overflow)
			{
				//			prints("\r\n !!!!!!!");
				overflow = 0;
			}
			dataready=0;
		}
		
		if ((outputen)&&(ackready > 0))
		{
			if (!((filt_en)&&(ackready == 0x7f)))
			{
				printh(ackready);
				prints(" ");
			}
			ackready = 0;
		}
		
		if ((cdc_ack == 1)) //cd changer init
		{
			cdc_ack = 0;
			sendcommand(14);
			for (i=0;i<100;i++) _delay_ms(10); //delay 1s
			sendcommand(15);
			for (i=0;i<100;i++) _delay_ms(10); //delay 1s
			sendcommand(16);
			for (i=0;i<20;i++) _delay_ms(10);
		}
		
		if (vol_set < 0xFF) //set volume request
		{
			if (vol_set<=35) //requested volume level within limit
			{
				ainet_commands[7][3]=vol[vol_set];
				crc(ainet_commands[7]);
				sendcommand(7);
			}
			vol_set = 0xFF;
		}
		
		if (vol_changed)
		{
			i=0;
			while((i<VOL_LEN-1) && vol[i]>vol_level) i++;
			vol_step=i;
			
			if (vol_changed>0) //volume increased
			{
				if (vol_step>VOL_LEN-vol_changed-1) k=vol[VOL_LEN-1]; else k=vol[vol_step+vol_changed];
			}
			else //volume decreased
			{
				if (vol_step<-vol_changed) k=vol[0]; else k=vol[vol_step+vol_changed];
			}
			vol_changed = 0;
			
			ainet_commands[7][3]=k;
			crc(ainet_commands[7]);
			sendcommand(7);
		}
		
		if (bass_changed)
		{
			if (bass_changed>0)
			{
				for (i=0;i<bass_changed;i++)
				{
					ainet_commands[13][4]=0x73;
					crc(ainet_commands[13]);
					sendcommand(13);
					for (i=0;i<10;i++) _delay_ms(10);
				}
			}
			else
			{
				for (i=0;i<-bass_changed;i++)
				{
					ainet_commands[13][4]=0x63;
					crc(ainet_commands[13]);
					sendcommand(13);
					for (i=0;i<10;i++) _delay_ms(10);
				}
			}
			bass_changed = 0;
		}
		
		if (preset_changed)
		{
			
			ainet_commands[10][5]=preset_changed;
			crc(ainet_commands[10]);
			sendcommand(10);
			
			preset_changed = 0;
		}
		
		if (track_changed)
		{
			if (track_changed>0)
			{
				for (i=0;i<track_changed;i++)
				{

					ainet_commands[17][2]=0xd5;
					ainet_commands[17][3]=0x75;
					crc(ainet_commands[17]);
					sendcommand(17);

					for (i=0;i<10;i++) _delay_ms(10);
				}
			}
			else
			{
				for (i=0;i<-track_changed;i++)
				{

					ainet_commands[17][2]=0xd5;
					ainet_commands[17][3]=0x65;
					crc(ainet_commands[17]);
					sendcommand(17);

					for (i=0;i<10;i++) _delay_ms(10);

				}
			}
			track_changed = 0;
		}
		
		if ((ack_en)&&(H701_init == 1)) //Init H701 after enabling ACC
		{
			j = eeprom_read_byte (&vol_state);

			for (i=0;i<200;i++) _delay_ms(10); //delay 2s ????
			sendcommand (2); //init command for preset switching support

			for (i=0;i<10;i++) _delay_ms(10);
			
			if ((slave_en)&&(proc_en))
			{
				ainet_commands[6][3]= 0x70;
				crc(ainet_commands[6]);
				sendcommand(6);
				for (i=0;i<10;i++) _delay_ms(10);
			}
			
			vol_changed = 0;
			H701_init = 0;
		}
		
		/*		if (H701_pwr_st >= 0x80) {
		if (H701_pwr_st == 0x80) { // //powering H701 down
		eeprom_write_byte (&vol_state, vol_level); //save up the volume level
		PORT(ACCPORT)&=~_BV(ACCOUT); //ACC off
		H701_init = 0;
		H701_pwr_st = 0;
		}
		
		else if (H701_pwr_st == 0x81) { // //powering H701 up
		PORT(ACCPORT)|=_BV(ACCOUT); //ACC on
		H701_init = 1;
		H701_pwr_st = 1;
		}
		} */

		if (ch_input) //Switch H701 input
		{
			if ((ch_input<4))
			{
				ainet_commands[12][3]=ainet_input[ch_input-1]; //Enable Analogue input
				crc(ainet_commands[12]);
				sendcommand(12);
				
				for (i=0;i<10;i++) _delay_ms(10);
				
			}
			else if (ch_input<7)
			{
				
				ainet_commands[12][3]=ainet_input[ch_input-1]; //Enable Digital input
				crc(ainet_commands[12]);
				sendcommand(12);
				
				for (i=0;i<10;i++) _delay_ms(10);
			}
			ch_input=0;
		}

		if (proc_en > 9) //Switch H701 input
		{
			if (proc_en == 10) //Set the processor to sleep
			{
				ainet_commands[6][3]= 0x60;
				crc(ainet_commands[6]);
				sendcommand(6);
				for (i=0;i<10;i++) _delay_ms(10);
				proc_en = 0;
			}
			if (proc_en == 11) //Wake up the processor
			{
				ainet_commands[6][3]= 0x70;
				crc(ainet_commands[6]);
				sendcommand(6);
				for (i=0;i<10;i++) _delay_ms(10);
				proc_en = 1;
			}
		}
		/*
		if ((H701_pwr_st == 0)&&(PIN(ACCPORT)&(_BV(ACCOUT))))
		{  //additional power off check
		PORT(ACCPORT)&=~_BV(ACCOUT); //ACC off
		} */
		
		/*		if (H701_pwr_st == 0) {
		sleep_enable ();
		sleep_cpu();
		sleep_disable();
		} */

		if (disk_changed)
		{ //Change disk in CD changer
			
			if ((disk_changed > 0)&&(disk_changed < 7))
			{
				ainet_commands[17][2]=0xd2;
				ainet_commands[17][3]=0x20 + disk_changed;
				crc(ainet_commands[17]);
				sendcommand(17);
			}
			else if (disk_changed == 11)
			{ //prev. disk
				ainet_commands[17][2]=0xd2;
				ainet_commands[17][3]=0x63;
				crc(ainet_commands[17]);
				sendcommand(17);
			}
			else if (disk_changed == 21)
			{ //Next disk
				ainet_commands[17][2]=0xd2;
				ainet_commands[17][3]=0x73;
				crc(ainet_commands[17]);
				sendcommand(17);
			}
			else if (disk_changed == 60)
			{ //Stop
				ainet_commands[17][2]=0xd3;
				ainet_commands[17][3]=0x60;
				crc(ainet_commands[17]);
				sendcommand(17);
			}
			else if (disk_changed == 70)
			{ //Play
				ainet_commands[17][2]=0xd3;
				ainet_commands[17][3]=0x70;
				crc(ainet_commands[17]);
				sendcommand(17);
			}
			disk_changed = 0;
		}
	}
	
	if ( acc_level != ( PIN( ACCPORT ) & _BV( ACCIN ) ) )
	{
		acc_level = ( PIN( ACCPORT ) & _BV( ACCIN ) );
		SetACC( acc_level );
	}
	return 0;
}