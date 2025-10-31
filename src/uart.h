/*
UART routines: init, putchar, getchar. Not used directly - only through
standard C I/O functions.
 */

#ifndef UART_H
#define UART_H
//Запрос версии (требует ответа outVERSION)
#define reqSEND 's'
#define reqBUILTIN 'l'
#define reqOUTPUTEN 'o'
#define reqFILTERS 'e'
#define reqJUMPERS 'j'
#define reqVOLDEC 'v'
#define reqVOLINC 'V'
#define reqVOLLEV 'L'
#define reqBALDEC 'b'
#define reqBALINC 'B'
#define reqFADDEC 'f'
#define reqFADINC 'F'
#define reqWOFDEC 'w'
#define reqWOFINC 'W'
#define reqPROGRAMMING 'P'
#define reqRESET 'r'
#define reqACK 'a'
#define reqINPUT 'i'
#define reqPRESET 'p'
#define reqSTORE 'R'
#define reqUART 'u'
#define reqACC 'c'
#define reqDISKN 'D'
#define reqDISKP 'd'
#define reqTRACKN 'T'
#define reqTRACKP 't'
#define reqSLAVE 'm'
#define reqENABLE 'n'

#define VOL_LEN 36

extern volatile uint8_t outputen;
extern volatile uint8_t ack_en, filt_en, cdc_en, ch_input, slave_en, proc_en;
extern volatile uint8_t vol_level, bal_level, fad_level, vol_set, vol_level_back, disk_changed, preset_changed, acc_level;
extern volatile int8_t vol_changed, bal_changed, fad_changed, bass_changed, track_changed;

extern uint8_t vol[VOL_LEN];
extern uint8_t ainet_input[6];
extern volatile uint8_t H701_pwr_st;

extern volatile uint8_t flags1;

extern uint8_t jmpr1 EEMEM;

extern void sendcommand(uint8_t commandindex);
extern void resetbus();
extern void invertLED (uint8_t lednum);


void uart_init(void);

void printh (uint8_t val);

void printd (uint8_t val);

void prints (char *input_str); 


int	uart_putchar0(char c, FILE *stream);
int	uart_getchar0(FILE *stream);
#endif
