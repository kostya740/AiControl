#define F_CPU 1000000UL	/* CPU clock in Hertz */
#define UART_BAUD  38400

#define MAX_START_VOL 0x14 //Maximum volume level at startup (23)

#define glue(a,b) a##b
#define PORT(name) glue(PORT,name)
#define DDR(name) glue(DDR,name)
#define PIN(name) glue(PIN,name)

#define AINETPORT D
#define AINET 3
#define AINETOUT 4

#define ACCPORT B
#define ACCIN 7
#define ACCOUT 6
