#include <stdio.h>

#define AINET_COMMANDS_NUM 18	
#define AINET_WATCH_NUM 5
#define AINET_INIT_LENGTH 7

#define USER_COMMAND 11

extern uint8_t ainet_commands[AINET_COMMANDS_NUM][11];
extern uint8_t ainet_watch[AINET_WATCH_NUM][11];

void crc(uint8_t *packet);
