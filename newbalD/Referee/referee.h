#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

typedef struct 
  {
  // 云台接收
  uint16_t maxHeat;   // 枪管最大热量
  uint16_t gunHeat1;  // 枪管1热量
  uint8_t target;     // 目标
  float  gunSpeed1;     // 枪管1射速

} Referee_data;

extern Referee_data RefereeData;

extern void refereeINIT(void);
extern void referee_unpack_fifo_data(void);
extern void refereeReceiveHandler(void);
void RefereeDataUpdate(Referee_data *data);

#endif
