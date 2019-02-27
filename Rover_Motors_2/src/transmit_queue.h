/* 
 * File:   transmit_queue.h
 * Author: Aliyah
 *
 * Created on November 6, 2018, 11:40 AM
 */

#ifndef TRANSMIT_QUEUE_H
#define	TRANSMIT_QUEUE_H

#include "FreeRTOS.h"
#include "queue.h"

typedef struct tx_mess
{
    char m[22];
}t_mess;

void create_transmit_queue();
void transmitQueueSend(t_mess tM);
t_mess transmitQueueReceiveFromISR(BaseType_t pxHigherPriorityTaskWoken);
char transmitQueueReceive();
unsigned int transmitQueueEmptyFromISR();

#endif	/* S_BYTES_QUEUE_H */

