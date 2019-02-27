/* 
 * File:   r_queue.h
 * Author: Aliyah
 *
 * Created on November 6, 2018, 11:40 AM
 */

#ifndef R_QUEUE_H
#define	R_QUEUE_H

#include "FreeRTOS.h"
#include "queue.h"

void create_r_queue();
void rQueueSendFromISR(unsigned char r_char, BaseType_t *pxHigherPriorityTaskWoken);
unsigned char rQueueReceive();

#endif	/* R_QUEUE_H */

