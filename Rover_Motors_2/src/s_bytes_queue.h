/* 
 * File:   s_bytes_queue.h
 * Author: Aliyah
 *
 * Created on November 6, 2018, 11:40 AM
 */

#ifndef S_BYTES_QUEUE_H
#define	S_BYTES_QUEUE_H

#include "FreeRTOS.h"
#include "queue.h"

void create_s_bytes_queue();
void sBytesQueueSend(char s_byte);
char sBytesQueueReceiveFromISR(BaseType_t *pxHigherPriorityTaskWoken);
char sBytesQueueReceive();
unsigned int sBytesQueueEmptyFromISR();
//

#endif	/* S_BYTES_QUEUE_H */

