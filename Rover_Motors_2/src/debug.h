/* 
 * File:   debug.h
 * Author: Logan
 *
 * Created on February 12, 2019, 9:32 PM
 */

#ifndef DEBUG_H
#define	DEBUG_H

#include "app.h"
#include <p32xxxx.h>
#include <stdio.h>
#include <stdlib.h>

#define MAXVAL 127

// Task-related dbgOutputLoc arguments. 
#define DLOC_ENTERING_TASK 0
#define DLOC_BEFORE_WHILE_ONE_TASK 0
#define DLOC_BEFORE_SENDING_TASK_QUEUE 0
#define DLOC_BEFORE_RECEIVE_TASK_QUEUE 0
#define DLOC_AFTER_SENDING_TASK_QUEUE 0
#define DLOC_AFTER_RECEIVE_TASK_QUEUE 0

// ISR-related dbgOutputLoc arguments.
#define DLOC_BEFORE_ENTER_ISR 0
#define DLOC_BEFORE_LEAVE_ISR 0
#define DLOC_BEFORE_SENDING_ISR_QUEUE 0
#define DLOC_BEFORE_RECEIVE_ISR_QUEUE 0
#define DLOC_AFTER_SENDING_ISR_QUEUE 0
#define DLOC_AFTER_RECEIVE_ISR_QUEUE 0

#define IN_TX_INT 0
#define TX_NOT_EMPTY 0
#define TX_SENT 0
#define TX_EMPTY 0


void dbgInit();

void dbgOutputVal(unsigned int outVal);

void dbgUARTVal(unsigned char outVal);

void dbgOutputLoc(unsigned int outVal);

void dbgHaltAll(unsigned int outVal);

#endif	/* DEBUG_H */

