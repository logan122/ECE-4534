/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system/common/sys_common.h"
#include "system_definitions.h"
#include "debug.h"
#include "sensor_queue.h"
#include "motor_control.h"
#include "motor_globals.h"
#include "sensor_state.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

char isr_count = 0;
char isr_count1 = 0;


void IntHandlerDrvUsartInstance0(void)
{
    //dbgOutputVal(76);
    
    BaseType_t pxHigherPriorityTaskWoken=pdFALSE;
    
    uint8_t t_byte;
    int8_t r_byte;
    
    //if the transmit flag is set
    if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT))
    {
        //dbgOutputVal(IN_TX_INT);
        //if the queue is not empty
        if(sBytesQueueEmptyFromISR() == 0)
        {
            //dbgOutputVal(TX_NOT_EMPTY);
            t_byte = sBytesQueueReceiveFromISR(&pxHigherPriorityTaskWoken);
            PLIB_USART_TransmitterByteSend(USART_ID_1, t_byte);
        }
        else
        {
            //dbgOutputVal(TX_EMPTY);
            PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        }
        
        //clear the flag
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        //SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_TRANSMIT);
    }
    //if there is an error occurring in the uart
    else if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR))
    {
        //clear the flag
        dbgOutputVal(52);
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
        //SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_ERROR);
        //killAll function
    }
    //if the receive flag is set
    if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE))
    {
        //dbgOutputVal(18);
        
        //this allows the uart interrupt to send all of the bytes in the 8 char buffer per interrupt flag
        while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
        {
            r_byte = PLIB_USART_ReceiverByteReceive(USART_ID_1);
 
            if(r_byte < 0 || r_byte > 127) //if the byte is not a valid number
            {
                dbgOutputVal(100);
            }
            if(r_byte == 123) //if the character is '{'
            {
                dbgOutputVal(120);
            }
            if(r_byte == 125) //if the character is '}'
            {
                dbgOutputVal(125);
            }
            rQueueSendFromISR(r_byte, &pxHigherPriorityTaskWoken); //send to the receive queue
        }
        //clear the flag
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
    }
    
    //DRV_USART_TasksTransmit(sysObj.drvUsart0);
    DRV_USART_TasksError(sysObj.drvUsart0);
    //DRV_USART_TasksReceive(sysObj.drvUsart0);
    
    //dbgOutputVal(86);
    portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
}
 
void IntHandlerDrvAdc(void)
{
  //dbgOutputVal(9);
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   sens_message s_mess;
   unsigned int val_FL, val_FR, s_value_FL, s_value_FR; 

   val_FL = readADCBlocking_FL();
   val_FR = readADCBlocking_FR();

   s_value_FL = convertADCVal(val_FL);
   s_value_FR = convertADCVal(val_FR);
   
   dbgOutputVal(55);
   
   
   //margin of error calculation
   if(abs(s_value_FL - s_value_FR) < 3){
       if ((s_value_FL <= 10) || (s_value_FL >= 7)){
           if ((s_value_FR <= 10) || (s_value_FR >= 7)){
               s_mess = STOP;
                sensorQueueSendFromISR(s_mess, &xHigherPriorityTaskWoken);
                
                isr_count1 = 0;
           }
       }
       else{
           s_mess = data.action;
           sensorQueueSendFromISR(s_mess, &xHigherPriorityTaskWoken);
       }
   }

   //dbgOutputVal(s_value_FL);
   //dbgOutputVal(s_value_FR);
    
   /* Clear ADC Interrupt Flag */
   PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);
   portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
 

void IntHandlerDrvTmrInstance0(void)
{
    //PLIB_INT_SourceFlagSet(INT_ID_0, INT_SOURCE_ADC_1);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}

//Motor L
void IntHandlerDrvTmrInstance1(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_3);
}

//Motor R
void IntHandlerDrvTmrInstance2(void)
{
    isr_count++;
    isr_count1++;

    /* Get the current value from the timer registers */
    short int ticksL = DRV_TMR0_CounterValueGet();
    short int ticksR = DRV_TMR1_CounterValueGet();

    /* Clear the counter registers */
    DRV_TMR0_CounterClear();
    DRV_TMR1_CounterClear();

    /* increment the distance traveled per motor in ticks*/
    distL        += ticksL;
    distR        += ticksR;
    total_ticksL += ticksL;
    total_ticksR += ticksR;

    /* Are Both Motors Stopped ? */
    bool stopped  = getMotorL_DC() == 0 && getMotorR_DC() == 0;

    /* Outgoing Message Construction */
    struct motorQueueData data;
    if (distL >= TICKS_PER_CM && getMotorL_DC() != 0)
    {
        unsigned int cm = (distL / TICKS_PER_CM) - prev_cm;
        prev_cm = distL / TICKS_PER_CM;

        data.dist = cm;
        data.action = getMotorAction();
        sendMsgToMotorQFromISR(data);
    }

    if (distL >= goalL || distR >= goalR || stopped)
    {
        distL    = 0;
        distR    = 0;
        prev_cm  = 0;

        if (!leftQIsEmpty())
            readFromQandSetPins(LEFT);
        else
            setMotorL_DC(0);

        if (!rightQIsEmpty())
            readFromQandSetPins(RIGHT);
        else
            setMotorR_DC(0);
    }
    
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_4);
}
 /*******************************************************************************
 End of File
*/
