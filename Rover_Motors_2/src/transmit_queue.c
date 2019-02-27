/* ************************************************************************** */
/** Descriptive File Name*/

#include "transmit_queue.h"
#include "system/common/sys_common.h"
#include "system_definitions.h"

QueueHandle_t transmit_queue;

void create_transmit_queue()
{
    transmit_queue = xQueueCreate(70, 22);
    //dbgOutputVal(17);
}

void transmitQueueSend(t_mess tM)
{   
    //dbgOutputVal(20);
    xQueueSendToBack(transmit_queue, &tM, portMAX_DELAY);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
}


t_mess transmitQueueReceiveFromISR(BaseType_t pxHigherPriorityTaskWoken)
{
    t_mess tM;
    xQueueReceiveFromISR(transmit_queue, &tM, &pxHigherPriorityTaskWoken);
    return tM;
}

char transmitQueueReceive()
{
    char t_byte;
    xQueueReceive(transmit_queue, &t_byte, portMAX_DELAY);
    return t_byte;
}

unsigned int transmitQueueEmptyFromISR()
{
    if(xQueueIsQueueEmptyFromISR(transmit_queue) == pdTRUE){
        return 1;
    }
    else{
        return 0;
    }
}

/* *****************************************************************************
 End of File
 */


