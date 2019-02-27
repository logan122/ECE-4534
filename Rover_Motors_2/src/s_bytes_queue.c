/* ************************************************************************** */
/** Descriptive File Name*/

#include "s_bytes_queue.h"
#include "system/common/sys_common.h"
#include "system_definitions.h"

QueueHandle_t s_bytes_queue;

void create_s_bytes_queue()
{
    s_bytes_queue = xQueueCreate(100, sizeof(char));
    //dbgOutputVal(17);
}

void sBytesQueueSend(char s_byte)
{   
    //dbgOutputVal(20);
    xQueueSendToBack(s_bytes_queue, &s_byte, portMAX_DELAY);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
 
}


char sBytesQueueReceiveFromISR(BaseType_t *pxHigherPriorityTaskWoken)
{
    char t_char;
    xQueueReceiveFromISR(s_bytes_queue, &t_char, pxHigherPriorityTaskWoken);
    return t_char;
}

char sBytesQueueReceive()
{
    char s_byte;
    xQueueReceive(s_bytes_queue, &s_byte, portMAX_DELAY);
    return s_byte;
}

unsigned int sBytesQueueEmptyFromISR()
{
    if(xQueueIsQueueEmptyFromISR(s_bytes_queue) == pdTRUE){
        return 1;
    }
    else{
        return 0;
    }
}

/* *****************************************************************************
 End of File
 */


