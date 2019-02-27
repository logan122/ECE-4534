/* ************************************************************************** */
/** Descriptive File Name*/

#include "r_queue.h"
#include "system/common/sys_common.h"
#include "system_definitions.h"

QueueHandle_t r_queue = NULL;

void create_r_queue()
{
    r_queue = xQueueCreate(100, sizeof(char));
}

void rQueueSendFromISR(unsigned char r_char, BaseType_t *pxHigherPriorityTaskWoken)
{
    if(xQueueSendToBackFromISR(r_queue, &r_char, pxHigherPriorityTaskWoken) != pdTRUE)
    {
        dbgOutputVal(100);
    }
    //dbgOutputVal(29);
}

unsigned char rQueueReceive()
{
    unsigned char r_byte;
    xQueueReceive(r_queue, &r_byte, portMAX_DELAY);
    //dbgOutputVal(13);
    return r_byte;
}


/* *****************************************************************************
 End of File
 */



