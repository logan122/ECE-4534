#include "sensor_queue.h"
#include "system/common/sys_common.h"
#include "system_definitions.h"
#include "debug.h"

unsigned int readADCBlocking_FL()
{
    DRV_ADC_Start();
    while(!(DRV_ADC_SamplesAvailable()));
    return (unsigned int)DRV_ADC_SamplesRead(0);
}

unsigned int readADCBlocking_FR()
{
    DRV_ADC_Start();
    while(!(DRV_ADC_SamplesAvailable()));
    return (unsigned int)DRV_ADC_SamplesRead(1);
}

unsigned int readADCBlocking_LS()
{
    DRV_ADC_Start();
    while(!(DRV_ADC_SamplesAvailable()));
    return (unsigned int)DRV_ADC_SamplesRead(2);
}

unsigned int readADCBlocking_RS()
{
    DRV_ADC_Start();
    while(!(DRV_ADC_SamplesAvailable()));
    return (unsigned int)DRV_ADC_SamplesRead(3);
}



unsigned int convertADCVal(unsigned int ADCVal)
{
    unsigned int adc = ADCVal;
    unsigned int conversion;
    
    conversion = (2914/(ADCVal+5))-1;
    
    return conversion;
}


void create_sensor_queue()
{
    //dbgOutputVal(100)
    sens_queue_FL = xQueueCreate(70, sizeof( unsigned int ) + (11*sizeof(char)));
    sens_queue_FR = xQueueCreate(70, sizeof( unsigned int ) + (11*sizeof(char)));
}

void sensorQueueSend(unsigned int convertedVal_FL, unsigned int convertedVal_FR, unsigned int convertedVal_LS, unsigned int convertedVal_RS)
{   
    //dbgOutputVal(95)
    xQueueSend(sens_queue_FL, &convertedVal_FL, portMAX_DELAY);
    xQueueSend(sens_queue_FR, &convertedVal_FR, portMAX_DELAY);
}

sens_message sensorQueueReceive_FL()
{
    sens_message val;
    xQueueReceive(sens_queue_FL, &val, portMAX_DELAY);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    //dbgOutputVal(90);
    return val;
}

sens_message sensorQueueReceive_FR()
{
    sens_message val;
    xQueueReceive(sens_queue_FR, &val, portMAX_DELAY);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    //dbgOutputVal(85);
    return val;
}

void sensorQueueSendFromISR(sens_message mess, BaseType_t *xHigherPriorityTaskWoken) 
{
    xQueueSendToBackFromISR( sens_queue_FL, &mess, xHigherPriorityTaskWoken );
    xQueueSendToBackFromISR( sens_queue_FR, &mess, xHigherPriorityTaskWoken );
    //dbgOutputVal(80);
}

/* *****************************************************************************
 End of File
 */


