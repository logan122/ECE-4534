/* 
 * File:   sensor_queue.h
 * Author: Logan
 *
 * Created on February 12, 2019, 9:33 PM
 */

#ifndef SENSOR_QUEUE_H
#define	SENSOR_QUEUE_H

#include "FreeRTOS.h"
#include "queue.h"

typedef struct sensor_message
{
    unsigned int value_FR; //sensor value FR
    unsigned int value_FL; //sensor value FL
    unsigned int value_LS; //sensor value LS
    unsigned int value_RS; //sensor value RS
    char units[14];     //centimeters & commas
}sens_message;

QueueHandle_t sens_queue_FL; //sensor queue
QueueHandle_t sens_queue_FR; //sensor queue
QueueHandle_t sens_queue_LS; //sensor queue
QueueHandle_t sens_queue_RS; //sensor queue

unsigned int readADCBlocking_FL();
unsigned int readADCBlocking_FR();
unsigned int readADCBlocking_LS();
unsigned int readADCBlocking_RS();
unsigned int convertADCVal(unsigned int ADCVal);

void sensorQueueSend(unsigned int convertedVal_FL, unsigned int convertedVal_FR, unsigned int convertedVal_LS, unsigned int convertedVal_RS);
void sensorQueueSendFromISR(sens_message mess, BaseType_t *xHigherPriorityTaskWoken);
sens_message sensorQueueReceive_FL();
sens_message sensorQueueReceive_FR();
sens_message sensorQueueReceive_LS();
sens_message sensorQueueReceive_RS();
sens_message sensorQueueReceiveFromISR();
void create_sensor_queue();

#endif /* SENSOR_QUEUE_H */

/* *****************************************************************************
 End of File
 */