/* 
 * File:   sensor_state.h
 * Author: Logan
 *
 * Created on February 12, 2019, 9:33 PM
 */

#ifndef SENSOR_STATE_H
#define	SENSOR_STATE_H

#define SENS_STATE_1 1
#define SENS_STATE_2 2
#define SENS_STATE_3 3
#define SENS_STATE_4 4
#define SENS_STATE_5 5

#include "debug.h"
#include "sensor_queue.h"

unsigned int sens_state = SENS_STATE_1;
void sensor_state(int *current_state, unsigned int *value_total_FL, unsigned int sensor_value_FL, unsigned int *value_total_FR, unsigned int sensor_value_FR, unsigned int *value_total_LS, unsigned int sensor_value_LS, unsigned int *value_total_RS, unsigned int sensor_value_RS);

#endif

