#include "sensor_state.h"
#include "debug.h"

void sensor_state(int *current_state, unsigned int *value_total_FL, unsigned int sensor_value_FL, unsigned int *value_total_FR, unsigned int sensor_value_FR)
{
	//store the average of the 5 cases without static or global variables
	switch(*current_state) {
        
		case 1:
			
			*value_total_FL = sensor_value_FL; //reset value_total to the current sensor_value
            *value_total_FR = sensor_value_FR; //reset value_total to the current sensor_value
			//dbgOutputVal(sensor_value_FL); 
            //dbgOutputVal(sensor_value_FR); 
			*current_state = 2;
			break;
		case 2:
            
            *value_total_FL = *value_total_FL + sensor_value_FL; //reset value_total to the current sensor_value
            *value_total_FR = *value_total_FR + sensor_value_FR; //reset value_total to the current sensor_value
            //dbgOutputVal(sensor_value_FL);
            //dbgOutputVal(sensor_value_FR);

			*current_state = 3;
			break;
		case 3:

            *value_total_FL = *value_total_FL + sensor_value_FL; //reset value_total to the current sensor_value
            *value_total_FR = *value_total_FR + sensor_value_FR; //reset value_total to the current sensor_value
            //dbgOutputVal(sensor_value_FL);
            //dbgOutputVal(sensor_value_FR);
			*current_state = 4;
			break;
		case 4:
		
			*value_total_FL = *value_total_FL + sensor_value_FL; //reset value_total to the current sensor_value
            *value_total_FR = *value_total_FR + sensor_value_FR; //reset value_total to the current sensor_value        
            //dbgOutputVal(sensor_value_FL);
            //dbgOutputVal(sensor_value_FR);
			*current_state = 5;
			break;
		case 5:
			
			*value_total_FL = *value_total_FL + sensor_value_FL; //reset value_total to the current sensor_value
            *value_total_FR = *value_total_FR + sensor_value_FR; //reset value_total to the current sensor_value      
            //dbgOutputVal(sensor_value_FL);
            //dbgOutputVal(sensor_value_FR);
			
			//get the average of the last 5 cases and print out
			unsigned char averageFive_FL = (*value_total_FL)/5;
            unsigned char averageFive_FR = (*value_total_FR)/5;
			//dbgUARTVal(averageFive);
			*current_state = 1;
			break;
		default:
			//Throw error
           ////////////******** dbgHaltAll(151); //TODO define a error code constant and use here
			break;
		
		
	}
	
}