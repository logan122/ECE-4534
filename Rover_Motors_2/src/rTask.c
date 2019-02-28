/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    rtask.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "rTask.h"
#include "motor_control.h"
#include "motor_globals.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void RTASK_Initialize ( void )

  Remarks:
    See prototype in rtask.h.
 */

void RTASK_Initialize ( void )
{
    create_r_queue(); //create the receive queue
}


/******************************************************************************
  Function:
    void RTASK_Tasks ( void )

  Remarks:
    See prototype in rtask.h.
 */

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) 
{
	if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0) 
    {
		return 0;
	}
	return -1;
}

int getIntFromKey(jsmntok_t key)
{
    unsigned int length = key.end - key.start;
    char keyString[length + 1];    
    memcpy(keyString, &JSON_STRING[key.start], length);
    keyString[length] = '\0';
    return atoi(keyString);
}

struct motorQueueData parseJSON (unsigned char rec[UART_RX_QUEUE_SIZE]) 
{
    struct motorQueueData out;
    out.type = ACTION;
    JSON_STRING = rec;
    int r;
    jsmn_parser p;
    jsmntok_t t[128]; /* We expect no more than 128 tokens */
    jsmn_init(&p);
    r = jsmn_parse(&p, JSON_STRING, strlen(JSON_STRING), t, sizeof(t)/sizeof(t[0]));
    
    /* Get the sequence ID number */
    int seq = getIntFromKey(t[2]);
    /* Is it the expected next sequence? */
    if (seq != prev_inc_seq + 1) // ERROR
    {
        char buf[128];
        sprintf(buf, STR_SEQUENCE_ERROR, outgoing_seq, prev_inc_seq+1, seq);
    }
    prev_inc_seq = seq;

    if(r == 9) // ACTION
    {
        out.action = getIntFromKey(t[4]);  //color red or green
    }
  
    else // ERROR
    {
        // Send message back to server something is wrong
        char buf[128];
        sprintf(buf, STR_JSON_ERROR, outgoing_seq);
    }
    return out;
}

//define the states of the state machine
typedef enum {
    BEGIN_STATE = 10,
    MIDDLE_STATE = 20,
    END_STATE = 30
}r_state;

static unsigned char message[100];

void RTASK_Tasks ( void )
{
    //dbgOutputVal(45);
    unsigned char r_byte;
    r_state state = BEGIN_STATE;  //set the starting state
    unsigned int count = 0;
    unsigned int color_count = 0;

    while(1)
    {
        r_byte = rQueueReceive(); //receive a byte from the receive queue
        
        if(state == BEGIN_STATE)            //looking for first brace
        {
            dbgOutputVal(BEGIN_STATE);
            
            if(r_byte == 123)        //if find first bracket, start message and go to next state
            {
                message[0] = r_byte;
                state = MIDDLE_STATE;
                count = 1;
            }
        }
        else if(state == MIDDLE_STATE)      //going through middle json
        {
            dbgOutputVal(MIDDLE_STATE);
            //message[count] = r_byte;
            count++;
            if(r_byte == 125)         //if find last brace, add to message and go to end state
            {  
                state = END_STATE;
            }
        }
        else if(state == END_STATE)         //if have finished json message, reset count and go to first state
        {
            dbgOutputVal(END_STATE);
            state = BEGIN_STATE;
        }
        if(state == END_STATE)
        {
            dbgOutputVal(END_STATE);
            state = BEGIN_STATE;
            //Logan - This is where the parser code goes as the message has been fully completed
            struct  motorQueueData out = parseJSON(rec);
            //sendMsgToMotorQ(out);
            if(color_count == 0){
                sendMsgToMotorQ(color[0]);
            }
            else if(color_count == 1){
                sendMsgToMotorQ(color[1]);
            }
            else if(color_count == 2){
                sendMsgToMotorQ(color[2]);
            }
            
        }
    }
    
   
}

 

/*******************************************************************************
 End of File
 */
