/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

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

#include "app.h"
#include "debug.h"
#include "portmacro.h"

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

APP_DATA appData;

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
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    initMotors();
    
    DRV_TMR0_Start();
    DRV_TMR1_Start();
    DRV_TMR2_Start();

    dbgInit();          //initialize the debug 
    DRV_ADC_Open();     //open ADC
    
    create_sensor_queue();   //create the sensor queue

}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    DRV_ADC_Start();    //start the ADC driver
    DRV_TMR0_Start();   //start the timer driver
    
    int countF1 = 0;
    int countF2 = 0;
    int countL = 0;
    int countR = 0;
    int countX = 0;
    
    switch(drive_state) {
        
        case Forward_1:
            //go forward first
            //call function to do so
            if(countF1 == 0){
                countF1++;
                drive_state = Left_turn;
            }
            else if(countF1 == 1){
                countX++;
                countF1++;
                drive_state = Right_turn;
            }
            else if(countF1 == 2){
                countX++;
                countF1++;
                drive_state = Left_turn;
            }
            else if(countF1 == 3){
                countX++;
                countF1++;
                drive_state = Right_turn;
            }
            else{
                drive_state = STOP;
            }
            
            break;
        case Forward_2:
            //go forward
            //call function to do so
            if(countX == 0 || countX == 2){
                if(countF2 > 3){
                    countF2++;
                    drive_state = Left_turn;
                }
            }
            else if(countX == 1 || countX == 3){
                if(countF2 > 3){
                    countF2++;
                    drive_state = Right_turn;
                }
            }
            else {
                countF2 = 0;
            }
            
            break;
        case Left_turn:
            
            //turn left 90 deg
            //call function to do so
            
            if(countL > 3){
               countL++;
               drive_case = Forward_2; 
            }
            else {
                countL = 0;
                countF2 = 0;
                drive_case = Forward_1;
            }

            break;
        case Right_turn:
                        
            //turn right 90 deg
            //call function to do so
            
            if(countR > 3){
               countR++;
               drive_case = Forward_2; 
            }
            else {
                countR = 0;
                countF2 = 0;
                drive_case = Forward_1;
            }
            
            break;
        case Stop:
            
            //set everything to zero
            //call function to do so
            DRV_OC0_PulseWidthSet(0);
            DRV_OC1_PulseWidthSet(0);
            
            break;
        default:
            
            break;                  
                  
    }
}


/*******************************************************************************
 End of File
 */
