#include "debug.h"

int DEBUG_INIT_CALLED = 0;

void dbgInit(){ 
    // ***Configure the Max32 board for respective outputs***
    // **Useful resource: http://microchipdeveloper.com/harmony:plib-ports ** 
    
    //set 1 of 8 output pins
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_9);   //53
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8);   //51
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11);  //49
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_6);   //47
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_0);   //45
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8);   //43
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_13);  //41
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_5);   //39
    
    //set 2 of 8 output pins
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0);   //37
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2);   //35
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4);   //33
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6);   //31
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_7);   //29
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_2);   //27
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3);   //25
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_3);   //23
}

void dbgOutputVal(unsigned int outVal){
 
    if(outVal > MAXVAL) return;
    
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, (outVal&0x40)>>6);  //51
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, (outVal&0x20)>>5); //49
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_6, (outVal&0x10)>>4);  //47
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_0, (outVal&0x08)>>3);  //45
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_13, (outVal&0x04)>>2);  //41
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_5, (outVal&0x02)>>1); //39
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0, (outVal&0x01));     //37
    
    // Toggle pin 53
    PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_9);
    
}
void dbgUARTVal(unsigned char outVal){
    //void DRV_USART0_WriteByte(const uint8_t byte)
    //DRV_USART0_WriteByte(outVal);
}

void dbgOutputLoc(unsigned int outVal){
    // Verify that outVal is either less than or equal to 127.
    if(outVal > MAXVAL) return;
    
    // Ensure that the necessary pins are set as outputs.
    if(DEBUG_INIT_CALLED == 0){
        dbgInit();
        DEBUG_INIT_CALLED = 1;
    }
    
    // Write to board pins 35:-2:23.
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2, (outVal&0x40)>>6);  //35
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_4, (outVal&0x20)>>5);  //33
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6, (outVal&0x10)>>4);  //31
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_7, (outVal&0x08)>>3);  //29
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_2, (outVal&0x04)>>2);  //27
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3, (outVal&0x02)>>1);  //25
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_3, (outVal&0x01));     //23
    
    // Toggle pin 37
    //PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_0);
    
}

void dbgHaltAll(unsigned int outVal){
    // -> Set Pins 13 as an output for the LED.
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
    
    // -> Set the LED to alert the user that the program has stopped. 
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3, 1);
    
    // -> Call dbgOutputLoc here:
    dbgOutputLoc(outVal);
    
    int timer = 0;
    while(1){
        timer = timer + 1;
        
        // Toggle the LED.
        if(timer == 2500000){
            PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_3);
            timer = 0;
        }
    }
}


