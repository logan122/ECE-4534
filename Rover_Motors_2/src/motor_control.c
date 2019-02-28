#include "motor_control.h"
#include "motor_globals.h"
#include "portmacro.h"


MOTOR_CONTROL_DATA motor_controlData;
int count = 0;

void generateActionItems(struct motorQueueData data, struct pwmQueueData * left, struct pwmQueueData * right)
{
    left->action  = data.action;
    right->action = data.action;

    switch (data.action)
    {
        case FORWARD_Middle1:
            left->dc    = SPEED;
            left->dist  = 47 * TICKS_PER_CM; 
            right->dc   = SPEED;
            right->dist = 47 * TICKS_PER_CM;   
            break;
            
        case FORWARD_Sides:
            left->dc    = SPEED;
            left->dist  = 26 * TICKS_PER_CM;  
            right->dc   = SPEED;  
            right->dist = 26 * TICKS_PER_CM;  
            break;
            
        case FORWARD_Up:
            left->dc    = SPEED;
            left->dist  = 47 * TICKS_PER_CM;  
            right->dc   = SPEED;  
            right->dist = 47 * TICKS_PER_CM;  
            break;
            
        case FORWARD_DownMid:
            left->dc    = SPEED;
            left->dist  = 41 * TICKS_PER_CM;
            right->dc   = SPEED;
            right->dist = 41 * TICKS_PER_CM;
            break;

        case TURN_LEFT: //90 degrees left
            left->dc    = 0;
            left->dist  = 0;
            right->dc   = SPEED;
            right->dist = 18.5 * TP_DEGREE_L;
            break;
            
        case TURN_RIGHT: //90 degrees right
            left->dc    = SPEED;
            left->dist  = 18.5 * TP_DEGREE_R;
            right->dc   = 0;
            right->dist = 0;
            break;
            
        case STOP:
            left->dc    = 0;
            left->dist  = 0;
            right->dc   = 0;
            right->dist = 0;
            break;
            
        default:
            // handle and error
            break;
    }
}

void initMotors()
{
    // Default directions to forward
    // Motor 1 direction 0x4000
    TRISCCLR = MOTOR_RIGHT_DIR_PIN;
    ODCCCLR  = MOTOR_RIGHT_DIR_PIN;
    LATCCLR  = MOTOR_RIGHT_DIR_PIN;

    // motor 2 direction 0x0002
    TRISGCLR = MOTOR_LEFT_DIR_PIN;
    ODCGCLR  = MOTOR_LEFT_DIR_PIN;
    LATGCLR  = MOTOR_LEFT_DIR_PIN;

    T2CONSET = 0x0070;          // set for 256 prescale
    PR2      = TMR2_PERIOD;     // set the period to 3125

    /* initialize motors to OFF (duty cycle = 0%) */
    OC1CON = 0x0000;            // turn off while setting up
    OC1R   = 0x0000;            // primary compare register
    OC1RS  = 0x0000;            // secondary compare register
    OC1CON = 0x0006;            // configure for PWM mode

    OC2CON = 0x0000;            // turn off while setting up
    OC2R   = 0x0000;            // primary compare register
    OC2RS  = 0x0000;            // secondary compare register
    OC2CON = 0x0006;            // configure for PWM mode

    T2CONSET  = 0x8000;         // enable timer 2
    OC1CONSET = 0x8000;         // enable OC1
    OC2CONSET = 0x8000;         // enable OC2
}

void setMotorR_DC(unsigned int dc)
{
    OC1RS = dc;
    motor_controlData.dcR = dc;
}

void setMotorL_DC(unsigned int dc)
{
    OC2RS = dc;
    motor_controlData.dcL = dc;
}

unsigned int getMotorR_DC()
{
    return motor_controlData.dcR;
}

unsigned int getMotorL_DC()
{
    return motor_controlData.dcL;
}

unsigned char getMotorAction()
{
    unsigned int movement[] = {6,0,6,4,6,1,6,4,6,2,6,4,6,1,6,4,6,3,6,5,6,1,6,5,6,2,6,5,6,1,6,5,6,3,6,4,6,1,6,4,6,2,6,4,6,1,6,4,6,3,6};
    
    motor_controlData.action = movement[count];
    count++;
            
    return motor_controlData.action;
}

void setMotorR_Fwd()
{
    LATCCLR  = MOTOR_RIGHT_DIR_PIN;
}

void setMotorL_Fwd()
{
    LATGCLR  = MOTOR_LEFT_DIR_PIN;
}

void sendMsgToMotorQ(struct motorQueueData msg)
{
    xQueueSendToBack(motor_q, &msg, portMAX_DELAY);
}

void sendMsgToMotorQFromISR(struct motorQueueData msg)
{
    xQueueSendToBackFromISR(motor_q, &msg, NULL);
}

void sendMsgToMotor_R(struct pwmQueueData msg)
{
     xQueueSendToBack(right_q, &msg, portMAX_DELAY);
}

void sendMsgToMotor_L(struct pwmQueueData msg)
{
     xQueueSendToBack(left_q, &msg, portMAX_DELAY);
}

void motorR_recvQInISR(struct pwmQueueData* msg)
{
    xQueueReceiveFromISR(right_q, msg, NULL);
}

void motorL_recvQInISR(struct pwmQueueData* msg)
{
    xQueueReceiveFromISR(left_q, msg, NULL);
}

QueueHandle_t sensor_values;

void create_sensor_value_queue()
{
    sensor_values = xQueueCreate(100, sizeof(char));
    //dbgOutputVal(17);
}

void s_valsQueueSend(char s_vals)
{   
    //dbgOutputVal(20);
    xQueueSendToBack(sensor_values, &s_vals, portMAX_DELAY); 
}

char sValsQueueReceiveFromISR(BaseType_t *pxHigherPriorityTaskWoken)
{
    char t_char;
    xQueueReceiveFromISR(sensor_values, &t_char, pxHigherPriorityTaskWoken);
    return t_char;
}

char sValsQueueReceive()
{
    char s_byte;
    xQueueReceive(sensor_values, &s_byte, portMAX_DELAY);
    return s_byte;
}

sens_message sensorQueue()
{
    sens_message val;
    xQueueReceive(sensor_values, &val, portMAX_DELAY);
    return val;
}

bool rightQIsEmpty()
{
    return xQueueIsQueueEmptyFromISR(right_q);
}

bool leftQIsEmpty()
{
    return xQueueIsQueueEmptyFromISR(left_q);
}

void stop_hold(void){
    int z = 0;
    for(z = 0; z < 1800000000; z++);
}

void readFromQandSetPins(unsigned char motor)
{
    struct pwmQueueData data;
    switch (motor)
    {
    case LEFT:
        motorL_recvQInISR(&data);
        setMotorL_DC(data.dc);
        motor_controlData.action = data.action;
        goalL = data.dist;
        break;
    case RIGHT:
        motorR_recvQInISR(&data);
        setMotorR_DC(data.dc);
        motor_controlData.action = data.action;
        goalR = data.dist;
        break;
    default:
        break;
    }
}

void MOTOR_CONTROL_Initialize ( void )
{
    motor_controlData.state = MOTOR_CONTROL_STATE_INIT;
    motor_controlData.action = STOP;
    // Incoming queue
    motor_q = xQueueCreate(32, sizeof (struct motorQueueData));
    right_q = xQueueCreate(32, sizeof (struct pwmQueueData));
    left_q  = xQueueCreate(32, sizeof (struct pwmQueueData));
    // Initialize the OCs and Timer2
    initMotors();

    // Start the encoder counters
    DRV_TMR0_Start();
    DRV_TMR1_Start();
    DRV_TMR2_Start();
}

void handleIncomingMsg(struct motorQueueData data)
{
    
    switch (data.type)
    {
        case ACTION:
        {
            struct motorQueueData out_m;
            out_m.action = data.action;
            out_m.dist   = data.dist;
            out_m.speed  = data.speed;
         
            struct pwmQueueData left, right;

            generateActionItems(out_m, &left, &right);

            sendMsgToMotor_L(left);
            sendMsgToMotor_R(right);
            break;
        }
        case SENSOR:
        {
            struct motorQueueData out_m;
            out_m.action = data.action;
            out_m.dist   = data.dist;
            out_m.speed  = data.speed;
         
            struct pwmQueueData left, right;

            generateActionItems(out_m, &left, &right);

            sendMsgToMotor_L(left);
            sendMsgToMotor_R(right);
            break;
        }
        default:
        {
            break;
        }
    }
}

void MOTOR_CONTROL_Tasks ( void )
{
    switch ( motor_controlData.state )
    {
        case MOTOR_CONTROL_STATE_INIT:
        {
            bool appInitialized = true;
            if (appInitialized)
            {
                motor_controlData.state = MOTOR_CONTROL_HANDLE_INCOMING;
            }
            break;
        }

        case MOTOR_CONTROL_HANDLE_INCOMING:
        {
            //dbgOutputLoc(MOTOR_THREAD_WAIT);

            struct motorQueueData rec;
            if(xQueueReceive(motor_q, &rec, portMAX_DELAY))
            {
                //dbgOutputLoc(MOTOR_THREAD_RECVD);
                
                
                handleIncomingMsg(rec); 
                
                
            }
            break;
        }

        default:
        {
            break;
        }
    }
}

