/* 
 * File:   motor_globals.h
 * Author: Logan
 *
 * Created on January 8, 2019, 2:29 PM
 */

#ifndef MOTOR_GLOBALS_H
#define	MOTOR_GLOBALS_H

// direction for motors
#define FORWARD_Middle1     0
#define FORWARD_Sides       1
#define FORWARD_Up          2
#define FORWARD_DownMid     3
#define TURN_LEFT           4
#define TURN_RIGHT          5
#define STOP                6

#define TICKS_PER_CM        76  // 1 in = 2.54 cm   // 1 ft = 30.48 cm ~ 30.5 cm
#define TP_DEGREE_L         7.25    // 90 deg = 652.5 ticks
#define TP_DEGREE_R         7.27    // 90 deg = 654.3 ticks

// Motor speeds
#define SPEED        2815

//unsigned int speeds[] = {SPEED_0, SPEED_1, SPEED_2, SPEED_3, SPEED_4, SPEED_5};

// For motor selection
#define LEFT  0
#define RIGHT 1

// For error correction
#define KP   5
#define KI   20
int integral = 0;

// For motion control
unsigned int distR = 0; // measured in ticks
unsigned int distL = 0;
unsigned int goalL = 0;
unsigned int goalR = 0;
unsigned int total_ticksL = 0;
unsigned int total_ticksR = 0;
unsigned int prev_cm = 0;

// Type definitions
#define ACTION      0
#define SENSOR      1

// Data inside incoming queue
struct motorQueueData
{
    char type;
    char action;
    unsigned int dist;
    char speed;
};

void sendMsgToMotorQ(struct motorQueueData msg);
void sendMsgToMotorQFromISR(struct motorQueueData msg);

struct pwmQueueData
{
    unsigned int dc;
    unsigned int dist;
    char action;
};

void sendMsgToMotor_R(struct pwmQueueData);
void sendMsgToMotor_L(struct pwmQueueData);

unsigned int getMotorR_DC();
unsigned int getMotorL_DC();

unsigned char getMotorAction();

unsigned int movement[] = {6,0,6,4,6,1,6,4,6,2,6,4,6,1,6,4,6,3,6,5,6,1,6,5,6,2,6,5,6,1,6,5,6,3,6,4,6,1,6,4,6,2,6,4,6,1,6,4,6,3,6};   //current = left,right,left  //based on actions

//6,0,6,4,6,1,6,4,6,2,6,4,6,1,6,4,6,3,6  for sing left
//6,0,6,5,6,1,6,5,6,2,6,5,6,1,6,5,6,3,6  for sing right


struct good_path
{
    char action;
};

struct bad_path
{
    char action;
};



#endif	/* MOTOR_GLOBALS_H */