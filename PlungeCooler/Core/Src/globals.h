/*
 * globals.h
 *
 *  Created on: Oct 26, 2023
 *      Author: Leo
 */

#include "main.h"
#include "stm32h7xx_hal.h"


#ifndef SRC_GLOBALS_H_
#define SRC_GLOBALS_H_

#define MOVING_AVG_LENGTH 200
#define TIMER_FREQ 96000000 	// in Hz

#define LOG_SIZE 30000
#define LOGGING_TIMEBASE 0.02		// in ms, how often a position is logged and drop calculation made
#define CLOCKS_PER_LOG (LOGGING_TIMEBASE * TIMER_FREQ / 1000)

#define US_TO_TICKS TIMER_FREQ/(1000*1000) //multiply by this to convert us to number of timer ticks
#define DROP_SPEED 5000			// in mm/s, the speed that the drop moves at
#define TARGET_DIST_ORTH 100	// in mm, horizontal distance between dispense tip and target
#define DISPENSE_LATENCY 56		// in us, latency between signal output and drop being dispensed

#define POST_DISP_LOG_SLOW_FACTOR 4 	// factor by which to slow down the logging rate after dispense since it is less essential
extern uint32_t dispense_delay_clocks; 	// in ticks, time between dispensing signal and hitting target
										// this is a function of latency(constant), drop speed and target distance
										// target distance is a function of tilt angle
extern uint16_t posLog[LOG_SIZE];	// log of all position poins taken. will be transmitted over UART after
extern uint32_t log_position;		//
extern uint32_t running_sum;		// used to keep a running sum for moving average speed estimate
extern uint32_t timepoint_pos; 		// the tick where you want the drop to intersect the target
extern uint8_t  disp_flag;			// flagged when its time to dispense
extern uint32_t dispense_pos;
extern uint32_t posn_braked_at;
extern uint8_t  plunge_done_flag;
extern uint8_t  DEPOSITED;
extern uint32_t next_next_pos;
extern uint32_t prev_pos;
extern double speed;
extern uint32_t clocks_to_disp;
void dispense(void);

#endif /* SRC_GLOBALS_H_ */
