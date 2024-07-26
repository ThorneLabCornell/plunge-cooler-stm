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

#define LOG_SIZE 100000
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
extern uint16_t posLog[LOG_SIZE];
extern uint16_t thermoLog[LOG_SIZE];	// log of all position poins taken. will be transmitted over UART after
extern uint32_t log_position;		



#endif /* SRC_GLOBALS_H_ */
