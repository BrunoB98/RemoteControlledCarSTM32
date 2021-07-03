/*
 * stepper.h
 *
 *  Created on: Apr 23, 2021
 *      Author: bruno
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#define STEP_DIR_INIT 1
#define STEP_RPM_INIT 8
#define stepsperrev 4096

void delayStepM (uint16_t us);

void stepper_set_rpm (int rpm);

void stepper_half_drive (int step);

void stepper_step_angle (float angle, int direction, int rpm);

#endif /* INC_STEPPER_H_ */
