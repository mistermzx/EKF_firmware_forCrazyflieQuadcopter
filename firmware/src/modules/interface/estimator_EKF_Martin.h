/*
 * estimator_EKF_Martin.h
 *
 *  Created on: Aug 14, 2018
 *      Author: bitcraze
 */
#pragma once

#ifndef SRC_MODULES_INTERFACE_ESTIMATOR_EKF_MARTIN_H_
#define SRC_MODULES_INTERFACE_ESTIMATOR_EKF_MARTIN_H_

#include <stdint.h>
#include "stabilizer_types.h"

void EKFUpdate (state_t *state, sensorData_t *sensors, int16_t  PWM[]);

bool stateEstimatorEnqueuePosition(positionMeasurement_t *pos);

#endif /* SRC_MODULES_INTERFACE_ESTIMATOR_EKF_MARTIN_H_ */
