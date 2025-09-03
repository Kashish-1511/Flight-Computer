/*
 * FlghtState.h
 *
 *  Created on: Aug 9, 2025
 *      Author: ambal
 */

#ifndef INC_FLIGHTSTATE_H_
#define INC_FLIGHTSTATE_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f7xx_hal.h"
#include "arm_math.h"  // for float32_t

// ----------------- Flight States -----------------
typedef enum {
    FLIGHT_READY = 0,
    FLIGHT_THRUSTING,
    FLIGHT_COASTING,
    FLIGHT_DROGUE,
    FLIGHT_MAIN,
    FLIGHT_TOUCHDOWN
} flight_state_e;

// ----------------- Events -----------------
typedef enum {
    EV_NONE = 0,
    EV_LIFTOFF,
    EV_MAX_VELOCITY,
    EV_APOGEE,
    EV_MAIN_DEPLOY,
    EV_TOUCHDOWN
} flight_event_e;

// ----------------- FSM Structure -----------------
typedef struct {
    flight_state_e current_state;
    bool state_changed;
    uint32_t counters[5];   // For safety counters per phase
} flight_fsm_t;

// ----------------- Sensor & Settings -----------------
typedef struct {
    float32_t x, y, z;
} vec3f_t;

typedef struct {
    float32_t acceleration; // m/s²
    float32_t velocity;     // m/s
    float32_t altitude;     // m
} state_estimate_t;

typedef struct {
    float32_t liftoff_acc_threshold;
    uint16_t liftoff_safety_iterations;
    uint16_t coasting_safety_iterations;
    uint16_t apogee_safety_iterations;
    uint16_t main_deploy_safety_iterations;
    float32_t main_deploy_altitude;
    float32_t touchdown_velocity_threshold;
    uint16_t touchdown_safety_iterations;
} flight_settings_t;

// ----------------- Functions -----------------
void flight_fsm_init(flight_fsm_t *fsm);
void flight_fsm_update(flight_fsm_t *fsm, const vec3f_t *acc, const state_estimate_t *est,
                       const flight_settings_t *settings, bool launch_pin_high);



#endif /* INC_FLIGHTSTATE_H_ */
