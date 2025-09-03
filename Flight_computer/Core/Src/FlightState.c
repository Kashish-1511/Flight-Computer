#include "FlightState.h"
#include <math.h>
#include <stdio.h>

// -------- Utility: Reset counters --------
static void reset_counters(flight_fsm_t *fsm) {
    for (int i = 0; i < 5; i++) fsm->counters[i] = 0;
}

// -------- Utility: Change state --------
static void change_state(flight_fsm_t *fsm, flight_state_e new_state, flight_event_e event) {
    if (fsm->current_state != new_state) {
        printf("[FSM] %d → %d (Event %d)\n", fsm->current_state, new_state, event);
        fsm->current_state = new_state;
        fsm->state_changed = true;
        reset_counters(fsm);
    }
}

// -------- Initialization --------
void flight_fsm_init(flight_fsm_t *fsm) {
    fsm->current_state = FLIGHT_READY;
    fsm->state_changed = false;
    reset_counters(fsm);
}

// -------- Update logic --------
void flight_fsm_update(flight_fsm_t *fsm, const vec3f_t *acc, const state_estimate_t *est,
                       const flight_settings_t *settings, bool launch_pin_high) {

    fsm->state_changed = false; // default

    switch (fsm->current_state) {
        case FLIGHT_READY: {
            // Condition 1: Launch pin
            if (launch_pin_high) {
                change_state(fsm, FLIGHT_THRUSTING, EV_LIFTOFF);
                break;
            }
            // Condition 2: Acc magnitude
            float acc_mag_sq = acc->x * acc->x + acc->y * acc->y + acc->z * acc->z;
            if (acc_mag_sq > settings->liftoff_acc_threshold * settings->liftoff_acc_threshold) {
                if (++fsm->counters[0] > settings->liftoff_safety_iterations) {
                    change_state(fsm, FLIGHT_THRUSTING, EV_LIFTOFF);
                }
            } else fsm->counters[0] = 0;
        } break;

        case FLIGHT_THRUSTING: {
            if (est->acceleration < 0) {
                if (++fsm->counters[1] > settings->coasting_safety_iterations) {
                    change_state(fsm, FLIGHT_COASTING, EV_MAX_VELOCITY);
                }
            } else fsm->counters[1] = 0;
        } break;

        case FLIGHT_COASTING: {
            if (est->velocity < 0) {
                if (++fsm->counters[2] > settings->apogee_safety_iterations) {
                    change_state(fsm, FLIGHT_DROGUE, EV_APOGEE);
                }
            } else fsm->counters[2] = 0;
        } break;

        case FLIGHT_DROGUE: {
            if (est->altitude < settings->main_deploy_altitude) {
                if (++fsm->counters[3] > settings->main_deploy_safety_iterations) {
                    change_state(fsm, FLIGHT_MAIN, EV_MAIN_DEPLOY);
                }
            } else fsm->counters[3] = 0;
        } break;

        case FLIGHT_MAIN: {
            if (fabsf(est->velocity) < settings->touchdown_velocity_threshold) {
                if (++fsm->counters[4] > settings->touchdown_safety_iterations) {
                    change_state(fsm, FLIGHT_TOUCHDOWN, EV_TOUCHDOWN);
                }
            } else fsm->counters[4] = 0;
        } break;

        case FLIGHT_TOUCHDOWN:
        default:
            break;
    }
}
