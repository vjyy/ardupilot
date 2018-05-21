/*
 * MoorDraw_Backend.cpp
 *
 *  Created on: 2018Äê4ÔÂ13ÈÕ
 *      Author: luokai
 */




#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "MoorDraw.h"
#include "MoorDraw_backend.h"
extern const AP_HAL::HAL& hal;

/*
  base class constructor.
  This incorporates initialisation as well.
*/
AP_MoorDraw_Backend::AP_MoorDraw_Backend(MoorDraw::MoorDraw_State &_state) :
        state(_state)
{
    _sem = hal.util->new_semaphore();
}

// update status based on distance measurement
void AP_MoorDraw_Backend::update_status()
{
    // check distance
    if ((int16_t)state.distance_cm > state.max_distance_cm) {
        set_status(MoorDraw::MoorDraw_OutOfRangeHigh);
    } else if ((int16_t)state.distance_cm < state.min_distance_cm) {
        set_status(MoorDraw::MoorDraw_OutOfRangeLow);
    } else {
        set_status(MoorDraw::MoorDraw_Good);
    }
}

// set status and update valid count
void AP_MoorDraw_Backend::set_status(MoorDraw::MoorDraw_Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == MoorDraw::MoorDraw_Good) {
        if (state.draw_valid_count < 10) {
            state.draw_valid_count++;
        }
    } else {
        state.draw_valid_count = 0;
    }
}

/*
  set pre-arm checks to passed if the range finder has been exercised through a reasonable range of movement
      max distance sensed is at least 50cm > min distance sensed
      max distance < 200cm
      min distance sensed is within 10cm of ground clearance or sensor's minimum distance
 */
void AP_MoorDraw_Backend::update_pre_arm_check()
{
    // return immediately if already passed or no sensor data
    if (state.pre_arm_check || state.status == MoorDraw::MoorDraw_NotConnected || state.status == MoorDraw::MoorDraw_NoData) {
        return;
    }

    // update min, max captured distances
    state.pre_arm_distance_min = MIN(state.distance_cm, state.pre_arm_distance_min);
    state.pre_arm_distance_max = MAX(state.distance_cm, state.pre_arm_distance_max);

    // Check that the range finder has been exercised through a realistic range of movement
    if (((state.pre_arm_distance_max - state.pre_arm_distance_min) > MOORDRAW_PREARM_REQUIRED_CHANGE_CM) &&
         (state.pre_arm_distance_max < MOORDRAW_PREARM_ALT_MAX_CM) &&
         ((int16_t)state.pre_arm_distance_min < (MAX(state.ground_distance_cm,state.min_distance_cm) + 10)) &&
         ((int16_t)state.pre_arm_distance_min > (MIN(state.ground_distance_cm,state.min_distance_cm) - 10))) {
        state.pre_arm_check = true;
    }
}
