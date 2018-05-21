/*
 * AP_MoorDraw_analog.cpp
 *
 *  Created on: 2018Äê4ÔÂ26ÈÕ
 *      Author: luokai
 */



#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "MoorDraw.h"
#include "AP_MoorDraw_analog.h"
extern const AP_HAL::HAL& hal;


AP_MoorDraw_analog::AP_MoorDraw_analog(MoorDraw::MoorDraw_State &_state) :
    AP_MoorDraw_Backend(_state)
{
    source = hal.analogin->channel(_state.pin);
    if (source == nullptr) {
        // failed to allocate a ADC channel? This shouldn't happen
        set_status(MoorDraw::MoorDraw_NotConnected);
        return;
    }
    source->set_stop_pin((uint8_t)_state.stop_pin);
    source->set_settle_time((uint16_t)_state.settle_time_ms);
    set_status(MoorDraw::MoorDraw_NoData);
}

/*
*/
bool AP_MoorDraw_analog::detect(MoorDraw::MoorDraw_State &_state)
{
    if (_state.pin != -1) {
        return true;
    }
    return false;
}


/*
  update raw voltage state
 */
void AP_MoorDraw_analog::update_voltage(void)
{
   if (source == nullptr) {
       state.voltage_mv = 0;
       return;
   }
   // cope with changed settings
   source->set_pin(state.pin);
   source->set_stop_pin((uint8_t)state.stop_pin);
   source->set_settle_time((uint16_t)state.settle_time_ms);
   if (state.ratiometric) {
       state.voltage_mv = source->voltage_average_ratiometric() * 1000U;
   } else {
       state.voltage_mv = source->voltage_average() * 1000U;
   }
}

/*
  update distance_cm
 */
void AP_MoorDraw_analog::update(void)
{
    update_voltage();
    float v = state.voltage_mv * 0.001f;
    float dist_m = 0;
    float scaling = state.scaling;
    float offset  = state.offset;
    MoorDraw::MoorDraw_Function function = (MoorDraw::MoorDraw_Function)state.function.get();
    int16_t _max_distance_cm = state.max_distance_cm;

    switch (function) {
    case MoorDraw::FUNCTION_LINEAR:
        dist_m = (v - offset) * scaling;
        break;

    case MoorDraw::FUNCTION_INVERTED:
        dist_m = (offset - v) * scaling;
        break;

    case MoorDraw::FUNCTION_HYPERBOLA:
        if (v <= offset) {
            dist_m = 0;
        }
        dist_m = scaling / (v - offset);
        if (isinf(dist_m) || dist_m > _max_distance_cm * 0.01f) {
            dist_m = _max_distance_cm * 0.01f;
        }
        break;
    }
    if (dist_m < 0) {
        dist_m = 0;
    }
    state.distance_cm = dist_m * 100.0f;

    // update range_valid state based on distance measured
    update_status();
}

