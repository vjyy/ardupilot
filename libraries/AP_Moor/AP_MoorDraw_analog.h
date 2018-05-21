#pragma once

#include "MoorDraw.h"
#include "MoorDraw_Backend.h"

class AP_MoorDraw_analog : public AP_MoorDraw_Backend
{
public:
    // constructor
    AP_MoorDraw_analog(MoorDraw::MoorDraw_State &_state);

    // static detection function
    static bool detect(MoorDraw::MoorDraw_State &_state);

    // update state
    void update(void);

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:
    // update raw voltage
    void update_voltage(void);

    AP_HAL::AnalogSource *source;
};
