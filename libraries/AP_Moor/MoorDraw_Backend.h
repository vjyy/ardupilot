/*
 * Moor_Backed.h
 *
 *  Created on: 2018Äê3ÔÂ29ÈÕ
 *      Author: luokai
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "MoorDraw.h"

class AP_MoorDraw_Backend
{
public:
    AP_MoorDraw_Backend(MoorDraw::MoorDraw_State &_state);
    virtual ~AP_MoorDraw_Backend(void){};
    // update the state structure
    virtual void update() = 0;

    virtual void handle_msg(mavlink_message_t *msg) { return; }

    void update_pre_arm_check();

    uint8_t instance() const { return state.instance; }
    uint16_t distance_kg() const { return state.pulling_kg; }
    uint16_t voltage_mv() const { return state.voltage_mv; }
    int16_t max_distance_kg() const { return state.max_pulling_kg; }
    int16_t min_distance_kg() const { return state.min_pulling_kg; }
    int16_t ground_clearance_kg() const { return state.ground_pulling_kg; }
    MAV_DISTANCE_SENSOR get_mav_distance_sensor_type() const {
        if (state.type == MoorDraw::MoorDraw_TYPE_NONE) {
            return MAV_DISTANCE_SENSOR_UNKNOWN;
        }
        return _get_mav_distance_sensor_type();
    }
    MoorDraw::MoorDraw_Status status() const {
        if (state.type == MoorDraw::MoorDraw_TYPE_NONE) {
            // turned off at runtime?
            return MoorDraw::MoorDraw_NotConnected;
        }
        return state.status;
    }
    MoorDraw::MoorDraw_Type type() const { return (MoorDraw::MoorDraw_Type)state.type.get(); }

    // true if sensor is returning data
    bool has_data() const {
        return ((state.status != MoorDraw::MoorDraw_NotConnected) &&
                (state.status != MoorDraw::MoorDraw_NoData));
    }

    // returns count of consecutive good readings
    uint8_t draw_valid_count() const { return state.draw_valid_count; }

    // return a 3D vector defining the position offset of the sensor
    // in metres relative to the body frame origin
    //const Vector3f &get_pos_offset() const { return state.pos_offset; }
protected:

    // update status based on distance measurement
    void update_status();

    // set status and update valid_count
    void set_status(MoorDraw::MoorDraw_Status status);

    MoorDraw::MoorDraw_State &state;

    // semaphore for access to shared frontend data
    AP_HAL::Semaphore *_sem;

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const = 0;
};
