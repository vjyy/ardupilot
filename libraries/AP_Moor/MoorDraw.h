/*
 * Moor_Draw.h
 *
 *  Created on: 2018Äê3ÔÂ29ÈÕ
 *      Author: luokai
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#include <AP_SerialManager/AP_SerialManager.h>



#define MOORDRAW_MAX_INSTANCES 2
#define MOORDRAW_GROUND_CLEARANCE_CM_DEFAULT 10
#define MOORDRAW_PREARM_ALT_MAX_CM           200
#define MOORDRAW_PREARM_REQUIRED_CHANGE_CM   50
class AP_MoorDraw_Backend;
class MoorDraw
{
public:
    friend class AP_MoorDraw_Backend;
    MoorDraw();

    enum MoorDraw_Type {
        MoorDraw_TYPE_NONE   = 0,
        MoorDraw_TYPE_ANALOG = 1,
        MoorDraw_TYPE_I2C  = 2,
    };
    enum MoorDraw_Function {
        FUNCTION_LINEAR    = 0,
        FUNCTION_INVERTED  = 1,
        FUNCTION_HYPERBOLA = 2
    };

    enum MoorDraw_Status {
        MoorDraw_NotConnected = 0,
        MoorDraw_NoData,
        MoorDraw_OutOfRangeLow,
        MoorDraw_OutOfRangeHigh,
        MoorDraw_Good
    };
    struct MoorDraw_State {
        uint8_t                instance;    // the instance number of this RangeFinder
        uint16_t               pulling_kg; // distance: in kg
        uint16_t               voltage_mv;  // voltage in millivolts,
        uint16_t               distance_cm; // distance: in cm
                                            // if applicable, otherwise 0
        enum MoorDraw_Status status;     // sensor status
        uint8_t                draw_valid_count;   // number of consecutive valid readings (maxes out at 10)
        bool                   pre_arm_check;   // true if sensor has passed pre-arm checks
        uint16_t               pre_arm_distance_min;    // min distance captured during pre-arm checks
        uint16_t               pre_arm_distance_max;    // max distance captured during pre-arm checks

        AP_Int8  type;
        AP_Int8  pin;
        AP_Int8  ratiometric;
        AP_Int8  stop_pin;
        AP_Int16 settle_time_ms;
        AP_Float scaling;
        AP_Float offset;
        AP_Int8  function;
        AP_Int16 min_distance_cm;
        AP_Int16 max_distance_cm;
        AP_Int16 ground_distance_cm;
        AP_Int16 min_pulling_kg;
        AP_Int16 max_pulling_kg;
        AP_Int8  ground_pulling_kg;
        AP_Int8  address;
        AP_Vector3f pos_offset; // position offset in body frame
        AP_Int8  orientation;
    };

    // parameters for each instance
    static const struct AP_Param::GroupInfo var_info[];

    void init(void);

    // update state of all MoorDraw. Should be called at around
    // 10Hz from main loop
    void update(void);

    // Handle an incoming DISTANCE_SENSOR message (from a MAVLink enabled range finder)
    void handle_msg(mavlink_message_t *msg);

    // return true if we have a range finder with the specified orientation
    uint16_t distance_cm() const;
    uint16_t voltage_mv() const;
    int16_t max_distance_cm() const;
    int16_t min_distance_cm() const;
    int16_t ground_clearance_cm() const;
    MoorDraw_Status status() const;
    bool has_data() const;
    uint8_t draw_valid_count() const;
    const Vector3f &get_pos_offset() const;

    // find first range finder instance with the specified orientation
    AP_MoorDraw_Backend *find_instance(enum Rotation orientation) const;

    AP_MoorDraw_Backend *get_backend(uint8_t id) const;
    bool pre_arm_check() const;

private:
    //AP_HAL::AnalogSource* ch;
    //static int8_t pin;
    Vector3f pos_offset_zero;
    MoorDraw_State state[MOORDRAW_MAX_INSTANCES];
    AP_MoorDraw_Backend *drivers[MOORDRAW_MAX_INSTANCES];
    uint8_t num_instances:1;
    float estimated_terrain_height;
    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);

    bool _add_backend(AP_MoorDraw_Backend *driver);
};

