/*
 * MoorDraw.cpp
 *
 *  Created on: 2018Äê4ÔÂ2ÈÕ
 *      Author: luokai
 */


#include "MoorDraw.h"
#include "MoorDraw_Backend.h"
#include "AP_MoorDraw_analog.h"
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo MoorDraw::var_info[]={
        // @Param: _TYPE
        // @DisplayName: MoorDraw type
        // @Description: What type of MoorDraw device that is connected
        // @Values: 0:None,1:Analog
        // @User: Standard
        AP_GROUPINFO("_TYPE",    0, MoorDraw, state[0].type, 0),
        // @Param: _PIN
        // @DisplayName: MoorDraw pin
        // @Description: Analog pin that MoorDraw is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
        // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
        // @User: Standard
        AP_GROUPINFO("_PIN",     1, MoorDraw, state[0].pin, -1),
        AP_GROUPINFO("_SCALING", 2, MoorDraw, state[0].scaling, 3.0f),
        // @Param: _OFFSET
        // @DisplayName: MoorDraw offset
        // @Description: Offset in volts for zero distance for analog MoorDraw. Offset added to distance in centimeters for PWM and I2C Lidars
        // @Units: V
        // @Increment: 0.001
        // @User: Standard
        AP_GROUPINFO("_OFFSET",  3, MoorDraw, state[0].offset, 0.0f),

        // @Param: _FUNCTION
        // @DisplayName: MoorDraw function
        // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
        // @Values: 0:Linear,1:Inverted,2:Hyperbolic
        // @User: Standard
        AP_GROUPINFO("_FUNCTION", 4, MoorDraw, state[0].function, 0),

        // @Param: _MIN_CM
        // @DisplayName: Rangefinder minimum distance
        // @Description: Minimum distance in centimeters that rangefinder can reliably read
        // @Units: cm
        // @Increment: 1
        // @User: Standard
        AP_GROUPINFO("_MIN_KG",  5, MoorDraw, state[0].min_pulling_kg, 20),

        // @Param: _MAX_CM
        // @DisplayName: Rangefinder maximum distance
        // @Description: Maximum distance in centimeters that rangefinder can reliably read
        // @Units: cm
        // @Increment: 1
        // @User: Standard
        AP_GROUPINFO("_MAX_KG",  6, MoorDraw, state[0].max_pulling_kg, 700),

        AP_GROUPINFO("_MAX_CM",7,MoorDraw,state[0].max_distance_cm,20000),
        AP_GROUPINFO("_MIN_CM",8,MoorDraw,state[0].min_distance_cm,50),
        AP_GROUPINFO("_GROUND_CM",9,MoorDraw,state[0].ground_distance_cm,10),

        // @Param: _RMETRIC
        // @DisplayName: Ratiometric
        // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
        // @Values: 0:No,1:Yes
        // @User: Standard
        AP_GROUPINFO("_RMETRIC", 10, MoorDraw, state[0].ratiometric, 1),
        AP_GROUPINFO("_SETTLE",11,MoorDraw,state[0].settle_time_ms,0),
        AP_GROUPINFO("_POS", 49,MoorDraw,state[0].pos_offset, 0.0f),

        AP_GROUPEND
};

MoorDraw::MoorDraw(void) :
            num_instances(0),
            estimated_terrain_height(0)
{
    AP_Param::setup_object_defaults(this, var_info);
    //ch=hal.analogin->channel(state.pin);
    //ch->set_stop_pin(state.stop_pin);
    //ch->set_settle_time(state.settle_time_ms);
    //

}

void MoorDraw::init(void){
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<MOORDRAW_MAX_INSTANCES; i++) {
        detect_instance(i);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
        // initialise pre-arm check variables
        state[i].pre_arm_check = false;
        state[i].pre_arm_distance_min = 9999;  // initialise to an arbitrary large value
        state[i].pre_arm_distance_max = 0;

        // initialise status
        state[i].status = MoorDraw_NotConnected;
        state[i].draw_valid_count = 0;
    }
}
void MoorDraw::update(void){
    /*ch->set_stop_pin(state.stop_pin);
    ch->set_settle_time(state.settle_time_ms);
    if (ch == nullptr) {
        state.voltage_mv = 0;
        return;
    }
    if (state.ratiometric) {
        state.voltage_mv = ch->voltage_average_ratiometric() * 1000U;
    } else {
        state.voltage_mv = ch->voltage_average() * 1000U;
    }
    hal.console->printf("[%u %u]",(unsigned)state.pin,(double)state.voltage_mv);*/
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (state[i].type == MoorDraw_TYPE_NONE) {
                // allow user to disable a MoorDraw at runtime
                state[i].status = MoorDraw_NotConnected;
                state[i].draw_valid_count = 0;
                continue;
            }
            drivers[i]->update();
            drivers[i]->update_pre_arm_check();
        }
    }

};

// Handle an incoming DISTANCE_SENSOR message (from a MAVLink enabled range finder)
void MoorDraw::handle_msg(mavlink_message_t *msg){
    return;
}
uint16_t MoorDraw::distance_cm() const {
    return 0;
}
uint16_t MoorDraw::voltage_mv() const {

    return state[0].voltage_mv;
}
int16_t MoorDraw::max_distance_cm() const{
    return state[0].max_distance_cm;
}
int16_t MoorDraw::min_distance_cm() const{
    return state[0].min_distance_cm;
}
int16_t MoorDraw::ground_clearance_cm() const {
    return state[0].ground_distance_cm;
}
MoorDraw::MoorDraw_Status MoorDraw::status() const{
    return state[0].status;
}
bool MoorDraw::has_data() const{
    AP_MoorDraw_Backend *backend = get_backend(0);
    if (backend == nullptr) {
        return false;
    }
    return backend->has_data();
}
uint8_t MoorDraw::draw_valid_count() const{
    AP_MoorDraw_Backend *backend = get_backend(0);
    if (backend == nullptr) {
        return 0;
    }
    return backend->draw_valid_count();
}
const Vector3f &MoorDraw::get_pos_offset() const {
    return  pos_offset_zero;
}
// return true if we have a range finder with the specified orientation
// find first range finder instance with the specified orientation
AP_MoorDraw_Backend *MoorDraw::find_instance(enum Rotation orientation) const{
            return nullptr;
}

AP_MoorDraw_Backend *MoorDraw::get_backend(uint8_t id) const{
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == MoorDraw_TYPE_NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
}
bool MoorDraw::pre_arm_check() const{
    for (uint8_t i=0; i<num_instances; i++) {
        // if driver is valid but pre_arm_check is false, return false
        if ((drivers[i] != nullptr) && (state[i].type != MoorDraw_TYPE_NONE) && !state[i].pre_arm_check) {
            return false;
        }
    }
    return true;
}

void MoorDraw::detect_instance(uint8_t instance){
    enum MoorDraw_Type _type = (enum MoorDraw_Type)state[instance].type.get();
    switch (_type) {
    case MoorDraw_TYPE_ANALOG:
            // note that analog will always come back as present if the pin is valid
            if (AP_MoorDraw_analog::detect(state[instance])) {
                state[instance].instance = instance;
                drivers[instance] = new AP_MoorDraw_analog(state[instance]);
            }
            break;
    case MoorDraw_TYPE_I2C:
    case MoorDraw_TYPE_NONE:
        break;

    }
}
void MoorDraw::update_instance(uint8_t instance){
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (state[i].type == MoorDraw_TYPE_NONE) {
                // allow user to disable a MoorDraw at runtime
                state[i].status = MoorDraw_NotConnected;
                state[i].draw_valid_count = 0;
                continue;
            }
            drivers[i]->update();
            drivers[i]->update_pre_arm_check();
        }
    }
}

bool MoorDraw::_add_backend(AP_MoorDraw_Backend *backend){
    if (!backend) {
        return false;
    }
    if (num_instances == MOORDRAW_MAX_INSTANCES) {
        AP_HAL::panic("Too many MoorDraw backends");
    }

    drivers[num_instances++] = backend;
    return true;
}
