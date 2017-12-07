#include "Copter.h"

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
/*
 * 修改计划：
 * 一、增加两种内部飞行状态
 * 1，起降状态
 *      用于解锁后到2米高度以前的飞行，不使用GPS，仅控制飞行姿态，整个过程为匀速上升过程，
 *      如果出现高度匀速下降则切换为降落状态，如果高度超过2米以上则切换为正常飞行状态，并
 *      触发收起起落架事件；
 * 2，悬停状态
 *      用于超过2米的正常飞行，使用GPS进行水平定位，如高度下降为1.5米以下则进入降落状态，
 *      并触发放下起落架事件；
 * 二、增加拉力油门控制
 *  1，最低油门值，即解锁后的油门值；
 *  2，通过AD转换获得拉力参数，并根据设置的换算比例值计算拉力；
 *  3，拉力与油门采用PID控制，需要增加PID参数值**
 * 三、增加线重补偿比例值
 *     线每上升10米高度所需的油门补偿值或拉力补偿值
 */
bool Copter::moor_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Loiter if the Rotor Runup is not complete
    if (!ignore_checks && !motors->rotor_runup_complete()){
        return false;
    }
#endif
    if(position_ok()){
        gcs().send_text(MAV_SEVERITY_WARNING,"position_ok");
    }else{
        gcs().send_text(MAV_SEVERITY_WARNING,"!position_ok");
    }
    if(ignore_checks){
        gcs().send_text(MAV_SEVERITY_WARNING,"ignore_checks");
    }else{
        gcs().send_text(MAV_SEVERITY_WARNING,"checks");
    }
    //设定飞行模式前强制检测GPS定位状态，如果不能定位则返回错误
    //
    if (position_ok() || ignore_checks) {

        // set target to current position
        wp_nav->init_loiter_target();

        // initialize vertical speed and acceleration
        /*pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control->set_accel_z(g.pilot_accel_z);
        */
        // initialise position and desired velocity
        /*if (!pos_control->is_active_z()) {
            pos_control->set_alt_target_to_current_alt();
            pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
        }*/
        pos_control->set_alt_target(0);//高度目标设置为0

        return true;
    }else{
        return false;
    }
}

#if PRECISION_LANDING == ENABLED
bool Copter::do_precision_moor()
{
    if (!_precision_loiter_enabled) {
        return false;
    }
    if (ap.land_complete_maybe) {
        return false;        // don't move on the ground
                            //降落后则禁止进行XY控制
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (wp_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }
    if (!precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void Copter::precision_moor_xy()
{
    wp_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel_rel;
    if (!precland.get_target_position_cm(target_pos)) {
        target_pos.x = inertial_nav.get_position().x;
        target_pos.y = inertial_nav.get_position().y;
    }
    if (!precland.get_target_velocity_relative_cms(target_vel_rel)) {
        target_vel_rel.x = -inertial_nav.get_velocity().x;
        target_vel_rel.y = -inertial_nav.get_velocity().y;
    }
    pos_control->set_xy_target(target_pos.x, target_pos.y);
    pos_control->override_vehicle_velocity_xy(-target_vel_rel);
}
#endif

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void Copter::moor_run()
{
    LoiterModeState loiter_state;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    //pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    //pos_control->set_accel_z(g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // process pilot's roll and pitch input
        wp_nav->set_pilot_desired_acceleration(channel_roll->get_control_in(), channel_pitch->get_control_in());

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        //target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        //target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        wp_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    //if (ap.land_complete_maybe) {
    //    wp_nav->loiter_soften_for_landing();
    //}

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif

    // Loiter State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        loiter_state = Loiter_MotorStopped;
        //gcs().send_text(MAV_SEVERITY_WARNING,"Loiter_MotorStopped");
    } else if (takeoff_state.running || takeoff_triggered) {
        loiter_state = Loiter_Takeoff;
        //gcs().send_text(MAV_SEVERITY_WARNING,"Loiter_Takeoff");
    } else if (!ap.auto_armed || ap.land_complete) {
        loiter_state = Loiter_Landed;
        //gcs().send_text(MAV_SEVERITY_WARNING,"Loiter_Landed");
    } else {
        loiter_state = Loiter_Flying;
        //gcs().send_text(MAV_SEVERITY_WARNING,"Loiter_Flying");
    }

    // Loiter State Machine
    switch (loiter_state) {

    case Loiter_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
#if FRAME_CONFIG == HELI_FRAME
        // force descent rate and call position controller
        //pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
#else
        wp_nav->init_loiter_target();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        //pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());
        //pos_control->update_z_controller();
        break;

    case Loiter_Takeoff:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }

        // get takeoff adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // run loiter controller
        wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());

        // update altitude target and call position controller
        //pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        //pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        //pos_control->update_z_controller();
        break;

    case Loiter_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            //motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        wp_nav->init_loiter_target();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        //pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        //pos_control->update_z_controller();

        break;
    case Loiter_Flying:
        // set motors to full range
        //motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
#if PRECISION_LANDING == ENABLED
        if (do_precision_loiter()) {
            precision_loiter_xy();
        }
#endif
        // run loiter controller
        wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());
        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            //target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
            //准备修改为比例关系
        }
        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
        // update altitude target and call position controller
        //pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        //pos_control->update_z_controller();
        break;
    }
    // get pilot's desired throttle
    float pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());
    // body-frame rate controller is run directly from 100hz loop
    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
