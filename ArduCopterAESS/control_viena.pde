/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_viena.pde - init and run calls for new flight mode
 */

// viena_init - initialise flight mode
static bool viena_init(bool ignore_checks)
{
    // do any required initialisation of the flight mode here
    // this code will be called whenever the operator switches into this mode
    pos_control.set_alt_target(0);
    // return true initialisation is successful, false if it fails
    // if false is returned here the vehicle will remain in the previous flight mode
    return true;
}

// newflightmode_run - runs the main controller
// will be called at 100hz or more
static void viena_run()
{
    // if not armed or throttle <= 100, set throttle to zero and exit immediately
    if(!motors.armed() || g.rc_3.control_in <= 200) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        return;
    } else if (g.rc_3.control_in <= 500) {
        //[Test] Cuando Thr (200 > x <= 500) que haga un aterrizaje decente
        // call z-axis position controller
        float cmb_rate = get_land_descent_speed();
        pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
        pos_control.update_z_controller();
        // record desired climb rate for logging
        desired_climb_rate = cmb_rate;
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    //Por si acaso ponemos límite de altura a 2.5m
    pos_control.set_alt_max(250);

    //[Test] Auto Yaw, el Yaw no debería moverse
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    // Ponemos la altura destino a 1m
    //pos_control.set_alt_target(100);
    pos_control.set_alt_target_with_slew(100, G_Dt);

    // Actualizamos Z del controlador para aplicar el destino a 1m de altura
    pos_control.update_z_controller();

    // convert pilot input into desired vehicle angles or rotation rates
    //   g.rc_1.control_in : pilots roll input in the range -4500 ~ 4500
    //   g.rc_2.control_in : pilot pitch input in the range -4500 ~ 4500
    //   g.rc_3.control_in : pilot's throttle input in the range 0 ~ 1000
    //   g.rc_4.control_in : pilot's yaw input in the range -4500 ~ 4500

    // call one of attitude controller's attitude control functions like
    //   attitude_control.angle_ef_roll_pitch_rate_yaw(roll angle, pitch angle, yaw rate);

    // call position controller's z-axis controller or simply pass through throttle
    //   attitude_control.set_throttle_out(desired throttle, true);


    float target_yaw_rate;
    int16_t target_roll, target_pitch;

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);


    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

}

// get_land_descent_speed - high level landing logic
//      returns climb rate (in cm/s) which should be passed to the position controller
//      should be called at 100hz or higher
static float get_land_descent_speed()
{
#if CONFIG_SONAR == ENABLED
    bool sonar_ok = sonar_enabled && sonar.healthy();
#else
    bool sonar_ok = false;
#endif
    // if we are above 10m and the sonar does not sense anything perform regular alt hold descent
    if (pos_control.get_pos_target().z >= 0.0 && !(sonar_ok && sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        return pos_control.get_speed_down();
    }else{
        return -abs(g.land_speed);
    }
}
