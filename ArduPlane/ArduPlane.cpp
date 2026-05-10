/*
   Lead developer: Andrew Tridgell
 
   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger, Tom Pittenger
   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

   Please contribute your ideas! See https://ardupilot.org/dev for details

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Plane.h"
#include <AP_GPS/AP_GPS.h> // 确保包含头文件
#include<AP_HAL/AP_HAL.h>


#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(Plane, &plane, func, rate_hz, max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(Plane, &plane, func)


/*
  scheduler table - all regular tasks should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table present in each of the
  vehicles to determine the order in which tasks are run.  Convenience
  methods SCHED_TASK and SCHED_TASK_CLASS are provided to build
  entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

FAST_TASK entries are run on every loop even if that means the loop
overruns its allotted time
 */
const AP_Scheduler::Task Plane::scheduler_tasks[] = {
                           // Units:   Hz      us
    FAST_TASK(ahrs_update),
    FAST_TASK(update_control_mode),
    FAST_TASK(stabilize),
    FAST_TASK(set_servos),
    SCHED_TASK(read_radio,             50,    100,   6),
    SCHED_TASK(check_short_failsafe,   50,    100,   9),
    SCHED_TASK(update_speed_height,    50,    200,  12),
    SCHED_TASK(update_throttle_hover, 100,     90,  24),
    SCHED_TASK_CLASS(RC_Channels,     (RC_Channels*)&plane.g2.rc_channels, read_mode_switch,           7,    100, 27),
    SCHED_TASK(update_GPS_50Hz,        50,    300,  30),
    SCHED_TASK(update_GPS_10Hz,        10,    400,  33),
    SCHED_TASK(navigate,               10,    150,  36),
    SCHED_TASK(update_compass,         10,    200,  39),
    SCHED_TASK(calc_airspeed_errors,   10,    100,  42),
    SCHED_TASK(update_alt,             10,    200,  45),
    SCHED_TASK(adjust_altitude_target, 10,    200,  48),
#if AP_ADVANCEDFAILSAFE_ENABLED
    SCHED_TASK(afs_fs_check,           10,    100,  51),
#endif
    SCHED_TASK(ekf_check,              10,     75,  54),
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_receive,   300,  500,  57),
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_send,      300,  750,  60),
#if AP_SERVORELAYEVENTS_ENABLED
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &plane.ServoRelayEvents, update_events, 50, 150,  63),
#endif
    SCHED_TASK_CLASS(AP_BattMonitor, &plane.battery, read,   10, 300,  66),
#if AP_RANGEFINDER_ENABLED
    SCHED_TASK(read_rangefinder,       50,    100, 78),
#endif
#if AP_ICENGINE_ENABLED
    SCHED_TASK_CLASS(AP_ICEngine,      &plane.g2.ice_control, update,     10, 100,  81),
#endif
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow, &plane.optflow, update,    50,    50,  87),
#endif
    SCHED_TASK(one_second_loop,         1,    400,  90),
    SCHED_TASK(three_hz_loop,           3,     75,  93),
    SCHED_TASK(check_long_failsafe,     3,    400,  96),
#if AP_RPM_ENABLED
    SCHED_TASK_CLASS(AP_RPM,           &plane.rpm_sensor,     update,     10, 100,  99),
#endif
#if AP_AIRSPEED_AUTOCAL_ENABLE
    SCHED_TASK(airspeed_ratio_update,   1,    100,  102),
#endif // AP_AIRSPEED_AUTOCAL_ENABLE
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount, &plane.camera_mount, update, 50, 100, 105),
#endif // HAL_MOUNT_ENABLED
#if AP_CAMERA_ENABLED  
    SCHED_TASK_CLASS(AP_Camera, &plane.camera, update,      50, 100, 108),
#endif // CAMERA == ENABLED
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler, &plane.scheduler, update_logging,         0.2,    100, 111),
#endif
    SCHED_TASK(compass_save,          0.1,    200, 114),
#if HAL_LOGGING_ENABLED
    SCHED_TASK(Log_Write_FullRate,        400,    300, 117),
    SCHED_TASK(update_logging10,        10,    300, 120),
    SCHED_TASK(update_logging25,        25,    300, 123),
#endif
#if HAL_SOARING_ENABLED
    SCHED_TASK(update_soaring,         50,    400, 126),
#endif
    SCHED_TASK(parachute_check,        10,    200, 129),
#if AP_TERRAIN_AVAILABLE
    SCHED_TASK_CLASS(AP_Terrain, &plane.terrain, update, 10, 200, 132),
#endif // AP_TERRAIN_AVAILABLE
    SCHED_TASK(update_is_flying_5Hz,    5,    100, 135),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Logger,         &plane.logger, periodic_tasks, 200, 400, 138),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor, &plane.ins,    periodic,       50,  50, 141),
#if HAL_ADSB_ENABLED
    SCHED_TASK(avoidance_adsb_update,  10,    100, 144),
#endif
    SCHED_TASK_CLASS(RC_Channels,       (RC_Channels*)&plane.g2.rc_channels, read_aux_all,           10,    200, 147),
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button, &plane.button, update, 5, 100, 150),
#endif
#if AP_LANDINGGEAR_ENABLED
    SCHED_TASK(landing_gear_update, 5, 50, 159),
#endif
#if AC_PRECLAND_ENABLED
    SCHED_TASK(precland_update, 400, 50, 160),
#endif
#if AP_QUICKTUNE_ENABLED
    SCHED_TASK(update_quicktune, 40, 100, 163),
#endif

    SCHED_TASK(update_altitude,        20,    400,  166), // 更新实时高度
    SCHED_TASK(reset_waypoint_altitude,10,    200,  169), //定高航线更新
    SCHED_TASK(data_send,              2,     400,  172), //发送信息
    SCHED_TASK(Throwwater,            200,    400,  175),// 投水主函数
    SCHED_TASK(active_airbrake,       100,    100,  178), //主动空刹
    SCHED_TASK(set_mixcontrol,        100,    100,  181), //设置混控
};

void Plane::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count,
                                uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

#if HAL_QUADPLANE_ENABLED
constexpr int8_t Plane::_failsafe_priorities[7];
#else
constexpr int8_t Plane::_failsafe_priorities[6];
#endif

// update AHRS system
void Plane::ahrs_update()
{
    arming.update_soft_armed();

    ahrs.update();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_IMU();
    }
#endif

    // calculate a scaled roll limit based on current pitch
    roll_limit_cd = aparm.roll_limit*100;
    pitch_limit_min = aparm.pitch_limit_min;

    bool rotate_limits = true;
#if HAL_QUADPLANE_ENABLED
    if (quadplane.tailsitter.active()) {
        rotate_limits = false;
    }
#endif
    if (rotate_limits) {
        roll_limit_cd *= ahrs.cos_pitch();
        pitch_limit_min *= fabsf(ahrs.cos_roll());
    }

    // updated the summed gyro used for ground steering and
    // auto-takeoff. Dot product of DCM.c with gyro vector gives earth
    // frame yaw rate
    steer_state.locked_course_err += ahrs.get_yaw_rate_earth() * G_Dt;
    steer_state.locked_course_err = wrap_PI(steer_state.locked_course_err);

#if HAL_QUADPLANE_ENABLED
    // check if we have had a yaw reset from the EKF
    quadplane.check_yaw_reset();

    // update inertial_nav for quadplane
    quadplane.inertial_nav.update();
#endif

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_VIDEO_STABILISATION)) {
        ahrs.write_video_stabilisation();
    }
#endif
}

/*
  update 50Hz speed/height controller
 */
void Plane::update_speed_height(void)
{
    bool should_run_tecs = control_mode->does_auto_throttle();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.should_disable_TECS()) {
        should_run_tecs = false;
    }
#endif

    if (auto_state.idle_mode) {
        should_run_tecs = false;
    }

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (mode_auto.in_pullup()) {
        should_run_tecs = false;
    }
#endif

    if (should_run_tecs) {
	    // Call TECS 50Hz update. Note that we call this regardless of
	    // throttle suppressed, as this needs to be running for
	    // takeoff detection
        TECS_controller.update_50hz();
    }

#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        quadplane.update_throttle_mix();
    }
#endif
}


/*
  read and update compass
 */
void Plane::update_compass(void)
{
    compass.read();
}

#if HAL_LOGGING_ENABLED
/*
  do 10Hz logging
 */
void Plane::update_logging10(void)
{
    bool log_faster = (should_log(MASK_LOG_ATTITUDE_FULLRATE) || should_log(MASK_LOG_ATTITUDE_FAST));
    if (should_log(MASK_LOG_ATTITUDE_MED) && !log_faster) {
        Log_Write_Attitude();
        ahrs.Write_AOA_SSA();
    } else if (log_faster) {
        ahrs.Write_AOA_SSA();
    }
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}

/*
  do 25Hz logging
 */
void Plane::update_logging25(void)
{
    // MASK_LOG_ATTITUDE_FULLRATE logs at 400Hz, MASK_LOG_ATTITUDE_FAST at 25Hz, MASK_LOG_ATTIUDE_MED logs at 10Hz
    // highest rate selected wins
    bool log_faster = should_log(MASK_LOG_ATTITUDE_FULLRATE);
    if (should_log(MASK_LOG_ATTITUDE_FAST) && !log_faster) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
        if (!should_log(MASK_LOG_NOTCH_FULLRATE)) {
            AP::ins().write_notch_log_messages();
        }
#endif
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
    }

    if (should_log(MASK_LOG_NTUN)) {
        Log_Write_Nav_Tuning();
        Log_Write_Guided();
    }

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();

    if (should_log(MASK_LOG_IMU))
        AP::ins().Write_Vibration();
}
#endif  // HAL_LOGGING_ENABLED

/*
  check for AFS failsafe check
 */
#if AP_ADVANCEDFAILSAFE_ENABLED
void Plane::afs_fs_check(void)
{
    afs.check(failsafe.AFS_last_valid_rc_ms);
}
#endif

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

void Plane::one_second_loop()
{
    // make it possible to change control channel ordering at runtime
    set_control_channels();

#if HAL_WITH_IO_MCU
    iomcu.setup_mixing(&rcmap, g.override_channel.get(), g.mixing_gain, g2.manual_rc_mask);
#endif

#if HAL_ADSB_ENABLED
    adsb.set_stall_speed_cm(aparm.airspeed_min * 100); // convert m/s to cm/s
    adsb.set_max_speed(aparm.airspeed_max);
#endif

    if (flight_option_enabled(FlightOptions::ENABLE_DEFAULT_AIRSPEED)) {
        // use average of min and max airspeed as default airspeed fusion with high variance
        ahrs.writeDefaultAirSpeed((float)((aparm.airspeed_min + aparm.airspeed_max)/2),
                                  (float)((aparm.airspeed_max - aparm.airspeed_min)/2));
    }

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    AP::srv().enable_aux_servos();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::Required::NO;

#if AP_TERRAIN_AVAILABLE && HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif

    // update home position if NOT armed and gps position has
    // changed. Update every 5s at most
    if (!arming.is_armed() &&
        gps.last_message_time_ms() - last_home_update_ms > 5000 &&
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            last_home_update_ms = gps.last_message_time_ms();
            update_home();
            
            // reset the landing altitude correction
            landing.alt_offset = 0;
    }

    // this ensures G_Dt is correct, catching startup issues with constructors
    // calling the scheduler methods
    if (!is_equal(1.0f/scheduler.get_loop_rate_hz(), scheduler.get_loop_period_s()) ||
        !is_equal(G_Dt, scheduler.get_loop_period_s())) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    const float loop_rate = AP::scheduler().get_filtered_loop_rate_hz();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        quadplane.attitude_control->set_notch_sample_rate(loop_rate);
    }
#endif
    rollController.set_notch_sample_rate(loop_rate);
    pitchController.set_notch_sample_rate(loop_rate);
    yawController.set_notch_sample_rate(loop_rate);
}

void Plane::three_hz_loop()
{
#if AP_FENCE_ENABLED
    fence_check();
#endif
}

void Plane::compass_save()
{
    if (AP::compass().available() &&
        compass.get_learn_type() >= Compass::LEARN_INTERNAL &&
        !hal.util->get_soft_armed()) {
        /*
          only save offsets when disarmed
         */
        compass.save_offsets();
    }
}

#if AP_AIRSPEED_AUTOCAL_ENABLE
/*
  once a second update the airspeed calibration ratio
 */
void Plane::airspeed_ratio_update(void)
{
    if (!hal.util->get_soft_armed() ||
        !ahrs.get_fly_forward() ||
        !is_flying() ||
        !airspeed.enabled() ||
        gps.status() < AP_GPS::GPS_OK_FIX_3D ||
        gps.ground_speed() < 4) {
        // don't calibrate when not moving
        return;        
    }
    if (airspeed.get_airspeed() < aparm.airspeed_min &&
        gps.ground_speed() < (uint32_t)aparm.airspeed_min) {
        // don't calibrate when flying below the minimum airspeed. We
        // check both airspeed and ground speed to catch cases where
        // the airspeed ratio is way too low, which could lead to it
        // never coming up again
        return;
    }
    if (labs(ahrs.roll_sensor) > roll_limit_cd ||
        ahrs.pitch_sensor > aparm.pitch_limit_max*100 ||
        ahrs.pitch_sensor < pitch_limit_min*100) {
        // don't calibrate when going beyond normal flight envelope
        return;
    }
    const Vector3f &vg = gps.velocity();
    airspeed.update_calibration(vg, aparm.airspeed_max);
}
#endif // AP_AIRSPEED_AUTOCAL_ENABLE

/*
  read the GPS and update position
 */
void Plane::update_GPS_50Hz(void)
{
    gps.update();

    update_current_loc();
}

/*
  read update GPS position - 10Hz update
 */
void Plane::update_GPS_10Hz(void)
{
    static uint32_t last_gps_msg_ms;
    if (gps.last_message_time_ms() != last_gps_msg_ms && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        last_gps_msg_ms = gps.last_message_time_ms();

        if (ground_start_count > 1) {
            ground_start_count--;
        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0 && current_loc.lng == 0) {
                ground_start_count = 5;

            } else if (!hal.util->was_watchdog_reset()) {
                if (!set_home_persistently(gps.location())) {
                    // silently ignore failure...
                }

                next_WP_loc = prev_WP_loc = home;

                ground_start_count = 0;
            }
        }

        // update wind estimate
        ahrs.estimate_wind();
    } else if (gps.status() < AP_GPS::GPS_OK_FIX_3D && ground_start_count != 0) {
        // lost 3D fix, start again
        ground_start_count = 5;
    }

    calc_gndspeed_undershoot();
}

/*
  main control mode dependent update code
 */
void Plane::update_control_mode(void)
{
    if ((control_mode != &mode_auto) && (control_mode != &mode_takeoff)) {
        // hold_course is only used in takeoff and landing
        steer_state.hold_course_cd = -1;
    }
    // refresh the throttle limits, to avoid using stale values
    // they will be updated once takeoff_calc_throttle is called
    takeoff_state.throttle_lim_max = 100.0f;
    takeoff_state.throttle_lim_min = -100.0f;

    update_fly_forward();

    control_mode->update();
}


void Plane::update_fly_forward(void)
{
    // ensure we are fly-forward when we are flying as a pure fixed
    // wing aircraft. This helps the EKF produce better state
    // estimates as it can make stronger assumptions
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        if (quadplane.tailsitter.is_in_fw_flight()) {
            ahrs.set_fly_forward(true);
            return;
        }

        if (quadplane.in_vtol_mode()) {
            ahrs.set_fly_forward(false);
            return;
        }

        if (quadplane.in_assisted_flight()) {
            ahrs.set_fly_forward(false);
            return;
        }
    }
#endif

    if (auto_state.idle_mode) {
        // don't fuse airspeed when in balloon lift
        ahrs.set_fly_forward(false);
        return;
    }

    if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        ahrs.set_fly_forward(landing.is_flying_forward());
        return;
    }

    ahrs.set_fly_forward(true);
}

/*
  set the flight stage
 */
void Plane::set_flight_stage(AP_FixedWing::FlightStage fs)
{
    if (fs == flight_stage) {
        return;
    }

    landing.handle_flight_stage_change(fs == AP_FixedWing::FlightStage::LAND);

    const bool is_landing = (fs == AP_FixedWing::FlightStage::LAND);

    landing.handle_flight_stage_change(is_landing);

#if AP_LANDINGGEAR_ENABLED
    if (is_landing) {
        plane.g2.landing_gear.deploy_for_landing();
    }

    const bool is_takeoff_complete = (flight_stage == AP_FixedWing::FlightStage::TAKEOFF &&
                                      fs == AP_FixedWing::FlightStage::NORMAL);
    if (is_takeoff_complete &&
        arming.is_armed_and_safety_off() &&
        is_flying()) {
            g2.landing_gear.retract_after_takeoff();
    }
#endif
    
    if (fs == AP_FixedWing::FlightStage::ABORT_LANDING) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Landing aborted, climbing to %dm",
                        int(auto_state.takeoff_altitude_rel_cm/100));
    }

    flight_stage = fs;
#if HAL_LOGGING_ENABLED
    Log_Write_Status();
#endif
}

void Plane::update_alt()
{
    barometer.update();

    // calculate the sink rate.
    float sink_rate;
    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        sink_rate = vel.z;
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
        sink_rate = gps.velocity().z;
    } else {
        sink_rate = -barometer.get_climb_rate();        
    }

    // low pass the sink rate to take some of the noise out
    auto_state.sink_rate = 0.8f * auto_state.sink_rate + 0.2f*sink_rate;
#if HAL_PARACHUTE_ENABLED
    parachute.set_sink_rate(auto_state.sink_rate);
#endif

    update_flight_stage();

#if AP_SCRIPTING_ENABLED
    if (nav_scripting_active()) {
        // don't call TECS while we are in a trick
        return;
    }
#endif

    bool should_run_tecs = control_mode->does_auto_throttle();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.should_disable_TECS()) {
        should_run_tecs = false;
    }
#endif

    if (auto_state.idle_mode) {
        should_run_tecs = false;
    }

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (mode_auto.in_pullup()) {
        should_run_tecs = false;
    }
#endif
    
    if (should_run_tecs && !throttle_suppressed) {

        float distance_beyond_land_wp = 0;
        if (flight_stage == AP_FixedWing::FlightStage::LAND &&
            current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
            distance_beyond_land_wp = current_loc.get_distance(next_WP_loc);
        }

        tecs_target_alt_cm = relative_target_altitude_cm();

        if (control_mode == &mode_rtl && !rtl.done_climb && (g2.rtl_climb_min > 0 || (plane.flight_option_enabled(FlightOptions::CLIMB_BEFORE_TURN)))) {
            // ensure we do the initial climb in RTL. We add an extra
            // 10m in the demanded height to push TECS to climb
            // quickly
            tecs_target_alt_cm = MAX(tecs_target_alt_cm, prev_WP_loc.alt - home.alt) + (g2.rtl_climb_min+10)*100;
        }

        TECS_controller.update_pitch_throttle(tecs_target_alt_cm,
                                                 target_airspeed_cm,
                                                 flight_stage,
                                                 distance_beyond_land_wp,
                                                 get_takeoff_pitch_min_cd(),
                                                 throttle_nudge,
                                                 tecs_hgt_afe(),
                                                 aerodynamic_load_factor,
                                                 g.pitch_trim.get());
    }
}

/*
  recalculate the flight_stage
 */
void Plane::update_flight_stage(void)
{
    // Update the speed & height controller states
    if (control_mode->does_auto_throttle() && !throttle_suppressed) {
        if (control_mode == &mode_auto) {
#if HAL_QUADPLANE_ENABLED
            if (quadplane.in_vtol_auto()) {
                set_flight_stage(AP_FixedWing::FlightStage::VTOL);
                return;
            }
#endif
            if (auto_state.takeoff_complete == false) {
                set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);
                return;
            } else if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {
                if (landing.is_commanded_go_around() || flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
                    // abort mode is sticky, it must complete while executing NAV_LAND
                    set_flight_stage(AP_FixedWing::FlightStage::ABORT_LANDING);
                } else if (landing.get_abort_throttle_enable() && get_throttle_input() >= 90 &&
                           landing.request_go_around()) {
                    gcs().send_text(MAV_SEVERITY_INFO,"Landing aborted via throttle");
                    set_flight_stage(AP_FixedWing::FlightStage::ABORT_LANDING);
                } else {
                    set_flight_stage(AP_FixedWing::FlightStage::LAND);
                }
                return;
            }
#if HAL_QUADPLANE_ENABLED
            if (quadplane.in_assisted_flight()) {
                set_flight_stage(AP_FixedWing::FlightStage::VTOL);
                return;
            }
#endif
            set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
        } else if (control_mode != &mode_takeoff) {
            // If not in AUTO then assume normal operation for normal TECS operation.
            // This prevents TECS from being stuck in the wrong stage if you switch from
            // AUTO to, say, FBWB during a landing, an aborted landing or takeoff.
            set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
        }
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        set_flight_stage(AP_FixedWing::FlightStage::VTOL);
        return;
    }
#endif
    set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
}




/*
    If land_DisarmDelay is enabled (non-zero), check for a landing then auto-disarm after time expires

    only called from AP_Landing, when the landing library is ready to disarm
 */
void Plane::disarm_if_autoland_complete()
{
    if (landing.get_disarm_delay() > 0 &&
        !is_flying() &&
        arming.arming_required() != AP_Arming::Required::NO &&
        arming.is_armed()) {
        /* we have auto disarm enabled. See if enough time has passed */
        if (millis() - auto_state.last_flying_ms >= landing.get_disarm_delay()*1000UL) {
            if (arming.disarm(AP_Arming::Method::AUTOLANDED)) {
                gcs().send_text(MAV_SEVERITY_INFO,"Auto disarmed");
            }
        }
    }
}

bool Plane::trigger_land_abort(const float climb_to_alt_m)
{
    if (plane.control_mode != &plane.mode_auto) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto()) {
        return quadplane.abort_landing();
    }
#endif

    uint16_t mission_id = plane.mission.get_current_nav_cmd().id;
    bool is_in_landing = (plane.flight_stage == AP_FixedWing::FlightStage::LAND) ||
        plane.is_land_command(mission_id);
    if (is_in_landing) {
        // fly a user planned abort pattern if available
        if (plane.have_position && plane.mission.jump_to_abort_landing_sequence(plane.current_loc)) {
            return true;
        }

        // only fly a fixed wing abort if we aren't doing quadplane stuff, or potentially
        // shooting a quadplane approach
#if HAL_QUADPLANE_ENABLED
        const bool attempt_go_around =
            (!plane.quadplane.available()) ||
            ((!plane.quadplane.in_vtol_auto()) &&
                (!plane.quadplane.landing_with_fixed_wing_spiral_approach()));
#else
        const bool attempt_go_around = true;
#endif
        if (attempt_go_around) {
            // Initiate an aborted landing. This will trigger a pitch-up and
            // climb-out to a safe altitude holding heading then one of the
            // following actions will occur, check for in this order:
            // - If MAV_CMD_CONTINUE_AND_CHANGE_ALT is next command in mission,
            //      increment mission index to execute it
            // - else if DO_LAND_START is available, jump to it
            // - else decrement the mission index to repeat the landing approach

            if (!is_zero(climb_to_alt_m)) {
                plane.auto_state.takeoff_altitude_rel_cm = climb_to_alt_m * 100;
            }
            if (plane.landing.request_go_around()) {
                plane.auto_state.next_wp_crosstrack = false;
                return true;
            }
        }
    }
    return false;
}


/*
  the height above field elevation that we pass to TECS
 */
float Plane::tecs_hgt_afe(void)
{
    /*
      pass the height above field elevation as the height above
      the ground when in landing, which means that TECS gets the
      rangefinder information and thus can know when the flare is
      coming.
    */
    float hgt_afe;

    if (flight_stage == AP_FixedWing::FlightStage::LAND) {

        #if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
            // if external HAGL is active use that
            if (get_external_HAGL(hgt_afe)) {
                return hgt_afe;
            }
        #endif

        hgt_afe = height_above_target();
#if AP_RANGEFINDER_ENABLED
        hgt_afe -= rangefinder_correction();
#endif
    } else {
        // when in normal flight we pass the hgt_afe as relative
        // altitude to home
        hgt_afe = relative_altitude;
    }
    return hgt_afe;
}

// vehicle specific waypoint info helpers
bool Plane::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        distance = quadplane.using_wp_nav() ? quadplane.wp_nav->get_wp_distance_to_destination() * 0.01 : 0;
        return true;
    }
#endif
    distance = auto_state.wp_distance;
    return true;
}

bool Plane::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        bearing = quadplane.using_wp_nav() ? quadplane.wp_nav->get_wp_bearing_to_destination() : 0;
        return true;
    }
#endif
    bearing = nav_controller->target_bearing_cd() * 0.01;
    return true;
}

bool Plane::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // see GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        xtrack_error = quadplane.using_wp_nav() ? quadplane.wp_nav->crosstrack_error() : 0;
        return true;
    }
#endif
    xtrack_error = nav_controller->crosstrack_error();
    return true;
}

#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
// set target location (for use by external control and scripting)
bool Plane::set_target_location(const Location &target_loc)
{
    Location loc{target_loc};
    fix_terrain_WP(loc, __LINE__);

    if (plane.control_mode != &plane.mode_guided) {
        // only accept position updates when in GUIDED mode
        return false;
    }
    // convert to absolute
    if (!loc.terrain_alt) {
        loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }
    plane.set_guided_WP(loc);
    return true;
}
#endif //AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED

#if AP_SCRIPTING_ENABLED
// get target location (for use by scripting)
bool Plane::get_target_location(Location& target_loc)
{
    switch (control_mode->mode_number()) {
    case Mode::Number::RTL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::AUTO:
    case Mode::Number::LOITER:
    case Mode::Number::TAKEOFF:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
#endif
        target_loc = next_WP_loc;
        return true;
        break;
    default:
        break;
    }
    return false;
}

/*
  update_target_location() works in all auto navigation modes
 */
bool Plane::update_target_location(const Location &old_loc, const Location &new_loc)
{
    /*
      by checking the caller has provided the correct old target
      location we prevent a race condition where the user changes mode
      or commands a different target in the controlling lua script
     */
    if (!old_loc.same_loc_as(next_WP_loc) ||
        old_loc.get_alt_frame() != new_loc.get_alt_frame()) {
        return false;
    }
    next_WP_loc = new_loc;

    fix_terrain_WP(next_WP_loc, __LINE__);

#if HAL_QUADPLANE_ENABLED
    if (control_mode == &mode_qland || control_mode == &mode_qloiter) {
        mode_qloiter.last_target_loc_set_ms = AP_HAL::millis();
    }
#endif

    return true;
}

// allow for velocity matching in VTOL
bool Plane::set_velocity_match(const Vector2f &velocity)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() || quadplane.in_vtol_land_sequence()) {
        quadplane.poscontrol.velocity_match = velocity;
        quadplane.poscontrol.last_velocity_match_ms = AP_HAL::millis();
        return true;
    }
#endif
    return false;
}

// allow for override of land descent rate
bool Plane::set_land_descent_rate(float descent_rate)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_land_descent() ||
        control_mode == &mode_qland) {
        quadplane.poscontrol.override_descent_rate = descent_rate;
        quadplane.poscontrol.last_override_descent_ms = AP_HAL::millis();
        return true;
    }
#endif
    return false;
}

// Allow for scripting to have control over the crosstracking when exiting and resuming missions or guided flight
// It's up to the Lua script to ensure the provided location makes sense
bool Plane::set_crosstrack_start(const Location &new_start_location)
{        
    prev_WP_loc = new_start_location;
    auto_state.crosstrack = true;
    return true;
}

#endif // AP_SCRIPTING_ENABLED

// returns true if vehicle is landing.
bool Plane::is_landing() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_land_descent()) {
        return true;
    }
#endif
    return control_mode->is_landing();
}

// returns true if vehicle is taking off.
bool Plane::is_taking_off() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_takeoff()) {
        return true;
    }
#endif
    return control_mode->is_taking_off();
}

// correct AHRS pitch for PTCH_TRIM_DEG in non-VTOL modes, and return VTOL view in VTOL
void Plane::get_osd_roll_pitch_rad(float &roll, float &pitch) const
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.show_vtol_view()) {
        pitch = quadplane.ahrs_view->pitch;
        roll = quadplane.ahrs_view->roll;
        return;
    }
#endif
    pitch = ahrs.get_pitch();
    roll = ahrs.get_roll();
    if (!(flight_option_enabled(FlightOptions::OSD_REMOVE_TRIM_PITCH))) {  // correct for PTCH_TRIM_DEG
        pitch -= g.pitch_trim * DEG_TO_RAD;
    }
}

/*
  update current_loc Location
 */
void Plane::update_current_loc(void)
{
    have_position = plane.ahrs.get_location(plane.current_loc);

    // re-calculate relative altitude
    ahrs.get_relative_position_D_home(plane.relative_altitude);
    relative_altitude *= -1.0f;
}

// check if FLIGHT_OPTION is enabled
bool Plane::flight_option_enabled(FlightOptions flight_option) const
{
    return g2.flight_options & flight_option;
}

void Plane::update_altitude()
{
    current_altitude = gps.location().alt - ahrs.get_home().alt; // 获取GPS高度，绝对高度，单位cm
    if(control_mode == &mode_manual){
        uint16_t temp_pwm = 0;
      if (SRV_Channels::get_output_pwm(SRV_Channel::k_elevator,temp_pwm)) {
        current_pitch_pwm = temp_pwm;
    }else{
        current_pitch_pwm = g2.man_neutral;
    }
    }
  
}


void Plane::set_mixcontrol()
{
    ap_var_type mixcontrol;
    AP_Param *ap_mctrl = AP_Param::find("KFF_RDDRMIX",&mixcontrol);
    AP_Int16 *mctrl = (AP_Int16*)ap_mctrl;

    ap_var_type pitch_neutral;
    AP_Param *ap_pntrl = AP_Param::find("SERVO2_TRIM",&pitch_neutral);
    AP_Int16 *pntrl = (AP_Int16*)ap_pntrl;

    static bool mixcontrol_flag = false;
    static bool neutral_flag = false;
    if (control_mode == &mode_auto) {
        if (!mixcontrol_flag) {
        mctrl->set_and_save(g2.Throwwater_mix);
        pntrl->set_and_save(plane.captured_pitch_pwm);
        mixcontrol_flag = true; 
        gcs().send_text(MAV_SEVERITY_INFO,"---YAW & ROLL MixControl activated---");
        gcs().send_text(MAV_SEVERITY_INFO,"------NEUTRAL CHANGED!!!------");
        }
        if( plane.ThroworNot == 1 && !neutral_flag){
            pntrl->set_and_save(g2.man_neutral);
            gcs().send_text(MAV_SEVERITY_INFO,"------NUETRAL RE——CHANGED!!!------");
            neutral_flag = true;
        }
    }else{
        if (mixcontrol_flag) {
            mctrl->set_and_save(0);
            pntrl->set_and_save(g2.man_neutral);
            mixcontrol_flag = false;
            neutral_flag = false;
            gcs().send_text(MAV_SEVERITY_INFO,"---YAW & ROLL MixControl deactivated---");
                    gcs().send_text(MAV_SEVERITY_INFO,"------NUETRAL RE——CHANGED!!!------");

        }
    }
}



void Plane::reset_waypoint_altitude()
{
    if (control_mode != &mode_auto) {
        mission_modified = false;
        return;
    }

    if (!mission_modified) {
        message_sent = false;
        uint16_t clear_num = mission.num_commands();
        
        // 获取当前位置
        Location new_loc;
        new_loc.lat = gps.location().lat;
        new_loc.lng = gps.location().lng;

         Location current_lo = new_loc;
         float heading_rad = ahrs.get_yaw(); ;
         current_lo.offset_bearing(heading_rad, g2.Airline_Feedforwad_distance);  //航线前视距离，为调整偏航预留裕度

        // 获取目标位置
        Location target_loc;
        target_loc.lat = g2.throwwater_target_lat;
        target_loc.lng = g2.throwwater_target_lng;
        target_loc.alt = (captured_altitude +g2.Throwwater_extra_altitude) * 100;
        
        // 4. 直接计算四分位点
        const int TOTAL_POINTS = 5; // 起点 + 四分位点 + 终点
        Location waypoints[TOTAL_POINTS];
        const int32_t max_lng = 1800000000;  // 180度 × 1e7
        const int32_t min_lng = -1800000000;
        const int64_t full_circle = 3600000000;  // 360度 × 1e7
        
        // 计算25%点（内联安全计算）
        Location quarter_point;
        quarter_point.lat = (int32_t)((static_cast<int64_t>(current_lo.lat) * 3 + target_loc.lat) / 4);
        quarter_point.alt = (captured_altitude+g2.Throwwater_extra_altitude) * 100;
        
        // 内联经度计算（防止溢出）
        int64_t quarter_lng = static_cast<int64_t>(current_lo.lng) * 3 + target_loc.lng;
        quarter_lng /= 4;
        // 处理经度边界
        while (quarter_lng > max_lng) quarter_lng -= full_circle;
        while (quarter_lng < min_lng) quarter_lng += full_circle;
        quarter_point.lng = static_cast<int32_t>(quarter_lng);
        
        // 计算中点（内联安全计算）
        Location mid_point;
        mid_point.lat = (int32_t)((static_cast<int64_t>(current_lo.lat) + target_loc.lat) / 2);
        mid_point.alt = (captured_altitude+g2.Throwwater_extra_altitude) * 100;
        
        // 内联经度计算
        int64_t mid_lng = static_cast<int64_t>(current_lo.lng) + target_loc.lng;
        mid_lng /= 2;

        // 处理经度边界
        while (mid_lng > max_lng) mid_lng -= full_circle;
        while (mid_lng < min_lng) mid_lng += full_circle;
        mid_point.lng = static_cast<int32_t>(mid_lng);
        
        // 计算75%点（内联安全计算）
        Location three_quarter_point;
        three_quarter_point.lat = (int32_t)((static_cast<int64_t>(current_lo.lat) + target_loc.lat * 3) / 4);
        three_quarter_point.alt = captured_altitude * 100;
        
        // 内联经度计算
        int64_t three_quarter_lng = current_lo.lng + static_cast<int64_t>(target_loc.lng) * 3;
        three_quarter_lng /= 4;
        // 处理经度边界
        while (three_quarter_lng > max_lng) three_quarter_lng -= full_circle;
        while (three_quarter_lng < min_lng) three_quarter_lng += full_circle;
        three_quarter_point.lng = static_cast<int32_t>(three_quarter_lng);


        // 起点
        waypoints[0] = current_lo;
        
        // 25%点：当前点和目标点平均值
        waypoints[1].lat =  quarter_point.lat;
        waypoints[1].lng = quarter_point.lng;
        waypoints[1].alt = (captured_altitude+g2.Throwwater_extra_altitude) * 100;
        
        // 50%点：直接取中点
        waypoints[2].lat = mid_point.lat;
        waypoints[2].lng = mid_point.lng;
        waypoints[2].alt = (captured_altitude+g2.Throwwater_extra_altitude) * 100;
        
        // 75%点：当前点1/4 + 目标点3/4
        waypoints[3].lat = three_quarter_point.lat;
        waypoints[3].lng = three_quarter_point.lng;
        waypoints[3].alt = (captured_altitude+g2.Throwwater_extra_altitude) * 100;
        
        // 终点
        waypoints[4] = target_loc;
        
        // 5. 添加所有点到任务
        for (int i = 0; i < TOTAL_POINTS; i++) {
            AP_Mission::Mission_Command cmd;
            cmd.id = MAV_CMD_NAV_WAYPOINT;
            cmd.p1 = 1; // 到达半径
            cmd.content.location = waypoints[i];
            
            // 直接添加到当前任务
            uint16_t index = mission.num_commands();
            mission.add_cmd(cmd);
            mission.write_cmd_to_storage(index, cmd);
        }//?

        mission.remove_cmd(clear_num-1); // 移除原有的任务点
        
        // 8. 更新状态
        mission_modified = true;
        mission_reload = true;
    }

    // 9. 任务重载 - FIXED: 从头开始(移除状态保留)
    if (mission_reload) {
        mission.set_current_cmd(0); // 设置为第一个航点
        mission.start();  // 从头开始执行
        mission_reload = false;
    }

    // 10. 状态通知
    if (mission_modified && !message_sent) {
        gcs().send_text(MAV_SEVERITY_INFO, "任务已重置: 共%u个航点", mission.num_commands());
        message_sent = true;
    }
}




void Plane::active_airbrake()      //开发完成：空气刹车
{
    
    static bool airbrake_flag = false;
    ap_var_type left_ptr;          // 用于接收组信息
    ap_var_type right_ptr; 
    ap_var_type left_min_ptr;          // 用于接收组信息
    ap_var_type right_min_ptr;  
    ap_var_type left_max_ptr;          // 用于接收组信息
    ap_var_type right_max_ptr;



    AP_Param *ap_l = AP_Param::find("SERVO1_TRIM",&left_ptr);     
    AP_Param *ap_r = AP_Param::find("SERVO6_TRIM",&right_ptr);      //中立位置变量是SRV_Channel类中私有成员，要调用中立位置的值只能用这种公共方法
    AP_Param *ap_lmin = AP_Param::find("SERVO1_MIN",&left_min_ptr);     
    AP_Param *ap_rmin = AP_Param::find("SERVO6_MIN",&right_min_ptr);      //中立位置变量是SRV_Channel类中私有成员，要调用中立位置的值只能用这种公共方法
    AP_Param *ap_lmax = AP_Param::find("SERVO1_MAX",&left_max_ptr);     
    AP_Param *ap_rmax = AP_Param::find("SERVO6_MAX",&right_max_ptr);      //中立位置变量是SRV_Channel类中私有成员，要调用中立位置的值只能用这种公共方法

    AP_Int16 *left = (AP_Int16*)ap_l;
    AP_Int16 *right = (AP_Int16*)ap_r;
    AP_Int16 *left_min = (AP_Int16*)ap_lmin;
    AP_Int16 *right_min = (AP_Int16*)ap_rmin;
    AP_Int16 *left_max = (AP_Int16*)ap_lmax;
    AP_Int16 *right_max = (AP_Int16*)ap_rmax;

    /*int16_t min_pwm_left = SRV_Channels::srv_channel(0)->get_output_min();
    int16_t max_pwm_left = SRV_Channels::srv_channel(0)->get_output_max();
    int16_t min_pwm_right = SRV_Channels::srv_channel(5)->get_output_min();
    int16_t max_pwm_right = SRV_Channels::srv_channel(5)->get_output_max();*/

    if(g2.Airbrake_record == 1){
         g2.min_pwm_left.set_and_save(SRV_Channels::srv_channel(0)->get_output_min());
         g2.max_pwm_left.set_and_save(SRV_Channels::srv_channel(0)->get_output_max());
         g2.min_pwm_right.set_and_save(SRV_Channels::srv_channel(5)->get_output_min());
         g2.max_pwm_right.set_and_save(SRV_Channels::srv_channel(5)->get_output_max());
         g2.Airbrake_record.set_and_save(0);
        gcs().send_text(MAV_SEVERITY_INFO,"airbrake position recorded");
    }
   

    if(control_mode == &mode_manual)
    {
        uint16_t airbrake_pwm_value = RC_Channels::get_radio_in(g2.Airbrake_rc_channel);
        if(airbrake_pwm_value < 1500)      //这个判断还可以再优化
        {
            if(!airbrake_flag)
            { //如果当前没有处于空气刹车状态
                if(g2.Airbrake_left_pwm < g2.min_pwm_left )
                {
                    left_min->set_and_save(g2.Airbrake_left_pwm); 
                    left->set_and_save(g2.Airbrake_left_pwm); //如果预设的PWM值不在范围内，则复位为最小值
                }else if(g2.Airbrake_left_pwm > g2.max_pwm_left)
                {
                    left_max->set_and_save(g2.Airbrake_left_pwm);
                    left->set_and_save(g2.Airbrake_left_pwm);
                }else{
                    left->set_and_save(g2.Airbrake_left_pwm);
                }
                
                if(g2.Airbrake_right_pwm < g2.min_pwm_right )
                {
                    right_min->set_and_save(g2.Airbrake_right_pwm);
                    right->set_and_save(g2.Airbrake_right_pwm); //如果预设的PWM值不在范围内，则复位为最小值
                }else if(g2.Airbrake_right_pwm > g2.max_pwm_right)
                {
                    right_max->set_and_save(g2.Airbrake_right_pwm);
                    right->set_and_save(g2.Airbrake_right_pwm);
                }else{
                    right->set_and_save(g2.Airbrake_right_pwm);
                }//?
            

            airbrake_flag = true; // 记录空速刹车状态
            gcs().send_text(MAV_SEVERITY_INFO,"airbrake activated, pwm: %.u", airbrake_pwm_value);
            }
        }
        else{
            if(airbrake_flag)
            {
                left->set_and_save(g2.Airbrake_pre_left);      //退出空气刹车，复位至中立位置（需要预设，和调中立的时候中立值保持一致）
                right->set_and_save(g2.Airbrake_pre_right);
                left_min->set_and_save(g2.min_pwm_left);
                right_min->set_and_save(g2.min_pwm_right);
                left_max->set_and_save(g2.max_pwm_left);
                right_max->set_and_save(g2.max_pwm_right);      


                airbrake_flag = false; // 复位空气刹车状态
                gcs().send_text(MAV_SEVERITY_INFO,"airbrake mode OUT");
            }
        }
    }

}


void Plane::data_send()
{       
    if(plane.ThroworNot==1)
    {
        gcs().send_text(MAV_SEVERITY_INFO,"Bombs away!!!");
        gcs().send_text(MAV_SEVERITY_INFO, "Yeah bombs away!!!");
        gcs().send_text(MAV_SEVERITY_INFO,"DISTANCE: %.2f", plane.tdistance_cur);
        //gcs().send_text(MAV_SEVERITY_INFO,"DISTANCE: %.2f", plane.last_distance);
    }
    else{
    gcs().send_text(MAV_SEVERITY_INFO,"DISTANCE: %.2f", plane.tdistance_cur);
    //gcs().send_text(MAV_SEVERITY_INFO,"P-DISTANCE: %.2f", plane.last_distance);
    }
}
void Plane::Throwwater()
{
    int8_t hih = 0;

    if (control_mode == &mode_auto){
    bool is_auto = (control_mode == &mode_auto);                //监测是否为自动模式，若为自动模式，则TECS控制器将会强制进入水平飞行状态
    get_tecs_controller()->set_force_level_flight(is_auto);  
    }
    

    if(g2.Throwwater_record_position==1){                           //记录投放位置
        g2.throwwater_target_lat.set_and_save(gps.location().lat);
        g2.throwwater_target_lng.set_and_save(gps.location().lng);
        g2.Throwwater_record_position.set(0);

    }


    plane.ttarget.lat = g2.throwwater_target_lat;//??
    plane.ttarget.lng = g2.throwwater_target_lng;

    plane.distance_current2target = plane.current_loc.get_distance_NE(plane.ttarget);//in meters
        //Vector3f tvel;
        //plane.ahrs.get_velocity_NED(tvel);
    float high = (plane.relative_ground_altitude(false) > 0) ? plane.relative_ground_altitude(false) : 0;
    plane.high_kg = high;

    
    plane.drop_time = (sqrtf(2 * high * 9.8015f + gps.velocity().z * gps.velocity().z) - gps.velocity().z) / 9.8015f;
    
      
    //Vector2f distance_current2drop;  //in meters
    plane.distance_current2drop.x = (drop_time + g2.throwwater_delay)* gps.velocity().x;
    plane.distance_current2drop.y = (drop_time + g2.throwwater_delay)* gps.velocity().y;
    //plane.tdistance_cur = hypotf(plane.distance_current2target.x - plane.distance_current2drop.x, plane.distance_current2target.y - plane.distance_current2drop.y);
    

    plane.tdistance_cur_sum = 0;
    for(hih=0;hih<9;hih++)
    {
    	plane.tdistance_cur_reg[hih] = plane.tdistance_cur_reg[hih+1];
        plane.tdistance_cur_sum += plane.tdistance_cur_reg[hih];
    }
    plane.tdistance_cur_reg[9] = hypotf(plane.distance_current2target.x - plane.distance_current2drop.x, plane.distance_current2target.y - plane.distance_current2drop.y);
    plane.tdistance_cur = (plane.tdistance_cur_sum + plane.tdistance_cur_reg[9])/10;
    /*plane.tdistance_chui = (plane.distance_current2drop.x*plane.distance_current2target.y - plane.distance_current2target.x*plane.distance_current2drop.y) / hypotf(plane.distance_current2drop.x, plane.distance_current2drop.y);
    plane.tdistance_yan = sqrtf(plane.tdistance_cur*plane.tdistance_cur - plane.tdistance_chui*plane.tdistance_chui);*/


        if(g2.throwwater==1)  //通过地面站强制投放（一般实验用,接了控的时候用）
        {
            plane.ThroworNot = 1;
        }
        else if(g2.throwwater==2)  //通过地面站强制投放（一般实验用，没接控的时候用）
        {
            plane.ThroworNot = 1;
        }
        else if(plane.tdistance_cur < g2.Throwwater_Radius_Pickle && plane.tdistance_cur > plane.last_distance && control_mode == &mode_auto )  //自动控制投放 增加沿切线方向和垂直距离判断条件：&& plane.tdistance_yan < g2.Throwwater_Yan_Pickle 
        {    
            plane.ThroworNot = 1;
        }

   
        if(RC_Channels::get_radio_in(6) < 1500)  //手动控制投放
        {
            plane.ThroworNot = 1;
        }
    


    //统一投放逻辑,进行投放动作
    if (plane.ThroworNot == 1) {
        SRV_Channels::set_output_pwm_chan(6, g2.Throwater_throw_pwm);    // 投放位置
    } else {
        SRV_Channels::set_output_pwm_chan(6, g2.Throwater_normal_pwm);    // 关闭位置
    }

    //投放自动复位系统
    static uint32_t throw_reset_timer = 0; // 静态变量记录投放时间
    if (plane.ThroworNot == 1) {
        // 如果是新投放（计时器未启动），启动计时器
        if (throw_reset_timer == 0) {
            throw_reset_timer = AP_HAL::millis();
            gcs().send_text(MAV_SEVERITY_INFO, "Bomb dropped, reset timer started");
        }
        // 检查是否超时（5秒）
        else if (AP_HAL::millis() - throw_reset_timer > 5000) {
            plane.ThroworNot = 0; // 重置投放状态
            throw_reset_timer = 0; // 重置计时器
            gcs().send_text(MAV_SEVERITY_INFO, "Bomb state reset after 5s");
        }
    }
    else {
        throw_reset_timer = 0; // 非投放状态时清零计时器
    }
    
    
    
    plane.last_distance = plane.tdistance_cur;
    plane.Log_Write_Throwwater();
}

#if AC_PRECLAND_ENABLED
void Plane::precland_update(void)
{
    // alt will be unused if we pass false through as the second parameter:
#if AP_RANGEFINDER_ENABLED
    return g2.precland.update(rangefinder_state.height_estimate*100, rangefinder_state.in_range);
#else
    return g2.precland.update(0, false);
#endif
}
#endif

#if AP_QUICKTUNE_ENABLED
/*
  update AP_Quicktune object. We pass the supports_quicktune() method
  in so that quicktune can detect if the user changes to a
  non-quicktune capable mode while tuning and the gains can be reverted
 */
void Plane::update_quicktune(void)
{
    quicktune.update(control_mode->supports_quicktune());
}
#endif

AP_HAL_MAIN_CALLBACKS(&plane);
