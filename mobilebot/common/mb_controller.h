#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H


#include "../mobilebot/mobilebot.h"
#define CFG_PATH "/home/debian/mobilebot-w20/bin/pid.cfg"

int mb_initialize_controller();
int mb_load_controller_config();
int mb_robot_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints);
int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints);
int mb_destroy_controller();

/************
* Add your PID and other SISO filters here.
* examples:lib
* rc_filter_t left_wheel_speed_pid;
* rc_filter_t fwd_vel_sp_lpf;
*************/
rc_filter_t left_wheel_velocity_pid;
rc_filter_t right_wheel_velocity_pid;
rc_filter_t left_wheel_velocity_lpf;
rc_filter_t right_wheel_velocity_lpf;
rc_filter_t turn_velocity_lpf;
rc_filter_t fwd_velocity_lpf;
rc_filter_t left_wheel_velocity_pd;
rc_filter_t right_wheel_velocity_pd;
rc_filter_t left_wheel_velocity_i;
rc_filter_t right_wheel_velocity_i;
//1.5
rc_filter_t fwd_velocity_pid;
rc_filter_t turn_velocity_pid;
rc_filter_t left_velocity_des_lpf;
rc_filter_t right_velocity_des_lpf; 

/***********
* For each PID filter you want to load from settings
* add a pid_parameter_t or filter_parameter_t
* example:
* pid_parameters_t left_wheel_speed_params;
* filter_parameters_t fwd_vel_sp_lpf_params;
************/

#endif

