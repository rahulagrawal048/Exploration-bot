#include "../mobilebot/mobilebot.h"

/*******************************************************************************
* int mb_initialize()
*
* this initializes all the PID controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

int mb_initialize_controller(){
    //mb_load_controller_config();
    left_wheel_velocity_pid = rc_filter_empty();
    right_wheel_velocity_pid = rc_filter_empty();
    rc_filter_pid(&left_wheel_velocity_pid, P, I, D, L_TIME_CONST, DT); 
    rc_filter_pid(&right_wheel_velocity_pid, P, I, D, R_TIME_CONST, DT); 
    rc_filter_enable_saturation(&left_wheel_velocity_pid, -LEFT_PWM_MAX, LEFT_PWM_MAX);
    rc_filter_enable_saturation(&right_wheel_velocity_pid, -RIGHT_PWM_MAX, RIGHT_PWM_MAX);

    //1.5 PIDs
    fwd_velocity_pid = rc_filter_empty();
    turn_velocity_pid = rc_filter_empty();
    rc_filter_pid(&fwd_velocity_pid, P, I, D, L_TIME_CONST, DT); 
    rc_filter_pid(&turn_velocity_pid, P, I, D, R_TIME_CONST, DT); 
    //1.5 LPFs
    left_velocity_des_lpf = rc_filter_empty();
    right_velocity_des_lpf = rc_filter_empty();
    rc_filter_first_order_lowpass(&left_velocity_des_lpf, DT, 0.1);
    rc_filter_first_order_lowpass(&right_velocity_des_lpf, DT, 0.1);

    //Low pass filter definitions:
    left_wheel_velocity_lpf = rc_filter_empty();
    right_wheel_velocity_lpf = rc_filter_empty();
    turn_velocity_lpf = rc_filter_empty();
    fwd_velocity_lpf = rc_filter_empty();
    rc_filter_first_order_lowpass(&left_wheel_velocity_lpf, DT, 0.1);
    rc_filter_first_order_lowpass(&right_wheel_velocity_lpf, DT, 0.1);
    rc_filter_first_order_lowpass(&turn_velocity_lpf, DT, 0.1);
    rc_filter_first_order_lowpass(&fwd_velocity_lpf, DT, 0.1);

    //Resetting integrator
    left_wheel_velocity_pd = rc_filter_empty();
    right_wheel_velocity_pd = rc_filter_empty();
    left_wheel_velocity_i = rc_filter_empty();
    right_wheel_velocity_i = rc_filter_empty();
    rc_filter_pid(&left_wheel_velocity_pd, P, 0.0, D, L_TIME_CONST, DT); 
    rc_filter_pid(&right_wheel_velocity_pd, P, 0.0, D, R_TIME_CONST, DT); 
    rc_filter_pid(&left_wheel_velocity_i, 0.0, I, 0.0, L_TIME_CONST, DT); 
    rc_filter_pid(&right_wheel_velocity_i, 0.0, I, 0.0, R_TIME_CONST, DT); 
    return 0;
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_load_controller_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening pid.cfg\n");
    }

/******
*
*   Example of loading a line from .cfg file:
*
*    fscanf(file, "%f %f %f %f", 
*        &pid_params.kp,
*        &pid_params.ki,
*        &pid_params.kd,
*        &pid_params.dFilterHz
*        );
*
******/

    fclose(file);
    return 0;
}


/*******************************************************************************
* int mb_robot_controller_update()
* 
* 1.5 Robot Velocity Frame Controller
*
* return 0 on success
*
*******************************************************************************/

int mb_robot_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){  

    //Compute Robot's actual forward and turn velocity
    float fwd_velocity_robot = (mb_state->left_velocity + mb_state->right_velocity) / 2;
    float turn_velocity_robot = (mb_state->right_velocity - mb_state->left_velocity) / WHEEL_BASE;

    //Calculate error for 1.5 PID
    float fwd_velocity_error =  mb_setpoints->fwd_velocity - fwd_velocity_robot;
    float turn_velocity_error = mb_setpoints->turn_velocity - turn_velocity_robot;

    //1.5 PID for desired forward and turn speeds
    float fwd_velocity_desired = rc_filter_march(&fwd_velocity_pid, fwd_velocity_error);
    float turn_velocity_desired = rc_filter_march(&turn_velocity_pid, turn_velocity_error);

    //Compute desired left and right velocity
    float left_velocity_desired = fwd_velocity_desired - (turn_velocity_desired * WHEEL_BASE / 2);
    float right_velocity_desired = fwd_velocity_desired + (turn_velocity_desired * WHEEL_BASE / 2);
    
    //Calculate error for adding low pass filter:
    float lpf_leftvel_des = rc_filter_march(&left_velocity_des_lpf, left_velocity_desired);
    float lpf_rightvel_des = rc_filter_march(&right_velocity_des_lpf, right_velocity_desired);
    float lpf_leftvel = rc_filter_march(&left_wheel_velocity_lpf, mb_state->left_velocity);
    float lpf_rightvel = rc_filter_march(&right_wheel_velocity_lpf, mb_state->right_velocity);
    float left_error_lpf = lpf_leftvel_des - lpf_leftvel;
    float right_error_lpf = lpf_rightvel_des - lpf_rightvel;

    //Feedforward with LPF commands:
    float ff_left_cmd_lpf = left_slope * (lpf_leftvel_des) + left_intercept;
    float ff_right_cmd_lpf = right_slope * (lpf_rightvel_des) + right_intercept;
    
    //****Feedfoward with PID with LPF(on fwd and left/right vel)****
    mb_state->left_cmd = rc_filter_march(&left_wheel_velocity_pid, left_error_lpf) + ff_left_cmd_lpf;
    mb_state->right_cmd = rc_filter_march(&right_wheel_velocity_pid, right_error_lpf) + ff_right_cmd_lpf;

    // Maintain PWM thresshold between -1 to 1
    if (mb_state->left_cmd > LEFT_PWM_MAX)
    {
        mb_state->left_cmd = LEFT_PWM_MAX;
    }
    else if (mb_state->left_cmd < -LEFT_PWM_MAX)
    {
        mb_state->left_cmd = -LEFT_PWM_MAX;
    }

    if (mb_state->right_cmd > RIGHT_PWM_MAX)
    {
        mb_state->right_cmd = RIGHT_PWM_MAX;
    }
    else if (mb_state->right_cmd < -RIGHT_PWM_MAX)
    {
        mb_state->right_cmd = -RIGHT_PWM_MAX;
    }

    //Prevent bot from rolling away
    if (mb_setpoints->fwd_velocity + mb_setpoints->turn_velocity == 0)
    {
        mb_state->right_cmd = 0;
        mb_state->left_cmd = 0;
    }

    return 0;
}


/*******************************************************************************
* int mb_controller_update()
* 
* TODO: Write your PID controller here
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints){  
    //Calculate error for PID:
    float left_error = mb_setpoints->fwd_velocity - (mb_setpoints->turn_velocity * WHEEL_BASE / 2) - mb_state->left_velocity;
    float right_error = mb_setpoints->fwd_velocity + (mb_setpoints->turn_velocity * WHEEL_BASE / 2) - mb_state->right_velocity;

    //Feedforward commands
    float ff_left_cmd = left_slope*(mb_setpoints->fwd_velocity - (mb_setpoints->turn_velocity * WHEEL_BASE / 2)) + left_intercept;
    float ff_right_cmd = right_slope*(mb_setpoints->fwd_velocity + (mb_setpoints->turn_velocity * WHEEL_BASE / 2)) + right_intercept;
    
    //****Feedforward (calibrated one motor at a time wait1s)****
    // mb_state->left_cmd = ff_left_cmd;
    // mb_state->right_cmd = ff_right_cmd;

    //****Just PID****
    // mb_state->left_cmd = rc_filter_march(&left_wheel_velocity_pid, left_error);
    // mb_state->right_cmd = rc_filter_march(&right_wheel_velocity_pid, right_error);

    //****Feedforward with PID(calibrated one motor at a time wait1s)****
    // mb_state->left_cmd = rc_filter_march(&left_wheel_velocity_pid, left_error) + ff_left_cmd;
    // mb_state->right_cmd = rc_filter_march(&right_wheel_velocity_pid, right_error) + ff_right_cmd;





    //Resetting integrator:
    //Option1:
    // if (mb_setpoints->fwd_velocity_last != mb_setpoints->fwd_velocity)
    // {
    //     rc_filter_reset(&left_wheel_velocity_i);
    //     rc_filter_reset(&right_wheel_velocity_i);
    // }
    //Option2:
    // if (mb_state->left_velocity == mb_setpoints->fwd_velocity)
    // {
    //     rc_filter_reset(&left_wheel_velocity_i);
    // }
    // if (mb_state->right_velocity == mb_setpoints->fwd_velocity)
    // {
    //     rc_filter_reset(&right_wheel_velocity_i);
    // }
    //Option3:
    // int tolerance = 0.025;
    // if ((mb_state->left_velocity + tolerance) >= mb_setpoints->fwd_velocity && (mb_state->left_velocity - tolerance) <= mb_setpoints->fwd_velocity)
    // {
    //     rc_filter_reset(&left_wheel_velocity_i);
    // }
    // if ((mb_state->right_velocity + tolerance) >= mb_setpoints->fwd_velocity && (mb_state->right_velocity - tolerance) <= mb_setpoints->fwd_velocity)
    // {
    //     rc_filter_reset(&right_wheel_velocity_i);
    // }

    // ****PID+FF, reset I (Choose option 1,2,or3 above)****
    // mb_state->left_cmd = rc_filter_march(&left_wheel_velocity_pd, left_error) + rc_filter_march(&left_wheel_velocity_i, left_error) + ff_left_cmd;
    // mb_state->right_cmd = rc_filter_march(&right_wheel_velocity_pd, right_error) + rc_filter_march(&right_wheel_velocity_i, right_error) + ff_right_cmd;
    
    // ****PID, reset I (Choose option 1,2,or3 above)****
    // mb_state->left_cmd = rc_filter_march(&left_wheel_velocity_pd, left_error) + rc_filter_march(&left_wheel_velocity_i, left_error);
    // mb_state->right_cmd = rc_filter_march(&right_wheel_velocity_pd, right_error) + rc_filter_march(&right_wheel_velocity_i, right_error);
    




    //Calculate error for adding low pass filter:
    float lpf_fwdvel = rc_filter_march(&fwd_velocity_lpf, mb_setpoints->fwd_velocity);
    float lpf_turnvel = rc_filter_march(&turn_velocity_lpf, (mb_setpoints->turn_velocity * WHEEL_BASE / 2));
    float lpf_leftvel = rc_filter_march(&left_wheel_velocity_lpf, mb_state->left_velocity);
    float lpf_rightvel = rc_filter_march(&right_wheel_velocity_lpf, mb_state->right_velocity);
    float left_error_lpf = lpf_fwdvel - lpf_turnvel - lpf_leftvel;
    float right_error_lpf = lpf_fwdvel + lpf_turnvel - lpf_rightvel;

    //Feedforward with LPF commands:
    float ff_left_cmd_lpf = left_slope * (lpf_fwdvel - lpf_turnvel) + left_intercept;
    float ff_right_cmd_lpf = right_slope * (lpf_fwdvel + lpf_turnvel) + right_intercept;
    
    //****PID with LPF****
    // mb_state->left_cmd = rc_filter_march(&left_wheel_velocity_pid, left_error_lpf);
    // mb_state->right_cmd = rc_filter_march(&right_wheel_velocity_pid, right_error_lpf);

    //****Feedfoward with PID with LPF(on fwd and left/right vel)****
    mb_state->left_cmd = rc_filter_march(&left_wheel_velocity_pid, left_error_lpf) + ff_left_cmd_lpf;
    mb_state->right_cmd = rc_filter_march(&right_wheel_velocity_pid, right_error_lpf) + ff_right_cmd_lpf;

    
    


    // Maintain PWM thresshold between -1 to 1
    if (mb_state->left_cmd > LEFT_PWM_MAX)
    {
        mb_state->left_cmd = LEFT_PWM_MAX;
    }
    else if (mb_state->left_cmd < -LEFT_PWM_MAX)
    {
        mb_state->left_cmd = -LEFT_PWM_MAX;
    }

    if (mb_state->right_cmd > RIGHT_PWM_MAX)
    {
        mb_state->right_cmd = RIGHT_PWM_MAX;
    }
    else if (mb_state->right_cmd < -RIGHT_PWM_MAX)
    {
        mb_state->right_cmd = -RIGHT_PWM_MAX;
    }

    //Prevent bot from rolling away
    if (mb_setpoints->fwd_velocity + mb_setpoints->turn_velocity == 0)
    {
        mb_state->right_cmd = 0;
        mb_state->left_cmd = 0;
    }

    return 0;
}


/*******************************************************************************
* int mb_destroy_controller()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_destroy_controller(){
    //T3add:
    rc_filter_free(&left_wheel_velocity_pid);
    rc_filter_free(&right_wheel_velocity_pid);
    rc_filter_free(&left_wheel_velocity_lpf);
    rc_filter_free(&right_wheel_velocity_lpf);
    rc_filter_free(&turn_velocity_lpf);
    rc_filter_free(&fwd_velocity_lpf);
    rc_filter_free(&left_wheel_velocity_pd);
    rc_filter_free(&right_wheel_velocity_pd);
    rc_filter_free(&left_wheel_velocity_i);
    rc_filter_free(&right_wheel_velocity_i);
    //1.5
    rc_filter_free(&fwd_velocity_pid);
    rc_filter_free(&turn_velocity_pid);
    rc_filter_free(&left_velocity_des_lpf);
    rc_filter_free(&right_velocity_des_lpf);
    return 0;
}
