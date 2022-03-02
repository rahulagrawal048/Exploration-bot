/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include "../mobilebot/mobilebot.h"
#include "mb_defs.h"
#include <math.h>

#define PI 3.14159265358979323846

/*******************************************************************************
* mb_initialize_odometry() 
*
* TODO: initialize odometry
* NOTE: you should initialize from Optitrack data if available
*
*******************************************************************************/
void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta){
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->theta = theta;
}


/*******************************************************************************
* mb_update_odometry() 
*
* TODO: calculate odometry from internal variables
*       publish new odometry to lcm ODOMETRY_CHANNEL
*
*******************************************************************************/
void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
    float left_phi_delta = (2 * PI * mb_state->left_encoder_delta) / (ENCODER_RES * GEAR_RATIO);
    float right_phi_delta = (2 * PI * mb_state->right_encoder_delta) / (ENCODER_RES * GEAR_RATIO);
    float left_s_delta = (WHEEL_DIAMETER / 2) * left_phi_delta;
    float right_s_delta = (WHEEL_DIAMETER / 2) * right_phi_delta;
    //float theta_delta = (left_s_delta - right_s_delta) / WHEEL_BASE;
    float theta_delta = (right_s_delta - left_s_delta) / WHEEL_BASE; //ZB change
    float distance_delta = (left_s_delta + right_s_delta) / 2;
    float x_delta = distance_delta * cos(mb_odometry->theta + theta_delta / 2);
    float y_delta = distance_delta * sin(mb_odometry->theta + theta_delta / 2);
    mb_odometry->x += x_delta;
    mb_odometry->y += y_delta;
    
    //Gyrodometry
    float thresh = 0.003;
    float imuAngleChange = mb_state->tb_angles[2] - mb_state->last_yaw;
    if(fabs(imuAngleChange - theta_delta) > thresh) {
        mb_odometry->theta += imuAngleChange;
    }
    else{
        mb_odometry->theta = mb_clamp_radians(mb_odometry->theta + theta_delta);
    }

    return;
}


/*******************************************************************************
* mb_clamp_radians() 
* clamp an angle from -PI to PI
*******************************************************************************/
float mb_clamp_radians(float angle){

    if(angle < -2.0*PI)
    {
        for(; angle < -2.0*PI; angle += 2.0*PI);
    }
    else if(angle > 2.0*PI)
    {
        for(; angle > 2.0*PI; angle -= 2.0*PI);
    }

    return angle;
}


/*******************************************************************************
* mb_angle_diff_radians() 
* computes difference between 2 angles and wraps from -PI to PI
*******************************************************************************/
float mb_angle_diff_radians(float angle1, float angle2){
    float diff = angle2 - angle1;
    while(diff < -PI) diff+=2.0*PI;
    while(diff > PI) diff-=2.0*PI;
    return diff;
}
