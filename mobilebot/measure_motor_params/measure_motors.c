/*******************************************************************************
* measure_motor_params.c
*   Complete this code to automatically measure motor parameters
*   or print out data to be namalyzed in numpy
* 
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/encoder.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/motor.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"

//global variables
float enc2meters = (WHEEL_DIAMETER * M_PI) / (GEAR_RATIO * ENCODER_RES);
float a[20][3];

/*******************************************************************************
* float ticks_to_speed() 
* Converts encoder counts to speed in m/s
*******************************************************************************/
float ticks_to_speed(int ticks, float enc2meters, float dt){
    return fabs(ticks * enc2meters / dt);
}

/*******************************************************************************
* void calibrate motors()
* Collects data for duty cycle vs. speed for a motor
*******************************************************************************/
void calibrate_motors(int curMotor, int mot_polarity){
    int row = 0;
    for (float pwm = 0.0; pwm <= 1.0; pwm += 0.05){
        rc_motor_set(curMotor, mot_polarity*pwm);
        rc_nanosleep(1E9);
        rc_encoder_eqep_write(curMotor,0);
        rc_nanosleep(1E9);
        int ticks = rc_encoder_eqep_read(curMotor);
        float speed = ticks_to_speed(ticks, enc2meters, 1.0);
        printf("%f, %f\n", pwm, speed);
        a[row][0] = pwm;
        a[row++][curMotor] = speed;
    }
    rc_motor_set(curMotor,0.0);
    return;
}

/*******************************************************************************
* write_to_csv()
* Writes the calibration data to a CSV file for analysis with Numpy and plotting
*******************************************************************************/
void write_to_csv(char *filename, int m){
 
    printf("\n Creating %s.csv file", filename);
    FILE *fp;
    int i;
    filename = strcat(filename, ".csv");
    fp = fopen(filename, "w+");
    fprintf(fp,"PWM, Left motor speed, Right motor speed\n");
    for(i = 0; i < m; i++){
      fprintf(fp, "%f, %f, %f\n", a[i][0], a[i][1], a[i][2]);
    }
    fclose(fp);
    printf("\n %s file created",filename);
    return;
}

/*******************************************************************************
* int main() 
* Motor Calibration Code
*******************************************************************************/
int main(int argc, char **argv){
    if (argc != 2) {
        fprintf(stderr, "usage: %s <your name>\n", argv[0]);
        return 1;
    }
    char *filename = NULL;
    filename = argv[1];
    strcat(filename, "_motor_calibration");
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
    }

#if defined(MRC_VERSION_1v3) || defined(MRC_VERSION_2v1)
    if(mb_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze mb_motors\n");
        return -1;
    }
#endif

#if defined(BEAGLEBONE_BLUE)
    if(rc_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze motors\n");
        return -1;
    }
#endif

    if(rc_encoder_eqep_init()<0){
        fprintf(stderr,"ERROR: failed to initialze encoders\n");
        return -1;
    }
    
    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);

	if(rc_get_state()==RUNNING){
	rc_nanosleep(1E9); //sleep for 1s
        //TODO: write routine here
        //Calibrate left motor
        calibrate_motors(LEFT_MOTOR, LEFT_MOTOR_POL);
        rc_nanosleep(2E9); 
        //Calibrate right motor
        calibrate_motors(RIGHT_MOTOR, RIGHT_MOTOR_POL);
        rc_nanosleep(1E9);
        //write to CSV file
        write_to_csv(filename,20);
        //Change state to exit the loop
        rc_set_state(EXITING);
	}
	
	// TODO: Plase exit routine here
    //Cleanup
    rc_encoder_eqep_cleanup();
    rc_motor_cleanup();
    // remove pid file LAST
	rc_remove_pid_file();   
	return 0;
}
