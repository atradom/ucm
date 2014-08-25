/* barebones arducopter for learning and experimentation
 * atr 7/30/14
 */
 
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_InertialSensor.h>


#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <AP_GPS.h>

#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <AP_Terrain.h>
#include <Filter.h>
#include <SITL.h>
#include <AP_Buffer.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>


// *atr* added for pixhawk
#include <AP_NavEKF.h>
#include <AP_HAL_PX4.h>
#include <AP_Scheduler.h>       // main loop scheduler

#include <AP_Motors.h>          // AP Motors library
#include <AP_Curve.h>

#include <RC_Channel.h>     // RC Channel Library


// Y6 motor mapping
// motor_num, roll_fac, pitch_fac, yaw_fac, 						test order, frame position
//        1, -1.0,		0.666, 		AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2,			bottom right
//        2,  1.0,		0.666, 		AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5,			top left
//        3,  1.0,		0.666, 		AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6,			bottom left
//        4,  0.0,		-1.333, 	AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4,			bottom rear
//        5, -1.0,		0.666, 		AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1,			top right
//        6,  0.0,		-1.333, 	AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3,			top rear



// const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;  // Hardware abstraction layer

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


// *atr* INS (Inertial Navigation System/sensors) for Pixhawk
AP_InertialSensor_PX4 ins;

// *atr Baro for Pixhawk
static AP_Baro_PX4 baro;

// *atr* compass for Pixhawk
static AP_Compass_PX4 compass;

// gps
AP_GPS gps;

// *atr* Inertial Navigation EKF
#if AP_AHRS_NAVEKF_AVAILABLE
AP_AHRS_NavEKF ahrs(ins, baro, gps);
#else
AP_AHRS_DCM ahrs(ins, baro, gps);
#endif


RC_Channel rc1(0), rc2(1), rc3(2), rc4(3), rc5(4), rc6(5), rc7(6), rc8(7), *rc = &rc1;

// uncomment the row below depending upon what frame you are using
//AP_MotorsTri	motors(rc1, rc2, rc3, rc4)
//AP_MotorsQuad   motors(rc1, rc2, rc3, rc4);
//AP_MotorsHexa	motors(rc1, rc2, rc3, rc4);
AP_MotorsY6	motors(rc1, rc2, rc3, rc4);
//AP_MotorsOcta	motors(rc1, rc2, rc3, rc4);
//AP_MotorsOctaQuad	motors(rc1, rc2, rc3, rc4);
//AP_MotorsHeli	motors(rc1, rc2, rc3, rc4);


// this function sets up the copter and is run once at bootup
void setup() 
{


#ifdef APM2_HARDWARE
    // we need to stop the barometer from holding the SPI bus
    hal.gpio->pinMode(40, HAL_HAL_GPIO_OUTPUT);
    hal.gpio->write(40, HIGH);
#endif

    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ);
    ins.init_accel();

    ahrs.init();

    if( compass.init() ) {
        hal.console->printf("Enabling compass\n");
        ahrs.set_compass(&compass);
    } else {
        hal.console->printf("No compass detected\n");
    }
    gps.init(NULL);

	
	// motor initialisation
    motors.set_update_rate(490);						//490Hz
    // motors.set_frame_orientation(AP_MOTORS_X_FRAME);
    motors.set_frame_orientation(AP_MOTORS_PLUS_FRAME);
    motors.set_min_throttle(130);
    motors.set_mid_throttle(500);
    motors.Init();      // initialise motors
    motors.enable();
 
    motors.output_min();

	// calibrate the radio
//	setup_radio();
	
	// my radio calibration is not working, so trying it manually
	rc1.radio_min=967;
	rc1.radio_max=2062;
	rc2.radio_min=970;
	rc2.radio_max=2063;
	rc3.radio_min=968;
	rc3.radio_max=2060;
	rc4.radio_min=968;
	rc4.radio_max=2062;
	print_radio_values();

    // set rc channel ranges
    rc1.set_angle(4500);		// roll is in centidegrees (so we get +/- 45 degree roll command)
    rc2.set_angle(4500);		// pitch is +/- 45 degrees
    rc3.set_range(130, 1000);	// throttle
    rc4.set_angle(4500);		// yaw is +/- 45 degrees
	print_radio_values();

	   motors.armed(true);		//arm the motors
}


// this is the main loop
void loop() 
{
	int16_t roll_in, pitch_in, yaw_in, throttle_in;
    static uint16_t counter;
    static uint32_t last_t, last_print, last_compass;
    uint32_t now = hal.scheduler->micros();
    float heading = 0;
    
	if (last_t == 0) {
		last_t = now;
		return;
    }
    last_t = now;

    if (now - last_compass > 100*1000UL &&
        compass.read()) {
        heading = compass.calculate_heading(ahrs.get_dcm_matrix());
        // read compass at 10Hz
        last_compass = now;
#if WITH_GPS
        g_gps->update();
        hal.console->printf_P("gps.update\n");
#endif
    }

    ahrs.update();
    counter++;

    if (now - last_print >= 100000 /* 100ms : 10hz */) {
        Vector3f drift  = ahrs.get_gyro_drift();
        hal.console->printf("dr: %4d dp: %4d dy: %4d dt: %4d,   r:%4.1f  p:%4.1f y:%4.1f  hdg=%.1f rate=%.1f  \n" ,
			rc1.control_in,
			rc2.control_in,
			rc4.control_in,
			rc3.control_in,
			ToDeg(ahrs.roll),
			ToDeg(ahrs.pitch),
			ToDeg(ahrs.yaw),
			compass.use_for_yaw() ? ToDeg(heading) : 0.0,
			(1.0e6*counter)/(now-last_print));			
        
        
        last_print = now;
        counter = 0;
    }
    
    
	read_radio();				// read the radio
//	print_pwm();				// debug function to show the pwm values
	
	roll_in=rc1.control_in;
	pitch_in=rc2.control_in;
	throttle_in=rc3.control_in;
	yaw_in=rc4.control_in;
	motors.set_pitch(pitch_in);
    motors.set_roll(roll_in);
    motors.set_yaw(yaw_in);
    motors.set_throttle(throttle_in);
    motors.output();

	//hal.scheduler->delay(50);  //Wait 50ms 
}


// radio reader
void read_radio()
{
    rc1.set_pwm(hal.rcin->read(CH_1));
    rc2.set_pwm(hal.rcin->read(CH_2));
    rc3.set_pwm(hal.rcin->read(CH_3));
    rc4.set_pwm(hal.rcin->read(CH_4));
    rc5.set_pwm(hal.rcin->read(CH_5));
    rc6.set_pwm(hal.rcin->read(CH_6));
    rc7.set_pwm(hal.rcin->read(CH_7));
    rc8.set_pwm(hal.rcin->read(CH_8));
}

void setup_radio(void)
{
	hal.console->println("\n\nRadio Setup:");
	uint8_t i;
	int16_t value;
	
	for(i = 0; i < 100;i++){
		hal.scheduler->delay(20);
		read_radio();
	}
		
	rc1.radio_min = rc1.radio_in;
	rc2.radio_min = rc2.radio_in;
	rc3.radio_min = rc3.radio_in;
	rc4.radio_min = rc4.radio_in;
	rc5.radio_min = rc5.radio_in;
	rc6.radio_min = rc6.radio_in;
	rc7.radio_min = rc7.radio_in;
	rc8.radio_min = rc8.radio_in;

	rc1.radio_max = rc1.radio_in;
	rc2.radio_max = rc2.radio_in;
	rc3.radio_max = rc3.radio_in;
	rc4.radio_max = rc4.radio_in;
	rc5.radio_max = rc5.radio_in;
	rc6.radio_max = rc6.radio_in;
	rc7.radio_max = rc7.radio_in;
	rc8.radio_max = rc8.radio_in;

	rc1.radio_trim = rc1.radio_in;
	rc2.radio_trim = rc2.radio_in;
	rc4.radio_trim = rc4.radio_in;
	// 3 is not trimed
	rc5.radio_trim = 1500;
	rc6.radio_trim = 1500;
	rc7.radio_trim = 1500;
	rc8.radio_trim = 1500;
			
	hal.console->println("\nMove all controls to each extreme. Hit S to save:");
	while(1){
		
		hal.scheduler->delay(20);
		read_radio();

		rc1.update_min_max();
		rc2.update_min_max();
		rc3.update_min_max();
		rc4.update_min_max();
		rc5.update_min_max();
		rc6.update_min_max();
		rc7.update_min_max();
		rc8.update_min_max();
		
        if(hal.console->available() > 0) {
			value = hal.console->read();
			if (value == 's' || value == 'S') {
            hal.console->println("Radio calibrated, Showing control values:");
            break;     
			}
        }
    }
    return;
}


void print_radio_values()
{
    for (int i=0; i<8; i++) {
	     hal.console->printf("CH%u: %u|%u\n",
			  (unsigned)i+1, 
			  (unsigned)rc[i].radio_min, 
			  (unsigned)rc[i].radio_max); 
    }
}


void print_pwm()
{
    for (int i=0; i<3; i++) {
	    hal.console->printf("ch%u: %4d ", (unsigned)i+1, (int)rc[i].control_in);
    }
    hal.console->printf("\n");
}



AP_HAL_MAIN();    // special macro that replace's one of Arduino's to setup the code (e.g. ensure loop() is called in a loop).
