// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple test for the AP_AHRS interface
//

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <AP_GPS.h>

// *atr* added for pixhawk
#include <AP_NavEKF.h>
#include <AP_HAL_PX4.h>
#include <AP_Scheduler.h>       // main loop scheduler

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

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>


const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


// *atr* INS for Pixhawk
AP_InertialSensor_PX4 ins;

// *atr Baro for Pixhawk
static AP_Baro_PX4 baro;

// INS and Baro declaration
//#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
//AP_InertialSensor_MPU6000 ins;
//AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
//#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
//AP_ADC_ADS7844 adc;
//AP_InertialSensor_Oilpan ins( &adc );
//AP_Baro_BMP085 baro;
//#else
//AP_InertialSensor_HIL ins;
//#endif

//AP_Compass_HMC5843 compass;

// *atr* compass for Pixhawk
static AP_Compass_PX4 compass;

AP_GPS gps;

// choose which AHRS system to use
//AP_AHRS_DCM  ahrs(ins, baro, gps);

// *atr* Inertial Navigation EKF
#if AP_AHRS_NAVEKF_AVAILABLE
AP_AHRS_NavEKF ahrs(ins, baro, gps);
#else
AP_AHRS_DCM ahrs(ins, baro, gps);
#endif




AP_Baro_HIL barometer;


#define HIGH 1
#define LOW 0

void setup(void)
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
}

void loop(void)
{
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
        hal.console->printf_P(
                PSTR("r:%4.1f  p:%4.1f y:%4.1f "
                    "drift=(%5.1f %5.1f %5.1f) hdg=%.1f rate=%.1f\n"),
                        ToDeg(ahrs.roll),
                        ToDeg(ahrs.pitch),
                        ToDeg(ahrs.yaw),
                        ToDeg(drift.x),
                        ToDeg(drift.y),
                        ToDeg(drift.z),
                        compass.use_for_yaw() ? ToDeg(heading) : 0.0,
                        (1.0e6*counter)/(now-last_print));
        last_print = now;
        counter = 0;
    }
}

AP_HAL_MAIN();
