/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */


#include <stdio.h>
#include "boards.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_mpu.h"
#include "math.h"
#include "Fusion.h"
#include "FusionTypes.h"

#define MPU_TWI_SCL_PIN 3
#define MPU_TWI_SDA_PIN 4

//UART buffer size.
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

int16_t CAX, CAY, CAZ; //current acceleration values
int16_t CGX, CGY, CGZ; //current gyroscope values
int16_t CMX, CMY, CMZ; //current magnetometer values
int16_t CT; 

float AX, AY, AZ; //acceleration floats
float GX, GY, GZ; //gyroscope floats
float MX, MY, MZ; //magnetometer floats
float yaw, pitch, roll;

static void uart_events_handler(app_uart_evt_t *p_event)
{
    switch (p_event->evt_type)
    {
    case APP_UART_COMMUNICATION_ERROR:
        APP_ERROR_HANDLER(p_event->data.error_communication);
        break;

    case APP_UART_FIFO_ERROR:
        APP_ERROR_HANDLER(p_event->data.error_code);
        break;

    default:
        break;
    }
}


// @brief UART initialization.

static void uart_config(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
        {
            RX_PIN_NUMBER,
            TX_PIN_NUMBER,
            RTS_PIN_NUMBER,
            CTS_PIN_NUMBER,
            APP_UART_FLOW_CONTROL_DISABLED,
            false,
            UART_BAUDRATE_BAUDRATE_Baud115200};

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_events_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

void mpu_setup(void)
{
    ret_code_t ret_code;
    // Initiate MPU driver
    ret_code = mpu_init();
    APP_ERROR_CHECK(ret_code); // Check for errors in return value

    // Setup and configure the MPU with intial values
    mpu_config_t p_mpu_config;// = MPU_DEFAULT_CONFIG(); // Load default values
    p_mpu_config.smplrt_div = 9;                      // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_2G;       // Set accelerometer full scale range to 2G
    p_mpu_config.gyro_config.fs_sel = GFS_1000DPS;
    ret_code = mpu_config(&p_mpu_config); // Configure the MPU with above values
    APP_ERROR_CHECK(ret_code);            // Check for errors in return value

    // Enable magnetometer
    mpu_magn_config_t magnetometer_config;
		magnetometer_config.mode = CONTINUOUS_MEASUREMENT_100Hz_MODE; //;//SINGLE_MEASUREMENT_MODE;//CONTINUOUS_MEASUREMENT_100Hz_MODE;
    
		ret_code = mpu_magnetometer_init(&magnetometer_config);
    APP_ERROR_CHECK(ret_code); // Check for errors in return value
}

/**
 * ReadAccelGyroMag - reads the accelerometer, gyro and mag value from the board.
 * Updates the following  global variable 
 * 
 * CAX, CAY, CAZ; //current acceleration values
 * CGX, CGY, CGZ; //current gyroscope values
 * CMX, CMY, CMZ; //current magnetometer values
 * CT; 
 * AX, AY, AZ; //acceleration floats
 * GX, GY, GZ; //gyroscope floats
 * MX, MY, MZ; //magnetometer floats
 */

void ReadAccelGyroMag()
{
    uint32_t err_code;
    accel_values_t acc_values;
    gyro_values_t gyr_values;
    magn_values_t magn_values;

    // Read accelerometer sensor values
    err_code = mpu_read_accel(&acc_values);
    APP_ERROR_CHECK(err_code);

    CAX = acc_values.x;
    CAY = acc_values.y;
    CAZ = acc_values.z;

    // Read gyro sensor values
    err_code = mpu_read_gyro(&gyr_values);
    APP_ERROR_CHECK(err_code);

    CGX = gyr_values.x;
    CGY = gyr_values.y;
    CGZ = gyr_values.z;

    // Read magnetometer values
    err_code = mpu_read_magnetometer(&magn_values, NULL);
    APP_ERROR_CHECK(err_code);

    MX = (float)magn_values.x;
    MY = (float)magn_values.y;
    MZ = (float)magn_values.z;
		
    //Normalize from full-scale 
    AX = ((float)CAX) / 16384.00;
    AY = ((float)CAY) / 16384.00; //16384 is 32768/2 to get our 1G value
    AZ = ((float)CAZ) / 16384.00; //To remove 1G, subtract 16834 before dividing

    GX = ((float)CGX) / 32.768; //32.768 is 32768/1000 to get us our 1deg/sec value
    GY = ((float)CGY) / 32.768;
    GZ = ((float)CGZ) / 32.768;

         
        //Accelerometer
        //printf("AX: %.3f \t AY %.3f \t AZ %.3f \t\r\n", AX,AY,AZ); 
        //nrf_delay_ms(25);
        
        //Gyro
        //printf("GX: %.3f \t GY %.3f \t GZ %.3f \t\r\n", GX,GY,GZ); 
        //nrf_delay_ms(25);
        
			//Magnetometer
			//printf("MX: %.3f \t MY %.3f \t MZ %.3f \t\r\n", MX,MY,MZ); 
			//nrf_delay_ms(25);

} // end of ReadAccelGyroMag

int main(void)
{
    FusionVector3 gyroscope;
		FusionVector3 accelerometer;	
    FusionVector3 magnetometer;
           
    uart_config();
    mpu_setup();
    mpu_init();
  

    // Initialises the AHRS algorithm with application settings
    // the gain, minMagneticField and maxMagneticField can be changed. 
    FusionAhrs fusionAhrs;
    FusionAhrsInitialise(&fusionAhrs, 0.5f, 20.0f, 70.0f); // orig. 0.5f, valid magnetic field defined as 20 uT to 70 uT

    FusionBias fusionBias;
    FusionBiasInitialise(&fusionBias, 50, 0.01f); // assumes 100 Hz sample rate

    ReadAccelGyroMag();

    while (1)
    {
			
        // assign the values that were read from the board.
        gyroscope.axis.x = GX;
        gyroscope.axis.y = GY; 
        gyroscope.axis.z = GZ; 		

        accelerometer.axis.x = AX;
        accelerometer.axis.y = AY;
        accelerometer.axis.z = AZ;
                
        magnetometer.axis.x = MX;
        magnetometer.axis.y = MY;
        magnetometer.axis.z = MZ;
                
     
        // Call the function to update the algorithm with the latest sensor measurements. .01 = 100 Hz
        FusionAhrsUpdate(&fusionAhrs, gyroscope, accelerometer, magnetometer, 0.01f); 
               
        // Initialises the bias correction algorithm with application settings.      
           
        FusionBiasUpdate(&fusionBias, GX, GY, GZ); 
        
        const FusionVector3 earthAcceleration = FusionAhrsCalculateEarthAcceleration(&fusionAhrs);
        if (earthAcceleration.array[0] == 0)
        {
        } // avoid warning for unused variable

        FusionQuaternion fqc =  FusionQuaternionConjugate(fusionAhrs.quaternion);
     
        FusionEulerAngles fea =  FusionQuaternionToEulerAngles(fqc);
        roll = fea.angle.roll;
        pitch = fea.angle.pitch;
        yaw = fea.angle.yaw;

             	
      //  const FusionRotationMatrix rotationMatrix = FusionQuaternionToRotationMatrix(fqc);

       // if (rotationMatrix.array[0] == 0)
      //  {
      //  }             
	   
				printf("Roll: %.3f \t Pitch %.3f \t Yaw %.3f \t\r\n", roll,pitch,yaw); 
				nrf_delay_ms(5);

        // read sensor information.
        ReadAccelGyroMag();
			 
			
    } // end loop

} //end of main

 
