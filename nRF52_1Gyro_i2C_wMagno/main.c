 /* 
  * This example is not extensively tested and only 
  * meant as a simple explanation and for inspiration. 
  * NO WARRANTY of ANY KIND is provided. 
  */

#include <stdio.h>
#include "boards.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_mpu.h"
#include "math.h"
#include "Madgwick.h"
#include "Fusion.h"
#include "FusionTypes.h"

#define MPU_TWI_SCL_PIN 3
#define MPU_TWI_SDA_PIN 4

/*UART buffer size. */
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

#define numberOfTests   25

   
int16_t CAX, CAY, CAZ; //current acceleration values
int16_t CGX, CGY, CGZ; //current gyroscope values
int16_t CMX, CMY, CMZ; //current magnetometer values
int16_t CT;            //current temperature
   
float   AXoff, AYoff, AZoff; //accelerometer offset values
float   GXoff, GYoff, GZoff; //gyroscope offset values
float   MXoff, MYoff, MZoff; //magnetometer offset values

float   AX, AY, AZ; //acceleration floats
float   GX, GY, GZ; //gyroscope floats
float 	MX, MY, MZ; //magnetometer floats
float   yaw, pitch, roll;

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
float PI = 3.14159265358979323846f;

float gx,gy,gz,ax,ay,az,mx,my,mz; //Added by DKW 3/31/15

bool stopMPU;

static void uart_events_handler(app_uart_evt_t * p_event)
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

/**
 * @brief UART initialization.
 * Just the usual way. Nothing special here
 */
static void uart_config(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

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
    mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); // Load default values
    p_mpu_config.smplrt_div = 19;   // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_2G; // Set accelerometer full scale range to 2G
  //  p_mpu_config.gyro_config.fs_sel = GFS_2000DPS;   
	  ret_code = mpu_config(&p_mpu_config); // Configure the MPU with above values
    APP_ERROR_CHECK(ret_code); // Check for errors in return value 
	
	// Enable magnetometer
	  mpu_magn_config_t magnetometer_config;
	  magnetometer_config.mode = CONTINUOUS_MEASUREMENT_8Hz_MODE;//SELF_TEST_MODE;//SINGLE_MEASUREMENT_MODE;//CONTINUOUS_MEASUREMENT_100Hz_MODE;
    ret_code = mpu_magnetometer_init(&magnetometer_config);
    APP_ERROR_CHECK(ret_code); // Check for errors in return value

}

void gyro_offset(accel_values_t acc_values, gyro_values_t gyr_values, magn_values_t magn_values)
	{	
		uint32_t err_code;
		int i;
		
	for(i=0; i<numberOfTests; i++)
    {
      printf("Test Number: %d \r\n", i);
      
      err_code = mpu_read_accel(&acc_values);
      APP_ERROR_CHECK(err_code);
			
			CAX = acc_values.x;
			CAY = acc_values.y;
			CAZ = acc_values.z;			
            			
			err_code = mpu_read_gyro(&gyr_values);
      APP_ERROR_CHECK(err_code);
			
			CGX = gyr_values.x;
			CGY = gyr_values.y;
			CGZ = gyr_values.z;	
			
	// Read magnetometer values
      err_code = mpu_read_magnetometer(&magn_values,NULL);
      APP_ERROR_CHECK(err_code);
	
			CMX = magn_values.x;
      CMY = magn_values.y;
	   	CMZ = magn_values.z;	

      AXoff += CAX;
      AYoff += CAY;
      AZoff += CAZ;			
      
			GXoff += CGX;
      GYoff += CGY;
      GZoff += CGZ;
			
			MXoff += CMX;
      MYoff += CMY;
      MZoff += CMZ;
       
     // printf("AX:%d \t AY:%d \t AZ:%d \t || GX:%d \t GY:%d \t GZ:%d,\t\r\n", CAX,CAY,CAZ,CGX,CGY,CGZ,CMX,CMY,CMZ);
			 printf("GX:%d \t GY:%d \t GZ:%d \t || MX:%d \t MY:%d \t MZ:%d,\t\r\n", CGX,CGY,CGZ,CMX,CMY,CMZ);
			 nrf_delay_ms(25);
  }
   
    AXoff = AXoff/numberOfTests;
    AYoff = AYoff/numberOfTests;
    AZoff = AZoff/numberOfTests;
    GXoff = GXoff/numberOfTests;
    GYoff = GYoff/numberOfTests;
    GZoff = GZoff/numberOfTests;
  	MXoff = MXoff/numberOfTests;
    MYoff = MYoff/numberOfTests;
    MZoff = MZoff/numberOfTests;
    
  //  printf ("\n\nTest finished, offset values are shown below\n\n\r");
    printf("AXoff:%d \t, AYoff:%d \t, AZoff:%d \t || GXoff:%d \t, GYoff:%d \t, GZoff:%d \r\n", (int)AXoff, (int)AYoff, (int)AZoff,(int)GXoff,(int)GYoff,(int)GZoff);
  //  uint32_t sample_number = 0;
	}	
	

int main(void)
{    
	 
	 uint32_t err_code;
  
	 LEDS_CONFIGURE(LEDS_MASK);
	 LEDS_OFF(LEDS_MASK);
    uart_config();
    mpu_setup();
	  mpu_init(); 
	        
  	accel_values_t acc_values;
	  gyro_values_t gyr_values;
		magn_values_t magn_values;
	  
						
    while(1)
    {
				
			 			
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
          err_code = mpu_read_magnetometer(&magn_values,NULL);
          APP_ERROR_CHECK(err_code);
					
					CMX = magn_values.x;
			    CMY = magn_values.y;
		    	CMZ = magn_values.z;	
					
					 // FusionTypes.h example tests
    {
        const FusionVector3 vector3 = FUSION_VECTOR3_ZERO;
        if (vector3.array[0] == 0) {
        } // avoid warning for unused variable

        const FusionQuaternion quaternion = FUSION_QUATERNION_IDENTITY;
        if (quaternion.array[0] == 0) {
        } // avoid warning for unused variable

        const FusionRotationMatrix rotationMatrix = FUSION_ROTATION_MATRIX_IDENTITY;
        if (rotationMatrix.array[0] == 0) {
        } // avoid warning for unused variable

        const FusionEulerAngles eulerAngles = FUSION_EULER_ANGLES_ZERO;
        if (eulerAngles.array[0] == 0) {
        } // avoid warning for unused variable

        {
            const FusionQuaternion quaternion = FUSION_QUATERNION_IDENTITY;
            const FusionRotationMatrix rotationMatrix = FusionQuaternionToRotationMatrix(quaternion);
            if (rotationMatrix.array[0] == 0) {
            } // avoid warning for unused variable
        }
        {
            const FusionQuaternion quaternion = FUSION_QUATERNION_IDENTITY;
            const FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(quaternion);
            if (eulerAngles.array[0] == 0) {
            } // avoid warning for unused variable
        }
    }

    // FusionAhrs.c example tests
    {
        FusionAhrs fusionAhrs;
        FusionAhrsInitialise(&fusionAhrs, 0.5f, 20.0f, 70.0f); // valid magnetic field defined as 20 uT to 70 uT

        const FusionVector3 gyroscope = {
            CGX = 0.0f,
            CGY = 0.0f,
            CGZ = 0.0f,
        }; // literal values should be replaced with sensor measurements

        const FusionVector3 accelerometer = {
            CAX = 0.0f,
            CAY = 0.0f,
            CAZ = 1.0f,
        }; // literal values should be replaced with sensor measurements

        const FusionVector3 magnetometer = {
            CMX = 1.0f,
            CMY = 0.0f,
            CMZ = 0.0f,
        }; // literal values should be replaced with sensor measurements

        FusionAhrsUpdate(&fusionAhrs, gyroscope, accelerometer, magnetometer, 0.01f); // assumes 100 Hz sample rate
        FusionAhrsUpdate(&fusionAhrs, gyroscope, accelerometer, FUSION_VECTOR3_ZERO, 0.01f); // alternative function call to ignore magnetometer
        FusionAhrsUpdate(&fusionAhrs, gyroscope, FUSION_VECTOR3_ZERO, FUSION_VECTOR3_ZERO, 0.01f); // alternative function call to ignore accelerometer and magnetometer

        const FusionVector3 earthAcceleration = FusionAhrsCalculateEarthAcceleration(&fusionAhrs);
        if (earthAcceleration.array[0] == 0) {
        } // avoid warning for unused variable

        if (FusionAhrsIsInitialising(&fusionAhrs) == true) {
            // AHRS algorithm is initialising
        } else {
            // AHRS algorithm is not initialising
        }

        FusionAhrsReinitialise(&fusionAhrs);

        FusionAhrsZeroYaw(&fusionAhrs);
    }

    // FusionBias.c example tests
    {
        FusionBias fusionBias;
        FusionBiasInitialise(&fusionBias, 50, 0.01f); // assumes 100 Hz sample rate

        FusionBiasUpdate(&fusionBias, 0, 0, 0); // literal values should be replaced with sensor measurements

        if (FusionBiasIsActive(&fusionBias) == true) {
            // Bias correction algorithm is active
        } else {
            // Bias correction algorithm is not active
        }
    }

    // FusionCompass.c example tests
    {
        const FusionVector3 accelerometer = {
            CAX = 0.0f,
            CAY = 0.0f,
            CAZ = 1.0f,
        }; // literal values should be replaced with sensor measurements

        const FusionVector3 magnetometer = {
            CMX = 1.0f,
            CMY = 0.0f,
            CMZ = 0.0f,
        }; // literal values should be replaced with sensor measurements

        const float heading = FusionCompassCalculateHeading(accelerometer, magnetometer);
        if (heading == 0) {
        } // avoid warning for unused variable
    }

		  //NEED TO OUTPUT EULER ANGLES HERE FOR GYRO ONLY
		
		//See FusionTypes.h
	   
   	//	FusionEulerAngles.angle.roll;
		
		
 //   printf("Success!\r\n");
	//	nrf_delay_ms(25);
}
}					
	

  





