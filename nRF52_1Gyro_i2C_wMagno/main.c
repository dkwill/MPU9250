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
float roll, pitch,yaw;


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
    p_mpu_config.smplrt_div = 0;   // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_2G; // Set accelerometer full scale range to 2G
    p_mpu_config.gyro_config.fs_sel = GFS_1000DPS;   
	  ret_code = mpu_config(&p_mpu_config); // Configure the MPU with above values
    APP_ERROR_CHECK(ret_code); // Check for errors in return value 
	
	// Enable magnetometer
	  mpu_magn_config_t magnetometer_config;
	  magnetometer_config.mode = CONTINUOUS_MEASUREMENT_8Hz_MODE;//SELF_TEST_MODE;//SINGLE_MEASUREMENT_MODE;//CONTINUOUS_MEASUREMENT_100Hz_MODE;
    ret_code = mpu_magnetometer_init(&magnetometer_config);
    APP_ERROR_CHECK(ret_code); // Check for errors in return value

}
	
int main(void)
{    
	 
	 uint32_t err_code;
  
	// LEDS_CONFIGURE(LEDS_MASK);
	// LEDS_OFF(LEDS_MASK);
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
					
					AX = ((float)CAX)/16384.00;
          AY = ((float)CAY)/16384.00; //16384 is just 32768/2 to get our 1G value
          AZ = ((float)CAZ-16384)/16384.00; //remove 1G before dividing
					
					GX = ((float)CGX)/32.768; //32.768 is just 32768/2000 to get us our 1deg/sec value
          GY = ((float)CGY)/32.768;
          GZ = ((float)CGZ)/32.768; 
			
					ax = AX;
					ay = AY;
					az = AZ;
					gx = GX;
					gy = GY;
					gz = GZ;  
					mx = CMX;
					my = CMY;
					mz = CMZ;  //TODO - Do Magnetometer values need to be normalized like Accel and Gyro?
					
					
					MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz);
					
				 // printf("AX:%.3f \t, AY:%.3f \t, AZ:%.3f \t|| GX:%.3f \t, GY:%.3f \t, GZ:%.3f,\t\r\n", ax,ay,az,gx,gy,gz);
				 yaw = atan2(2*q1*q2-2*q0*q3,2*q0*q0+2*q1*q1-1)*180/PI;
         pitch = -1*asin(2*q1*q3+2*q0*q2)*180/PI;
         roll = atan2(2*q2*q3-2*q0*q1,2*q0*q0+2*q3*q3-1)*180/PI;
				 
				 printf("roll:%.3f \t, pitch:%.3f \t, yaw:%.3f \t|| MX:%.3f \t, MY:%.3f \t, MZ:%.3f,\t\r\n", roll,pitch,yaw, mx,my,mz);
			
			    nrf_delay_ms(5);
					
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
            gx = 0.0f,
            gy = 0.0f,
            gz = 0.0f,
        }; // literal values should be replaced with sensor measurements

        const FusionVector3 accelerometer = {
            ax = 0.0f,
            ay = 0.0f,
            az = 1.0f,
        }; // literal values should be replaced with sensor measurements

        const FusionVector3 magnetometer = {
            mx = 1.0f,
            my = 0.0f,
            mz = 0.0f,
        }; // literal values should be replaced with sensor measurements

        FusionAhrsUpdate(&fusionAhrs, gyroscope, accelerometer, magnetometer, 0.01f); // assumes 100 Hz sample rate
      //  FusionAhrsUpdate(&fusionAhrs, gyroscope, accelerometer, FUSION_VECTOR3_ZERO, 0.01f); // alternative function call to ignore magnetometer
     //   FusionAhrsUpdate(&fusionAhrs, gyroscope, FUSION_VECTOR3_ZERO, FUSION_VECTOR3_ZERO, 0.01f); // alternative function call to ignore accelerometer and magnetometer

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
            ax = 0.0f,
            ay = 0.0f,
            az = 1.0f,
        }; // literal values should be replaced with sensor measurements

        const FusionVector3 magnetometer = {
            mx = 1.0f,
            my = 0.0f,
            mz = 0.0f,
        }; // literal values should be replaced with sensor measurements

        const float heading = FusionCompassCalculateHeading(accelerometer, magnetometer);
        if (heading == 0) {
        } // avoid warning for unused variable
    }

		  //NEED TO OUTPUT EULER ANGLES HERE FOR GYRO ONLY
		
		//See FusionTypes.h
	   
   
				
		eulerAngles.angle.roll = FUSION_RADIANS_TO_DEGREES(atan2f(Q.y * Q.z - Q.w * Q.x, qwSquaredMinusHalf + Q.z * Q.z));
		
		//roll = FusionEulerAngles(roll);
		//FusionEulerAngles(pitch);
	//	FusionEulerAngles(yaw);
		
		//printf("cgx %d", CGX);
		
		//printf("CMX %d", CGX);
		
		
   // printf("Success!\r\n");
	//	nrf_delay_ms(25);
}
}					
	

  





