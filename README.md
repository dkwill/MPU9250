This project is setup with i2c and initially used Madgwick open source to get accelerometer, gyroscope and magnetometer readings. Code returns those values; however, they are relative to the starting position of the sensor. The application will not have the benefit of being initalized relative to a global position. 

The "Fusion" files are open source and it appears the Fusion.h contains the code to convert the quaternions to Euler angles at the end of the file. 

Speed (Samples per Second) is not critical for this application so 50-100 Hz should be fine. I prefer the open source vs. Invensense motion driver code since there is another similar application that needs to run at ~400HZ which is beyond the capabilities of the Invensense provided solutions. 

Once the values are obtained irrespective to the starting position, Phase II will be to transfer these values to an iOS device. 

Note - This project builds and runs using an nRF52DK using SDK 11.0. The project was compiled with Keil.
