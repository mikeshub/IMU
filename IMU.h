#ifndef IMU_h
#define IMU_h

#include <CONFIG.h>
#include <Arduino.h>
#include <L3G2.h>
#include <I2C.h>
#include <LSM3032.h>
#include <ADXL345.h>
#include <digitalWriteFast.h>
#include <SPI.h>

#define ToRad(x) ((x) * 0.01745329252)  // *pi/180
#define ToDeg(x) ((x) * 57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f


#define GYRO_SCALE 0.07f

#define betaDef		0.1f
//#define betaDefY		0.2f

class IMU{
	public:

		void init(void);
		void GetIMUData(void);
		void GetAcc(void);
		void GetMag(void);
		void GetGyro(void);
		void GYROupdate(float*);
		void AHRSupdate(float*);
		void IMUupdate(float*);
		float invSqrt(float );
		void GetEuler(void);
		struct{
			int16_t accX;
			int16_t accY;
			int16_t accZ;
			int16_t magX;
			int16_t magY;
			int16_t magZ;
			int16_t gyroX;
			int16_t gyroY;
			int16_t gyroZ;

		}raw;
		struct{
			float accX;
			float accY;
			float accZ;
			float magX;
			float magY;
			float magZ;
			float gyroX;
			float gyroY;
			float gyroZ;
		}scaled;
		struct{
			float accX;
			float accY;
			float accZ;
		}smoothed;
		struct{
			float pitchAct;
			float rollAct;
			float pitch;
			float roll;
			float yaw;
		}angles;



		LSM3032 compass;
		L3G2 gyro;
		ADXL345 accel;
		float q0;
		float q1;
		float q2;
		float q3;
		float beta;
		float betaY;
		float fastAtan2( float , float );
	private:
		void Smoothing(float*, float*);
		void Smoothing(int16_t*, float*);

		struct{
			int32_t gyroX;
			int32_t gyroY;
			int32_t gyroZ;
		}sum;
		struct{
			int16_t gyroX;
			int16_t gyroY;
			int16_t gyroZ;
			float pitch;
			float roll;
		}offset;
		float magnitude;
		float xSquared;
		float ySquared;
		float zSquared;
		boolean safeAcc;

		int i;//GP loop index
};


#endif