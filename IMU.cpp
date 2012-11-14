// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms

#include <IMU.h>
#include <math.h>
#include <CONFIG.h>
#include <Streaming.h>


void IMU::init(void){
	long timer;
	float G_Dt;
	gyro.init();
  	compass.init();
  	accel.init();



	gyro.writeReg(L3G2_CTRL_REG1, 0xCF);
	gyro.writeReg(L3G2_CTRL_REG2, 0x00);
	gyro.writeReg(L3G2_CTRL_REG3, 0x00);
	gyro.writeReg(L3G2_CTRL_REG4, 0x20); //
	gyro.writeReg(L3G2_CTRL_REG5, 0x02);


	compass.writeMagReg(0x00, 0x1C);
	compass.writeMagReg(0x01, 0x60);
    compass.writeMagReg(0x02, 0x00);
	beta = betaDef;

	//betaPR = betaDefPR;
	//betaY = betaDefY;
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;

	for(i=0;i<64;i++){
		GetAcc();
		GetGyro();
		delayMicroseconds(5000);
	}
	for(i=0;i<512;i++){
		GetAcc();//to init the smoothing filter
		GetGyro();
		//Serial<<gyro.g.x<<","<<gyro.g.y<<","<<gyro.g.z<<"\r\n";
		sum.gyroX += gyro.g.x;
		sum.gyroY += gyro.g.y;
		sum.gyroZ += gyro.g.z;
		delayMicroseconds(5000);
	}
	offset.gyroX = sum.gyroX >> 9;
	offset.gyroY = sum.gyroY >> 9;
	offset.gyroZ = sum.gyroZ >> 9;
	timer = micros();
	delay(5);
	for ( i =0; i < 1000; i++){
	    G_Dt = ((micros() - timer) / 1000000.0);
	    timer = micros();
	    GetIMUData();
	    AHRSupdate(&G_Dt);
	    delay(5);
  	}
  	GetEuler();
  	offset.pitch = angles.pitchAct;
  	offset.roll = angles.rollAct;
	//Serial<<offset.gyroX<<","<<offset.gyroY<<","<<offset.gyroZ<<"\r\n";


}
void IMU::GYROupdate(float *dt){
	static float qDot1, qDot2, qDot3, qDot4;
	static float recipNorm;
	static float gx;
	static float gy;
	static float gz;
	gx = ToRad(scaled.gyroX);
	gy = ToRad(scaled.gyroY);
	gz = ToRad(scaled.gyroZ);
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * *dt;
    q1 += qDot2 * *dt;
    q2 += qDot3 * *dt;
    q3 += qDot4 * *dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
  q3 *= recipNorm;
}
void IMU::IMUupdate(float *dt) {
  static float gx;
  static float gy;
  static float gz;
  static float ax;
  static float ay;
  static float az;

  static float recipNorm;
  static float s0, s1, s2, s3;
  static float qDot1, qDot2, qDot3, qDot4;
  static float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	gx = ToRad(scaled.gyroX);
  	gy = ToRad(scaled.gyroY);
	gz = ToRad(scaled.gyroZ);

	ax = -1.0 * smoothed.accX;
	ay = -1.0 * smoothed.accY;
	az = -1.0 * smoothed.accZ;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  magnitude = sqrt(smoothed.accX * smoothed.accX + smoothed.accY * smoothed.accY + smoothed.accZ * smoothed.accZ);
  if ((magnitude > 384) || (magnitude < 128)){
	  //Serial<<magnitude<<", bad acc IMU\r\n";
	  ax = 0;
	  ay = 0;
	  az = 0;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * *dt;
  q1 += qDot2 * *dt;
  q2 += qDot3 * *dt;
  q3 += qDot4 * *dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void IMU::AHRSupdate(float *dt) {
  static float gx;
  static float gy;
  static float gz;
  static float ax;
  static float ay;
  static float az;
  static float mx;
  static float my;
  static float mz;


  static float recipNorm;
  static float s0, s1, s2, s3;
  static float qDot1, qDot2, qDot3, qDot4;
  static float hx, hy;
  static float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	gx = ToRad(scaled.gyroX);
  	gy = ToRad(scaled.gyroY);
	gz = ToRad(scaled.gyroZ);

	ax = -1.0 * smoothed.accX;
	ay = -1.0 * smoothed.accY;
	az = -1.0 * smoothed.accZ;

	mx = raw.magX;
	my = raw.magY;
	mz = raw.magZ;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  magnitude = sqrt(smoothed.accX * smoothed.accX + smoothed.accY * smoothed.accY + smoothed.accZ * smoothed.accZ);
  if ((magnitude > 384) || (magnitude < 128)){
  	  //Serial<<magnitude<<", bad acc AHRS\r\n";
  	  ax = 0;
  	  ay = 0;
  	  az = 0;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {


    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    //Serial<<recipNorm<<"\r\n";
    //recipNorm = 1.0 / sqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    //Serial<<gx<<","<<gy<<","<<gz<<","<<ax<<","<<ay<<","<<az<<","<<mx<<","<<my<<","<<mz<<"\r\n";
    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;



    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  /*q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);*/
  q0 += qDot1 * *dt;
  q1 += qDot2 * *dt;
  q2 += qDot3 * *dt;
  q3 += qDot4 * *dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void IMU::GetEuler(void){
  angles.rollAct = ToDeg(fastAtan2(2 * (q0 * q1 + q2 * q3),1 - 2 * (q1 * q1 + q2 * q2)));
  angles.pitchAct = ToDeg(asin(2 * (q0 * q2 - q3 * q1)));
  angles.pitch = angles.pitchAct - offset.pitch;
  angles.roll = angles.rollAct - offset.roll;
  angles.yaw = ToDeg(fastAtan2(2 * (q0 * q3 + q1 * q2) , 1 - 2* (q2 * q2 + q3 * q3)));
  if (angles.yaw < 0){
    angles.yaw +=360;
  }

}

void IMU::GetIMUData(void){
	GetAcc();
	GetGyro();
	GetMag();

}
void IMU::GetAcc(){
	accel.read();

	Smoothing(&accel.accX,&smoothed.accX);
	Smoothing(&accel.accY,&smoothed.accY);
	Smoothing(&accel.accZ,&smoothed.accZ);


}
void IMU::GetMag(){
	compass.readMag();
	raw.magX = compass.m.x;
	raw.magY = compass.m.y;
	raw.magZ = compass.m.z;

}
void IMU::GetGyro(){
	gyro.read();

	scaled.gyroX = GYRO_SCALE * (gyro.g.x - offset.gyroX);
	scaled.gyroY = GYRO_SCALE * (gyro.g.y - offset.gyroY);
	scaled.gyroZ = GYRO_SCALE * (gyro.g.z - offset.gyroZ);

}

/*void IMU::Smoothing(float *raw, float *smooth){
	*smooth = (*raw * (0.10)) + (*smooth * 0.90);
}
*/
void IMU::Smoothing(int16_t *raw, float *smooth){
	*smooth = (*raw * (0.10)) + (*smooth * 0.90);
}



float IMU::fastAtan2( float y, float x)
{
  static float atan;
  static float z;
  if ( x == 0.0f )
  {
    if ( y > 0.0f ) return PIBY2_FLOAT;
    if ( y == 0.0f ) return 0.0f;
    return -PIBY2_FLOAT;
  }
  atan;
  z = y / x;
  if ( fabs( z ) < 1.0f )
  {
    atan = z/(1.0f + 0.28f*z*z);
    if ( x < 0.0f )
    {
      if ( y < 0.0f ) return atan - PI_FLOAT;
      return atan + PI_FLOAT;
    }
  }
  else
  {
    atan = PIBY2_FLOAT - z/(z*z + 0.28f);
    if ( y < 0.0f ) return atan - PI_FLOAT;
  }
  return atan;
}

float IMU::invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}
