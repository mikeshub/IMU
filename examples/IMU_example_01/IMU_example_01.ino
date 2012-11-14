#include <IMU.h>
#include <L3G2.h>
#include <I2C.h>
#include <LSM3032.h>
#include <Streaming.h>
#include <ADXL345.h>
#include <CONFIG.h>
#include <SPI.h>

IMU imu;

long timer;
float G_Dt;

int loopCount;

void setup(){
  Serial.begin(115200);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);  
  I2c.begin();
  I2c.setSpeed(1);
  loopCount = 0;
  imu.init();
  timer = micros();
}

void loop(){

  if (micros() - timer >= 2500){
    G_Dt = ((micros() - timer) / 1000000.0);
    timer = micros();


    if ((loopCount % 8) == 0){
      if (loopCount == 40){
        imu.GetIMUData();
        imu.AHRSupdate(&G_Dt);
        loopCount = 0;
      }
      else{
        imu.GetAcc();
        imu.GetGyro();
        imu.IMUupdate(&G_Dt);
        loopCount++;
      }
    }
    else{
      imu.GetGyro();
      imu.GYROupdate(&G_Dt);
      loopCount++;
    }

    imu.GetEuler();
    Serial<<millis()<<","<<imu.angles.pitch<<","<<imu.angles.roll<<","<<imu.angles.yaw<<"\r\n";

  }
}


