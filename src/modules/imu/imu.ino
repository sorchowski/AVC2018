// MPU9250-specific code is from https://github.com/sparkfun/MPU-9250_Breakout

#include "quaternionFilters.h"
#include "MPU9250.h"

// Generally, one or the other should be true while the other false; not true at the same time.
#define SerialDebug false // Set to true to get Serial output for debugging

#define GS_PER_METSECSQ 0.10197162129779

// 10 times per second
#define SAMPLE_RATE 100

MPU9250 myIMU;

//ros::Publisher imuPublisher(avc_common::ROS_TOPIC_IMU, &imu_message);
//ros::Publisher magPublisher(avc_common::ROS_TOPIC_MAG, &mag_message);

void setup()
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed

  Serial.begin(57600);

  if (SerialDebug) {
    Serial.println("hello");
  }

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (SerialDebug) {
    Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
    Serial.print(" I should be "); Serial.println(0x71, HEX);
  }

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);

    if (SerialDebug) {
      Serial.println("MPU9250 is online...");
      Serial.print("x-axis self test: acceleration trim within : ");
      Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
      Serial.print("y-axis self test: acceleration trim within : ");
      Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
      Serial.print("z-axis self test: acceleration trim within : ");
      Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
      Serial.print("x-axis self test: gyration trim within : ");
      Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
      Serial.print("y-axis self test: gyration trim within : ");
      Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
      Serial.print("z-axis self test: gyration trim within : ");
      Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");
    }

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    if (SerialDebug) {
      Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
      Serial.print(" I should be "); Serial.println(0x48, HEX);
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer

    if (SerialDebug)
    {
      Serial.println("AK8963 initialized for active data mode....");
      Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }

  } // if (c == 0x71)
  else
  {
    if (SerialDebug) {
      Serial.print("Could not connect to MPU9250: 0x");
      Serial.println(c, HEX);
    }

    error(); // Go into error mode and hold steady
  }
}

void loop()
{

  // Not sure of the following two comments:
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  myIMU.delt_t = millis() - myIMU.count;

  // update based on the following rate
  if (myIMU.delt_t > SAMPLE_RATE)
  {
    myIMU.tempCount = myIMU.readTempData();  // Read the adc values
    // Temperature in degrees Centigrade
    myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;

    // in Gs. need to convert to meters/sec^2 for ros message
    float linear_acceleration_x = myIMU.ax*GS_PER_METSECSQ;
    float linear_acceleration_y = myIMU.ay*GS_PER_METSECSQ;
    float linear_acceleration_z = myIMU.az*GS_PER_METSECSQ;
    //float linear_acceleration_covariance[0] = -1;

    // deg/s. need to convert to radians for ros message
    float angular_velocity_x = myIMU.gx*DEG_TO_RAD;
    float angular_velocity_y = myIMU.gy*DEG_TO_RAD;
    float angular_velocity_z = myIMU.gz*DEG_TO_RAD;
    //float angular_velocity_covariance = -1;

    // in mG
    float mx = myIMU.mx;
    float my = myIMU.my;
    float mz = myIMU.mz;

    // q0: *getQ()
    // qx: *(getQ() + 1);
    // qy: *(getQ() + 2);
    // qz: *(getQ() + 3);

    float orientation_x = *(getQ() + 1);
    float orientation_y = *(getQ() + 2);
    float orientation_z = *(getQ() + 3);
    float orientation_w = *(getQ());

    String lin_acc_x = String(linear_acceleration_x);
    String lin_acc_y = String(linear_acceleration_y);
    String lin_acc_z = String(linear_acceleration_z);

    String ang_vel_x = String(angular_velocity_x);
    String ang_vel_y = String(angular_velocity_y);
    String ang_vel_z = String(angular_velocity_z);

    String orient_x = String(orientation_x);
    String orient_y = String(orientation_y);
    String orient_z = String(orientation_z);
    String orient_w = String(orientation_w);

    Serial.print(lin_acc_x+","+lin_acc_y+","+lin_acc_z+","+ang_vel_x+","+ang_vel_y+","+ang_vel_z+","+orient_x+","+orient_y+","+orient_z+","+orient_w);

    /*
    myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                  *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
    myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                  *(getQ()+2)));
    myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                  *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
    myIMU.pitch *= RAD_TO_DEG;
    myIMU.yaw   *= RAD_TO_DEG;
    // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
    // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    myIMU.yaw   -= 8.5;
    myIMU.roll  *= RAD_TO_DEG;

    // IMU values available at this point if desired
    //myIMU.yaw
    //myIMU.pitch
    //myIMU.roll
    */

    myIMU.count = millis();
    myIMU.sum = 0;
  } // if (myIMU.delt_t > 500)
}


void error() {
  pinMode(LED_BUILTIN, OUTPUT);
  while(1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
}

