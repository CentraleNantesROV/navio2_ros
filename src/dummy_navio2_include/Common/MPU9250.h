#ifndef NAVIO2_DUMMY_MPU9250_H
#define NAVIO2_DUMMY_MPU9250_H

struct InertialSensor
{
  inline bool probe() {return true;}
  inline void update() {}
  inline void initialize() {}
  inline void setGyroOffset() {}
  inline void read_accelerometer(float*,float*,float* z) {*z = -9.81;}
  inline void read_gyroscope(float*,float*,float*) {}
  inline void read_magnetometer(float*,float*,float*) {}
};

struct MPU9250 : public InertialSensor {};

#endif // NAVIO2_DUMMY_MPU9250_H
