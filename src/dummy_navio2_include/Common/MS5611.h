#ifndef NAVIO2_DUMMY_MS5611_H
#define NAVIO2_DUMMY_MS5611_H

struct MS5611
{
  inline void initialize() {}

  inline void refreshPressure() {}
  inline void refreshTemperature() {}

  inline void readPressure() {}
  inline void readTemperature() {}
  inline void calculatePressureAndTemperature() {}

  inline float getTemperature() {return 25.f;}
  inline float getPressure() {return 1015.f;};
};


#endif // NAVIO2_DUMMY_MS5611_H
