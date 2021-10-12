#ifndef NAVIO2_DUMMY_RCOUTPUT_NAVIO2_H
#define NAVIO2_DUMMY_RCOUTPUT_NAVIO2_H

struct RCOutput_Navio2
{
  inline bool initialize(int) {return true;}
  inline void set_frequency(int, int) {}
  inline bool enable(int) {return true;}
  inline void set_duty_cycle(int,int) {}
};

#endif // NAVIO2_DUMMY_RCOUTPUT_NAVIO2_H
