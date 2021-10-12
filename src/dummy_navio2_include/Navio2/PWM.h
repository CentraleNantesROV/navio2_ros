#ifndef NAVIO2_DUMMY_PWM_H
#define NAVIO2_DUMMY_PWM_H

struct PWM {
  inline PWM() {}

  inline bool init(unsigned int) {return true;}
  inline bool enable(unsigned int) {return true;}
  inline bool set_period(unsigned int, unsigned int) {return true;}
  inline bool set_duty_cycle(unsigned int, float) {return true;}
};

#endif // NAVIO2_DUMMY_PWM_H
