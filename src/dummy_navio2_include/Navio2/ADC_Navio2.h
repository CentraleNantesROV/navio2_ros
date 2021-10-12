#ifndef NAVIO2_DUMMY_ADC_NAVIO2_H
#define NAVIO2_DUMMY_ADC_NAVIO2_H

struct ADC
{
  inline int get_channel_count() {return 6;}
  inline int read(int) {return 0;}
  inline void initialize() {}
};

struct ADC_Navio2 : public ADC {};

#endif // NAVIO2_DUMMY_ADC_NAVIO2_H