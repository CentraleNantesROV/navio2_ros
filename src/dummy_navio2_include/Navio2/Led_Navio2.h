#ifndef NAVIO2_DUMMY_LED_NAVIO2_H
#define NAVIO2_DUMMY_LED_NAVIO2_H

enum class Colors {
    Black,
    Red,
    Green,
    Blue,
    Cyan,
    Magenta,
    Yellow,
    White};

struct Led_Navio2
{
public:
    inline Led_Navio2() {}
    inline bool initialize() {return true;}
    inline void setColor(Colors) {}
};

#endif // NAVIO2_DUMMY_LED_NAVIO2_H
