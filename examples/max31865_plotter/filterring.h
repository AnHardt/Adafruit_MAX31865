#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif



class Filter{
public:
Filter(uint8_t len);  
void fill(float new_value);
void length(uint8_t len);
float request(void);
private:
uint8_t _len;
float _ring[256] = { 0 };
uint8_t _ringIndex;
bool _ringFull;
uint8_t _loopEnd;
};
