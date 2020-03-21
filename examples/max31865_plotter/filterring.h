#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#define MAXRINGLENGTH 128

class Filter{
public:
Filter(uint8_t len);  
void fill(float new_value);
void length(uint8_t len);
float request(void);
private:
uint8_t _len;
float _ring[MAXRINGLENGTH + 1] = { 0 };
uint8_t _ringIndex;
bool _ringFull;
uint8_t _loopEnd;
};
