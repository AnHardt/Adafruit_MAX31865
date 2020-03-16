#include "filterring.h"

Filter::Filter(uint8_t len) {
  _len = len;
  _ringIndex = 0;
  _ringFull = false;
  _loopEnd = 0;
}

void Filter::fill(float newVal){
  _ring[_ringIndex] = newVal;                                     // replace the oldest value
  _loopEnd = (_ringFull) ? _len : _ringIndex;                 // set the number of filled elements
  _ringIndex++;  ;
  if ( _ringIndex > _len ) { _ringFull = true; _ringIndex = 0; } // close the ring
}

float Filter::request(void) {
  float ringSum = 0.0f;
  for (uint8_t i = 0;  i < _loopEnd; i++) ringSum += _ring[i];  // summ up the values
  return ringSum / (max(_loopEnd, 1.0f)); // don't devide by 0 if request() is caled before the first call of fill().
}

void Filter::length(uint8_t len) {
  _len = len;
  _ringIndex = 0;
  _ringFull = false;
  _loopEnd = 0;
}
