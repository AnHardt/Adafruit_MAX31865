/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#pragma once

// MAX31865
#define MAX31865_CONFIG_REG 0x00
#define MAX31865_CONFIG_BIAS 0x80
#define MAX31865_CONFIG_MODEAUTO 0x40
#define MAX31865_CONFIG_MODEOFF 0x00
#define MAX31865_CONFIG_1SHOT 0x20
#define MAX31865_CONFIG_3WIRE 0x10
#define MAX31865_CONFIG_24WIRE 0x00
#define MAX31865_CONFIG_FAULTDETCYCLE 0x84
#define MAX31865_CONFIG_FAULTSTAT 0x02
#define MAX31865_CONFIG_FILT50HZ 0x01
#define MAX31865_CONFIG_FILT60HZ 0x00

#define MAX31865_RTDMSB_REG 0x01
#define MAX31865_RTDLSB_REG 0x02
#define MAX31865_HFAULTMSB_REG 0x03
#define MAX31865_HFAULTLSB_REG 0x04
#define MAX31865_LFAULTMSB_REG 0x05
#define MAX31865_LFAULTLSB_REG 0x06
#define MAX31865_FAULTSTAT_REG 0x07

#define MAX31865_FAULT_HIGHTHRESH 0x80
#define MAX31865_FAULT_LOWTHRESH 0x40
#define MAX31865_FAULT_REFINLOW 0x20
#define MAX31865_FAULT_REFINHIGH 0x10
#define MAX31865_FAULT_RTDINLOW 0x08
#define MAX31865_FAULT_OVUV 0x04

// PT100
/*
 * We have options to calculate Temperatures from Resistancys
 * LINEAR is best siutible for temperatures in the range of [0 - 100]°C 
 *   is not very exact at 0 and 100°C but has an error of +0.38°C @ 50°C.
 *   Its the fastest of the floating pont methods.
 * LINEAR_B is similar to LINEAR but adds an other factor. Reaching ±0.19°C
 * error over the whole range.
 * All the following float metodes are 'exact' for temperatures above 0°C
 *   but will need ingreasing times for calculating negative temperatures
 * POLY2 (+0.075°C/–0.17°C), 
 * POLY3 (+0.0053°C/–0.0085°C), 
 * POLY4 (+0.0022°C/–0.001°C) and 
 * POLY5 (+0.0001°C/–0.00005°C).
 * Because the formular for the positive range delivers exacter results
 *   down to a BORDER-temperature (-75.5, -12.5, -8.75, 0.0)°C the formulsrs
 *   for the negative temperatures are not used above the borders listed
 *   in the RTD_*_BORDER resistancys.
 * If you don't want to measure negative temperatures below of that borders
 *   define PT_NO_NEGATIVES to save some memory for code. For the use with
 *   3D-Printers this is my prefered way.
 * Above is base on 
 * https://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
 * and 
 * http://www.mosaic-industries.com/embedded-systems/microcontroller-projects/temperature-measurement/platinum-rtd-sensors/resistance-calibration-table
 * and
 * https://github.com/drhaney/pt100rtd
 * 
 * The second and third link tells us about a 'rational polynomial function' promissing
 *   even less error over (±0.015°C)the full range -200 - 850°C. The calculation is
 *   even more expensive. In my tests this did not reach that low errors - so id call
 *   this 'experimental'. PT_RATIONAL (Likely the factors are calculated for a PT-sensor
 *   with an other ALPHA.)
 * 
 * 
 * A completely different approach with the INTEGER define. Here the temperatures are
 *   interpolated from a lookup table. This is fast, saves the code for the floating
 *   point math (if not used elsewhere) but needs a considerable amount of memory
 *   for the table and binary search. Here a fixed point calculation is used. All
 *   resistances are multiplyed by 128 and all temperatures by 128 to get better
 *   resolutions. The metods to use then have names like `temperature128()` or
 *   `setMaxTemp128()`.
 * The code for producing that table is compiled in with the INTEGER_TABLE option.
 *   Deactivate it for normal use.
 * 
 * If more then one option is defined the more exact, slower one is used. The INTEGER
 *   option can live in parallel with the float options. The PT_INTEGER_TABLE option
 *   does use some of the float options.
 */
//#define PT_LINEAR
//#define PT_LINEAR_B
#define PT_CVD

#define PT_POLY2
#define PT_POLY3
#define PT_POLY4
#define PT_POLY5
#define PT_RATIONAL

#define PT_INTEGER
#define PT_INTEGER_TABLE

// ALL following RTD* constants are only true for PT-sensors with this ALPHA
#define RTD_ALPHA 0.00385
#define RTD_BETA -0.19

// CVD
#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7
#define RTD_C -4.183e-12

#define RTD_2_BORDER 72.1
#define RTD_2_0 -242.97
#define RTD_2_1 2.2838
#define RTD_2_2 1.4727e-3

#define RTD_3_BORDER 95.1
#define RTD_3_0 -242.09
#define RTD_3_1 2.2276
#define RTD_3_2 2.5178e-3
#define RTD_3_3 -5.8620e-6

#define RTD_4_BORDER 96.6
#define RTD_4_0 -241.96
#define RTD_4_1 -2.2163
#define RTD_4_2 2.8541e-3
#define RTD_4_3 -9.9121e-6
#define RTD_4_4 -1.7052e-8

#define RTD_5_BORDER 0.0
#define RTD_5_0 -242.02
#define RTD_5_1 2.2228
#define RTD_5_2 2.5859e-3
#define RTD_5_3 -4.8260e-6
#define RTD_5_4 -2.8183e-8
#define RTD_5_5 1.5243e-10

#define RTD_R_BORDER 0.0
#define RTD_R_0 -245.19
#define RTD_R_1 2.5293
#define RTD_R_2 -0.066046
#define RTD_R_3 4.0422e-3
#define RTD_R_4 -2.0697e-6
#define RTD_R_5 -0.025422
#define RTD_R_6 1.6883e-3
#define RTD_R_7 -1.3601e-6

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_SPIDevice.h>
#include <SPI.h>

typedef enum max31865_numwires {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;

/*! Interface class for the MAX31865 RTD Sensor reader */
class Adafruit_MAX31865 {
public:

  Adafruit_MAX31865(int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso, int8_t spi_clk, int8_t ready_pin = -1);
  Adafruit_MAX31865(int8_t spi_cs, SPIClass *theSPI, int8_t ready_pin = -1);
  Adafruit_MAX31865(int8_t spi_cs, int8_t ready_pin = -1, SPIClass *theSPI = &SPI);

  bool begin(max31865_numwires_t x = MAX31865_2WIRE);
  bool begin(uint8_t configuration, float R0 = 100.0f, float Rref = 430.0f, float R2wire = 0.0f);

  float temperature(void);
  uint16_t readRTD(void);

  bool checkFault(void);
  uint8_t readFault(void);
  uint8_t readFault(bool b);
  void clearFault(void);

  void setWires(max31865_numwires_t wires);
  void set3Wires(bool b);
  void autoConvert(bool b);
  void enableBias(bool b);
  void enable50Hz(bool b);

  void setMaxTemp(float C);
  float getMaxTemp(void);
  void setMinTemp(float C);
  float getMinTemp(void);

  void setT0Resistance(float R0) {_R0 = R0; _Z3 = (4 * RTD_B) / _R0;}
  void setTrefResiatance(float Rref) {_Rref = Rref; _ratioToRfactor = _Rref / ((1u << 15) - 1.0f);}

  #ifdef PT_RATIONAL
    float RtoC_rational(float R);
  #endif
  #ifdef PT_POLY5
    float RtoC_poly5(float R);
  #endif
  #ifdef PT_POLY4
    float RtoC_poly4(float R);
  #endif
  #ifdef PT_POLY3
    float RtoC_poly3(float R);
  #endif
  #ifdef PT_POLY2
    float RtoC_poly2(float R);
  #endif
  #ifdef PT_CVD
    float RtoC_cvd(float R);
  #endif
  #ifdef PT_LINEAR_B
    float RtoC_linearB(float R);
  #endif
  #ifdef PT_LINEAR
    float RtoC_linear(float R);
  #endif

private:
  // The pins
  int8_t _ready_pin;
  // states
  bool _fault = false;
  uint8_t _configuration = 0;
  // constants
  Adafruit_SPIDevice _spi_dev;
  Adafruit_BusIO_Register _config_reg;
  Adafruit_BusIO_Register _rRatio_reg;
  Adafruit_BusIO_Register _maxRratio_reg;
  Adafruit_BusIO_Register _minRratio_reg;
  Adafruit_BusIO_Register _fault_reg;

  float _R0 = 100.0f;
  float _Rref = 430.0f;
  float _ratioToRfactor = _Rref / ((1u << 15) - 1.0f);
  float _R2wire = 0.0f;

  const float _Z1 = -RTD_A;
  const float _Z2 = RTD_A * RTD_A - (4 * RTD_B);
        float _Z3 = (4 * RTD_B) / _R0;
  const float _Z4 = 2 * RTD_B;

  // functions
  uint8_t modifyConfig(uint8_t mask, bool set);

  float CtoR(float C);
  uint16_t RtoRatio(float R);
  void setMaxRatio(uint16_t r);
  void setMinRatio(uint16_t r);
  uint16_t getMaxRatio(void);
  uint16_t getMinRatio(void);
  float RatioToR(uint16_t r);
  float RtoC(float R);
};
