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
#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7
#define RTD_C -4.183e-12

#define RTD_0 -242.02
#define RTD_1 2.2228
#define RTD_2 2.5859e-3
#define RTD_3 -4.8260e-6
#define RTD_4 -2.8183e-8
#define RTD_5 1.5243e-10

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

  float temperature(float RTDnominal, float refResistor);
  float temperature(void);
  uint16_t readRTD();

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
