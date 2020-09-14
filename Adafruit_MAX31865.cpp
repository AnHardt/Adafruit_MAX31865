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

#include "Adafruit_MAX31865.h"
#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif

#include <SPI.h>
#include <stdlib.h>

/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI
    @param spi_cs the SPI CS pin to use
    @param spi_mosi the SPI MOSI pin to use
    @param spi_miso the SPI MISO pin to use
    @param spi_clk the SPI clock pin to use
    @param ready_pin the pin where the modules DRDY output is connected to.
*/
/**************************************************************************/

Adafruit_MAX31865::Adafruit_MAX31865(int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso, int8_t spi_clk, int8_t ready_pin) :
  _spi_dev( spi_cs, spi_clk, spi_miso, spi_mosi, 5000000UL, SPI_BITORDER_MSBFIRST, SPI_MODE1),
  _config_reg(&_spi_dev, MAX31865_CONFIG_REG, ADDRBIT8_HIGH_TOWRITE, 1, MSBFIRST),
  _rRatio_reg(&_spi_dev, MAX31865_RTDMSB_REG, ADDRBIT8_HIGH_TOWRITE, 2, MSBFIRST),
  _maxRratio_reg(&_spi_dev, MAX31865_HFAULTMSB_REG, ADDRBIT8_HIGH_TOWRITE, 2, MSBFIRST),
  _minRratio_reg(&_spi_dev, MAX31865_LFAULTMSB_REG, ADDRBIT8_HIGH_TOWRITE, 2, MSBFIRST),
  _fault_reg(&_spi_dev, MAX31865_FAULTSTAT_REG, ADDRBIT8_HIGH_TOWRITE, 1, MSBFIRST)
{
  _ready_pin = ready_pin;
}

/**************************************************************************/
/*!
    @brief Create the interface object using hardware SPI
    @param spi_cs the SPI CS pin to use along with the default SPI device
    @param theSPI the processors SPI device the module is connected to (if it has more than one)
    @param ready_pin the pin where the modules DRDY output is connected to.
*/
/**************************************************************************/

  Adafruit_MAX31865::Adafruit_MAX31865(int8_t spi_cs, int8_t ready_pin, SPIClass *theSPI) :
  Adafruit_MAX31865(spi_cs, theSPI, ready_pin)
{}


Adafruit_MAX31865::Adafruit_MAX31865(int8_t spi_cs, SPIClass *theSPI, int8_t ready_pin) :
  _spi_dev( spi_cs, 5000000UL, SPI_BITORDER_MSBFIRST, SPI_MODE1, theSPI),
  _config_reg(&_spi_dev, MAX31865_CONFIG_REG, ADDRBIT8_HIGH_TOWRITE, 1, MSBFIRST),
  _rRatio_reg(&_spi_dev, MAX31865_RTDMSB_REG, ADDRBIT8_HIGH_TOWRITE, 2, MSBFIRST),
  _maxRratio_reg(&_spi_dev, MAX31865_HFAULTMSB_REG, ADDRBIT8_HIGH_TOWRITE, 2, MSBFIRST),
  _minRratio_reg(&_spi_dev, MAX31865_LFAULTMSB_REG, ADDRBIT8_HIGH_TOWRITE, 2, MSBFIRST),
  _fault_reg(&_spi_dev, MAX31865_FAULTSTAT_REG, ADDRBIT8_HIGH_TOWRITE, 1, MSBFIRST)
{
  _ready_pin = ready_pin;
}

/**************************************************************************/
/*!
    @brief Initialize the SPI interface and set the number of RTD wires used
    @param wires The number of wires in enum format. Can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @return True if selfest detected no rerrors
*/
/**************************************************************************/
bool Adafruit_MAX31865::begin(max31865_numwires_t wires) {
  return begin((wires == MAX31865_3WIRE) ? MAX31865_CONFIG_3WIRE : MAX31865_CONFIG_24WIRE);
}

/**************************************************************************/
/*!
    @brief Initialize the SPI interface and set the basic configuration
    @param configuration or together to your needs: MAX31865_CONFIG_BIAS, 
    MAX31865_CONFIG_MODEAUTO, MAX31865_CONFIG_3WIRE, MAX31865_CONFIG_FILT50HZ
    @return True if selfest detected no rerrors
*/
/**************************************************************************/
bool Adafruit_MAX31865::begin(uint8_t configuration, float R0, float Rref, float R2wire) {
  _configuration = configuration & (MAX31865_CONFIG_BIAS | MAX31865_CONFIG_MODEAUTO | MAX31865_CONFIG_3WIRE | MAX31865_CONFIG_FILT50HZ);
  if (_configuration & MAX31865_CONFIG_MODEAUTO) // MODEAUTO without BIAS does not make sense
    _configuration |= (MAX31865_CONFIG_BIAS | MAX31865_CONFIG_MODEAUTO);

  if (_ready_pin >= 0) {
    pinMode(_ready_pin, INPUT_PULLUP);
  }

  _R0 = R0;
  _Z3 = (4 * RTD_B) / _R0;
  _Rref = Rref;
  _ratioToRfactor = _Rref / ((1u << 15) - 1.0f);
  _R2wire = R2wire;

  _spi_dev.begin();
   
  set3Wires(_configuration & MAX31865_CONFIG_3WIRE);
  enable50Hz(_configuration & MAX31865_CONFIG_FILT50HZ);
  uint8_t err = readFault(true); // perform a selftest
  // ToDo: errorhandling!
  _config_reg.write(_configuration, 1);

  clearFault();

  //Serial.println(_configuration, HEX);
  //Serial.println(uint8_t(_config_reg.read()), HEX);
  return (err == 0);
}

/**************************************************************************/
/*!
    @brief modify the configuration register and its mirror in memory
    @param mask the bitmask of the bits to set/reset
    @param set If true the masks bit are set - else erased
    @return the new value of the configuration register
*/
/**************************************************************************/
uint8_t Adafruit_MAX31865::modifyConfig(uint8_t mask, bool set) {
  if (set) {
    _configuration |=  mask; // enable bias
  } else {
    _configuration &= ~ mask; // disable bias
  }
  _config_reg.write(_configuration, 1);
  return _configuration;
}

/**************************************************************************/
/*!
    @brief Read the stored fault status bit included in every RTD read
    @return true if the bit was set during the last readRTD()
*/
/**************************************************************************/
bool Adafruit_MAX31865::checkFault(void) { return _fault; }

/**************************************************************************/
/*!
    @brief Read the raw 8-bit FAULTSTAT register
    @return The raw unsigned 8-bit FAULT status register
*/
/**************************************************************************/
uint8_t Adafruit_MAX31865::readFault(void) {
  return _fault_reg.read();
}

/**************************************************************************/
/*!
    @brief Read the raw 8-bit FAULTSTAT register
    @param b If true a automatic fault-detection cycle is performed
    @return The raw unsigned 8-bit FAULT status register
*/
/**************************************************************************/
uint8_t Adafruit_MAX31865::readFault(boolean b) {
  if (b) {
    // B100X010X
    uint8_t t = MAX31865_CONFIG_FAULTDETCYCLE | (_configuration & (MAX31865_CONFIG_3WIRE | MAX31865_CONFIG_FILT50HZ));
    _config_reg.write( t, 1 );
    delay(5); // wait for 5ms
  }
  _config_reg.write( _configuration, 1 );
  return _fault_reg.read();
}

/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
void Adafruit_MAX31865::clearFault(void) {
  _config_reg.write( _configuration | MAX31865_CONFIG_FAULTSTAT, 1 );
  _fault = false;
}

/**************************************************************************/
/*!
    @brief Enable the bias voltage on the RTD sensor
    @param b If true bias is enabled, else disabled
*/
/**************************************************************************/
void Adafruit_MAX31865::enableBias(bool b) {
  modifyConfig(MAX31865_CONFIG_BIAS, b);
}

/**************************************************************************/
/*!
    @brief Enable the 50Hz-filter mode
    @param b If true bias is enabled, else disabled
*/
/**************************************************************************/
void Adafruit_MAX31865::enable50Hz(bool b) {
  bool autoc = _configuration  & MAX31865_CONFIG_MODEAUTO;
  if (autoc) autoConvert(false);
  modifyConfig(MAX31865_CONFIG_FILT50HZ, b);
  if (autoc) autoConvert(true);
}

/**************************************************************************/
/*!
    @brief Whether we want to have continuous conversions (50/60 Hz)
    @param b If true, auto conversion is enabled
*/
/**************************************************************************/
void Adafruit_MAX31865::autoConvert(bool b) {
  modifyConfig(MAX31865_CONFIG_MODEAUTO, b);
  enableBias(b); // MODEAUTO requires bias ON. 
                 // In 1SHOT mode we prefer the bias to be OFF.
}

/**************************************************************************/
/*!
    @brief How many wires we have in our RTD setup, can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @param wires The number of wires in enum format
*/
/**************************************************************************/
void Adafruit_MAX31865::setWires(max31865_numwires_t wires) {
  modifyConfig(MAX31865_CONFIG_3WIRE, wires == MAX31865_3WIRE);
}

/**************************************************************************/
/*!
    @brief Whether we want to have 3WIRE mode
    @param b If true, 3WIRE mode is enabled
*/
/**************************************************************************/
void Adafruit_MAX31865::set3Wires(bool b) {
  modifyConfig(MAX31865_CONFIG_3WIRE, b);
}

void Adafruit_MAX31865::setMaxRatio(uint16_t r) {
  _maxRratio_reg.write(r << 1, 2);
}

void Adafruit_MAX31865::setMinRatio(uint16_t r) {
  _minRratio_reg.write(r << 1, 2);
}

uint16_t Adafruit_MAX31865::getMaxRatio(void) {
  return _maxRratio_reg.read() >> 1;
}

uint16_t Adafruit_MAX31865::getMinRatio(void) {
  return _minRratio_reg.read() >> 1;
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t Adafruit_MAX31865::readRTD(void) {
  uint16_t rtd;
  if (_configuration & MAX31865_CONFIG_MODEAUTO) {
    // Bias current and Automode are ON.
    // This always results in a valid value.
    // If the new concersion was not ready you will get the old value.
    rtd = _rRatio_reg.read();

  } else {
    clearFault();
    enableBias(true);
    delay(10);
    uint8_t t = _config_reg.read();
    t |= MAX31865_CONFIG_1SHOT;
    _config_reg.write( t, 1 );
    delay(65);

    rtd = _rRatio_reg.read();

    enableBias(false);	 // to lessen sensor self-heating
  }

  _fault = rtd & 0x0001; // Store the fault flag

  rtd >>= 1; // remove fault

  return rtd;
}

uint16_t Adafruit_MAX31865::RtoRatio(float R) {
  //Serial.print("RtoRatio Ratio: ");
  //Serial.println(R / _ratioToRfactor);
  return uint16_t(R / _ratioToRfactor);
}

float Adafruit_MAX31865::RatioToR(uint16_t r) {
  //Serial.print("RatioToR Ratio: ");
  //Serial.println(r * _ratioToRfactor, 4);
  return r * _ratioToRfactor;
}

void Adafruit_MAX31865::setMaxTemp(float C) {
  setMaxRatio( RtoRatio( CtoR(C) + _R2wire));
}

float Adafruit_MAX31865::getMaxTemp(void) {
  return RtoC( RatioToR( getMaxRatio()) - _R2wire);
}

void Adafruit_MAX31865::setMinTemp(float C) {
  setMinRatio( RtoRatio( CtoR(C) + _R2wire));
}

float Adafruit_MAX31865::getMinTemp(void) {
  return RtoC( RatioToR( getMinRatio()) - _R2wire);
}

float Adafruit_MAX31865::temperature(void) {
  return RtoC( RatioToR( readRTD()) - _R2wire);
}

float Adafruit_MAX31865::RtoC(float R) {
  //Serial.print("R: "); Serial.println(R,6);
  #ifdef PT_RATIONAL
    if (R < RTD_R_BORDER) return RtoC_rational(R);
  #endif
  #ifdef PT_POLY5
    if (R < RTD_5_BORDER) return RtoC_poly5(R);
  #endif
  #ifdef PT_POLY4
    if (R < RTD_4_BORDER) return RtoC_poly4(R);
  #endif
  #ifdef PT_POLY3
    if (R < RTD_3_BORDER) return RtoC_poly3(R);
  #endif
  #ifdef PT_POLY2
    if (R < RTD_2_BORDER) return RtoC_poly2(R);
  #endif
  #ifdef PT_CVD 
    return RtoC_cvd(R);
  #endif
  #ifdef PT_LINEAR_B
    return RtoC_linearB(R);
  #endif
  #ifdef PT_LINEAR
    return RtoC_linear(R);
  #endif
  
}

// CVD forward calculation. Was used to define the temperature scale
// http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
// technique
float Adafruit_MAX31865::CtoR(float C) {
  float nt = (C < 0.0f) ? RTD_C * (C - 100.0f) * C : 0.0f;
  return _R0 * (1 + (RTD_A + (RTD_B + nt) * C) * C);
}

// inverse callendar van dusen formula.
// accurate from 0C up to 850 C.
#ifdef PT_CVD
  float Adafruit_MAX31865::RtoC_cvd(float R) {
    return ( sqrt(_Z2 + (_Z3 * R) ) + _Z1) / _Z4;
  }
#endif


#ifdef PT_RATIONAL
  // Rational polynomial fraction approximation taken from
  // Mosaic Industries.com page on "RTD calibration."
  // Accurate, probably beyond the ITS-90 spec
  float Adafruit_MAX31865::RtoC_rational(float R) {
    float z = R * (RTD_R_1 + R * (RTD_R_2 + R * (RTD_R_3 + R * RTD_R_4)));
    float n = 1.0 + R * (RTD_R_5 + R * (RTD_R_6 + R * RTD_R_7));
    return RTD_R_0 + (z / n);
  }
#endif

#ifdef PT_POLY5
// R2T polynomial from Analog Devices AN709 app note.
// implementation ganked from Adafruit MAX31865 library.
// Use for accurate temperatures -60C and below.
// Warning! Exceeds Class B tolerance spec above +164C

float Adafruit_MAX31865::RtoC_poly5(float R) {
    float rpoly = R;
    float temp = RTD_5_0;
    temp += RTD_5_1 * rpoly;
    rpoly *= R; // square
    temp += RTD_5_2 * rpoly;
    rpoly *= R; // ^3
    temp += RTD_5_3 * rpoly;
    rpoly *= R; // ^4
    temp += RTD_5_4 * rpoly;
    rpoly *= R; // ^5
    temp += RTD_5_5 * rpoly;
    return temp;
}
#endif

#ifdef PT_POLY4
float Adafruit_MAX31865::RtoC_poly4(float R) {
    float rpoly = R;
    float temp = RTD_4_0;
    temp += RTD_4_1 * rpoly;
    rpoly *= R; // square
    temp += RTD_4_2 * rpoly;
    rpoly *= R; // ^3
    temp += RTD_4_3 * rpoly;
    rpoly *= R; // ^4
    temp += RTD_4_4 * rpoly;
    return temp;
}
#endif

#ifdef PT_POLY3
float Adafruit_MAX31865::RtoC_poly3(float R) {
    float rpoly = R;
    float temp = RTD_3_0;
    temp += RTD_3_1 * rpoly;
    rpoly *= R; // square
    temp += RTD_3_2 * rpoly;
    rpoly *= R; // ^3
    temp += RTD_3_3 * rpoly;
    return temp;
}
#endif

#ifdef PT_POLY2
float Adafruit_MAX31865::RtoC_poly2(float R) {
    float rpoly = R;
    float temp = RTD_2_0;
    temp += RTD_2_1 * rpoly;
    rpoly *= R; // square
    temp += RTD_2_2 * rpoly;
    return temp;
}
#endif

#ifdef PT_LINEAR_B
  float Adafruit_MAX31865::RtoC_linearB(float R) {
    return (R / _R0 - 1.0f) / RTD_ALPHA + RTD_BETA;
  }
#endif

#ifdef PT_LINEAR
  float Adafruit_MAX31865::RtoC_linear(float R) {
    return (R / _R0 - 1.0f) / RTD_ALPHA;
  }
#endif
