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
  _config_reg(&_spi_dev, 0x00, ADDRBIT8_HIGH_TOWRITE, 1, MSBFIRST),
  _rRatio_reg(&_spi_dev, 0x01, ADDRBIT8_HIGH_TOWRITE, 2, MSBFIRST),
  _maxRratio_reg(&_spi_dev, 0x03, ADDRBIT8_HIGH_TOWRITE, 2, MSBFIRST),
  _minRratio_reg(&_spi_dev, 0x05, ADDRBIT8_HIGH_TOWRITE, 2, MSBFIRST),
  _fault_reg(&_spi_dev, 0x07, ADDRBIT8_HIGH_TOWRITE, 1, MSBFIRST)
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
  _config_reg(&_spi_dev, 0x00, ADDRBIT8_HIGH_TOWRITE, 1, MSBFIRST),
  _rRatio_reg(&_spi_dev, 0x01, ADDRBIT8_HIGH_TOWRITE, 2, MSBFIRST),
  _maxRratio_reg(&_spi_dev, 0x03, ADDRBIT8_HIGH_TOWRITE, 2, MSBFIRST),
  _minRratio_reg(&_spi_dev, 0x05, ADDRBIT8_HIGH_TOWRITE, 2, MSBFIRST),
  _fault_reg(&_spi_dev, 0x07, ADDRBIT8_HIGH_TOWRITE, 1, MSBFIRST)
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

/**************************************************************************/
/*!
    @brief Read the temperature in C from the RTD through calculation of the
    resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param RTDnominal The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param refResistor The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
float Adafruit_MAX31865::temperature(float RTDnominal, float refResistor) {
  float Z1, Z2, Z3, Z4, Rt, temp;

  Rt = readRTD();
  Rt /= 32768;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  // first, if needed, normalize to 100 ohm
  if (RTDnominal != 100.0)
    Rt *= 100.0 / RTDnominal;

  if (Rt >= 100.0) { // above 100 Ohm temperature will become positive
    Z1 = -RTD_A;
    Z2 = RTD_A * RTD_A - (4 * RTD_B);
    Z3 = (4 * RTD_B) / 100.0;
    Z4 = 2 * RTD_B;

    temp = Z2 + (Z3 * Rt);
    temp = (sqrt(temp) + Z1) / Z4;
  } else {
    float rpoly = Rt;

    temp = -242.02;
    temp += 2.2228 * rpoly;
    rpoly *= Rt; // square
    temp += 2.5859e-3 * rpoly;
    rpoly *= Rt; // ^3
    temp -= 4.8260e-6 * rpoly;
    rpoly *= Rt; // ^4
    temp -= 2.8183e-8 * rpoly;
    rpoly *= Rt; // ^5
    temp += 1.5243e-10 * rpoly;
  }
  return temp;
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
