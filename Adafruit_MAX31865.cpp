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

static SPISettings max31865_spisettings =
    SPISettings(500000, MSBFIRST, SPI_MODE1);

/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI
    @param spi_cs the SPI CS pin to use
    @param spi_mosi the SPI MOSI pin to use
    @param spi_miso the SPI MISO pin to use
    @param spi_clk the SPI clock pin to use
*/
/**************************************************************************/
//
Adafruit_MAX31865::Adafruit_MAX31865(int8_t spi_cs, int8_t spi_mosi,
                                     int8_t spi_miso, int8_t spi_clk) {
  _sclk = spi_clk;
  _cs = spi_cs;
  _miso = spi_miso;
  _mosi = spi_mosi;
}

/**************************************************************************/
/*!
    @brief Create the interface object using hardware SPI
    @param spi_cs the SPI CS pin to use along with the default SPI device
*/
/**************************************************************************/
Adafruit_MAX31865::Adafruit_MAX31865(int8_t spi_cs) {
  _cs = spi_cs;
  _sclk = _miso = _mosi = -1;
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
bool Adafruit_MAX31865::begin(uint8_t configuration) {
  _configuration = configuration & (MAX31865_CONFIG_BIAS | MAX31865_CONFIG_MODEAUTO | MAX31865_CONFIG_3WIRE | MAX31865_CONFIG_FILT50HZ);
  if (_configuration & MAX31865_CONFIG_MODEAUTO) // MODEAUTO without BIAS does not make sense
    _configuration |= (MAX31865_CONFIG_BIAS | MAX31865_CONFIG_MODEAUTO);

  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  if (_sclk != -1) {
    // define pin modes
    pinMode(_sclk, OUTPUT);
    digitalWrite(_sclk, LOW);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
  } else {
    // start and configure hardware SPI
    SPI.begin();
  }

  set3Wires(_configuration & MAX31865_CONFIG_3WIRE);
  enable50Hz(_configuration & MAX31865_CONFIG_FILT50HZ);
  uint8_t err = readFault(true); // perform a selftest
  writeRegister8(MAX31865_CONFIG_REG, _configuration);
  clearFault();

  //Serial.println(_configuration, HEX);
  //Serial.println(readRegister8(MAX31865_CONFIG_REG), HEX);
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
  writeRegister8(MAX31865_CONFIG_REG, _configuration);
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
  return readRegister8(MAX31865_FAULTSTAT_REG);
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
    writeRegister8(MAX31865_CONFIG_REG, t);
    delay(5); // wait for 5ms
  }
  writeRegister8(MAX31865_CONFIG_REG, _configuration); // restore configuration
  return readRegister8(MAX31865_FAULTSTAT_REG);
}

/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
void Adafruit_MAX31865::clearFault(void) {
  writeRegister8(MAX31865_CONFIG_REG, _configuration | MAX31865_CONFIG_FAULTSTAT);
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
    rtd = readRegister16(MAX31865_RTDMSB_REG);
  } else {
    clearFault();
    enableBias(true);
    delay(10);
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    t |= MAX31865_CONFIG_1SHOT;
    writeRegister8(MAX31865_CONFIG_REG, t);
    delay(65);

    rtd = readRegister16(MAX31865_RTDMSB_REG);

    enableBias(false);	 // to lessen sensor self-heating
  }

  _fault = rtd & 0x0001; // Store the fault flag

  rtd >>= 1; // remove fault

  return rtd;
}

/**********************************************/

uint8_t Adafruit_MAX31865::readRegister8(uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);

  return ret;
}

uint16_t Adafruit_MAX31865::readRegister16(uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];

  return ret;
}

void Adafruit_MAX31865::readRegisterN(uint8_t addr, uint8_t buffer[],
                                      uint8_t n) {
  addr &= 0x7F; // make sure top bit is not set

  if (_sclk == -1)
    SPI.beginTransaction(max31865_spisettings);
  else
    digitalWrite(_sclk, LOW);

  digitalWrite(_cs, LOW);

  spixfer(addr);

  // Serial.print("$"); Serial.print(addr, HEX); Serial.print(": ");
  while (n--) {
    buffer[0] = spixfer(0xFF);
    // Serial.print(" 0x"); Serial.print(buffer[0], HEX);
    buffer++;
  }
  // Serial.println();

  if (_sclk == -1)
    SPI.endTransaction();

  digitalWrite(_cs, HIGH);
}

void Adafruit_MAX31865::writeRegister8(uint8_t addr, uint8_t data) {
  if (_sclk == -1)
    SPI.beginTransaction(max31865_spisettings);
  else
    digitalWrite(_sclk, LOW);

  digitalWrite(_cs, LOW);

  spixfer(addr | 0x80); // make sure top bit is set
  spixfer(data);

  // Serial.print("$"); Serial.print(addr, HEX); Serial.print(" = 0x");
  // Serial.println(data, HEX);

  if (_sclk == -1)
    SPI.endTransaction();

  digitalWrite(_cs, HIGH);
}

uint8_t Adafruit_MAX31865::spixfer(uint8_t x) {
  if (_sclk == -1)
    return SPI.transfer(x);

  // software spi
  // Serial.println("Software SPI");
  uint8_t reply = 0;

  for (int i = 7; i >= 0; i--) {
    reply <<= 1;
    digitalWrite(_sclk, HIGH);
    digitalWrite(_mosi, x & (1 << i));
    digitalWrite(_sclk, LOW);
    if (digitalRead(_miso))
      reply |= 1;
  }

  return reply;
}
