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


#include <Adafruit_BusIO_Register.h>
#include <Adafruit_SPIDevice.h>

#include <Adafruit_MAX31865.h>
#include "filterring.h"

#define MAX31865_READY_PIN (2)

Filter Pt100Filter = Filter(100);

// Use software SPI: CS, DI, DO, CLK, [ready]
  //Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13, MAX31865_READY_PIN);
// use hardware SPI, just pass in the CS pin and optionally the ready pin
  Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, MAX31865_READY_PIN, &SPI);

// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0f
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0f
// The resistance of the wires to the sonsor - can't automatically be compensated for in 2Wire-mode.
// But we can do it manually wit this value.
#define R2WIRE    0.0f

void setup() {
  #if (defined(MAX31865_1_READY_PIN) && MAX31865_1_READY_PIN != -1)
    pinMode(MAX31865_READY_PIN ,INPUT_PULLUP);
  #endif

  Serial.begin(115200);
  Serial.println("Adafruit MAX31865 PT100 Sensor Test in MODEAUTO!");

  //thermo.begin(/*MAX31865_CONFIG_MODEAUTO |*/ MAX31865_CONFIG_24WIRE | MAX31865_CONFIG_FILT50HZ);
  thermo.begin(MAX31865_CONFIG_MODEAUTO | MAX31865_CONFIG_24WIRE | MAX31865_CONFIG_FILT50HZ, RNOMINAL, RREF, R2WIRE);
}

void loop() {
  static bool printTemp = true;
  static bool printtime = false;
  static bool printResistance = false;
  static bool Automode = true;
  static float Pt = RNOMINAL;
  static float Rr = RREF;
  static int Scale = 1;
  static int pause = 0;
  bool skip = false;

    if (Automode) {
      #if (defined( MAX31865_READY_PIN ) && (MAX31865_1_READY_PIN != -1))
        // Is converstion ready?
        skip = digitalRead(MAX31865_READY_PIN);
      #else
      // Warant conversion is ready.
      if (Hz50) {
        delay(21);
      } else {
        delay(17);
      }
      #endif
    }
    if (!skip) {
      Pt100Filter.fill(thermo.temperature(Pt, Rr)) ;
      if (thermo.checkFault())
        printFaults();
    } // skip

  char c = 0;
  if (Serial.available()) {
    c = Serial.read();
    delay(1); // gife the Serial time to read the rest of the line
  }
  switch (c) {
    case '1': thermo.autoConvert(false); Automode = false; break; // set ONE_SHOT_MODE
    case '2': thermo.set3Wires(false); break;                     // set 2_WIRES
    case '3': thermo.set3Wires(true); break;                      // set 3_WIRES
    case '4': thermo.set3Wires(false); break;                     // set 4_WIRES
    case '5': thermo.enable50Hz(true); break;                     // set 50HZ
    case '6': thermo.enable50Hz(false); break;                    // set 60HZ
    case 'a': thermo.autoConvert(true); Automode = true; break;   // set AUTO_MODE
    case 'c': thermo.clearFault(); break;                         // clear Faults
    case 'h': Pt = 100.0; Rr = 430.0; break;                      // set Adafruit PT100-module
    case 'k': Pt = 1000.0; Rr = 4300.0; break;                    // set Adafruit PT1000-module
    case 'r': printResistance = !printResistance; break;          // toggle print resistance
    case 't': printtime = !printtime; break;                      // toggle print conversion-time
    case 'T': printTemp = !printTemp; break;                      // toggle print temperature
    case 'S': thermo.readFault(true); break;                      // initiate a complete selftest
    case 's': { 
                Scale = parseuint32();
                Scale = constrain(Scale, 0, 32000);
              }
              break;
    case 'd': {                                                   // set time between printing values d[0-32000] (32 seconds)
                pause = parseuint32();
                pause = constrain(pause, 1, 32000); 
              }
              break;
    case 'f': { 
                int L = 16;                                       // set circumflex of the filter-ring f[1-254]
                L = parseuint32();
                L = constrain(L, 1, 254);
                Pt100Filter.length(L);
              }
              break;
    case '+': { 
                int L = 16;                                       // set upper alarm temperature
                L = parseuint32();
                L = constrain(L, -200, 850);
                thermo.setMaxTemp(L);
              }
              break;
    case '-': { 
                int L = 16;                                       // set lower alarm temperature
                L = parseuint32();
                L = constrain(L, -200, 850);
                thermo.setMinTemp(L);
              }
              break;
    default: ;
  } // switch
  
  // print temperature
  {
    uint32_t now = millis();
    static uint32_t then = now;
    if (now >= then) {
      then = now + pause;

      if (printTemp) {
        Serial.print("Temperature: ");
        Serial.print(Pt100Filter.request() * Scale,4);
        Serial.print(" °C ");
      }
      
      if (printResistance) {
        uint16_t rtd = thermo.readRTD();
        float ratio = rtd / 32768.0;
        Serial.print(", Resistance: "); 
        Serial.print(Rr*ratio,4);
        Serial.print(" Ω "); 
      }

      if (printtime) {
        static uint32_t lastTime = micros();
        uint32_t now = micros();
        Serial.print(", Time: ");
        Serial.print((now - lastTime)/1000.0f,4);
        Serial.print(" ms");
        lastTime = now;
      }

      Serial.println();

    }
  }
}

void printFaults( void ) {
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
  }
}

uint32_t parseuint32( void ) {
  delay(1);                                         // set scale s[0-32000]
  char in[11] = { 0 }; uint8_t i = 0;
  do { in[i] = Serial.read(); } while((i < 10) && (in[i++] != '\n'));
  in[i] = 0;
  return atoi(in);
}
