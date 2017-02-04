/************************************************************************
 *  433MHz - Bodenfeuchte-Sensor [FHEM]
 *  Thanks to TantaJu@https://forum.fhem.de/index.php/topic,57460.0.html
 *  
 *  juergs, 16.10.2016, initial version.
 *  
 *  *********************************************************************
 *  ATMEL ATTINY 25/45/85 / ARDUINO
 *
 *                      +-\/-+
 *     Ain0 (D 5) PB5  1|    |8  Vcc
 *     Ain3 (D 3) PB3  2|    |7  PB2 (D 2) Ain1 *INT2*
 *     Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1 *INT1*
 *                GND  4|    |5  PB0 (D 0) pwm0 *INT0*
                        +----+
 *  
 *  Install: ATtiny-Models in Arduino IDE:
 *  http://highlowtech.org/?p=1695
 *  
 *  ATTINY:  
 *    https://cpldcpu.wordpress.com/2014/04/25/the-nanite-85/ 
 *    https://thewanderingengineer.com/2014/08/11/pin-change-interrupts-on-attiny85/
 *  
 *  
 * The LaCrosse-protocol seems to be:
 *
 *     Bits 0-3: header
 *     Bits 4-11: device ID, changes when replacing the batteries. Unlike in the post linked above, bit 11 does not appear to be a checksum.
 *     Bits 12-15: either 1111 for automatic transmission (once every 60 seconds) or 1011 for manual transmission (using the button in the battery compartment). Manual transmission does not update the weather station.
 *     Bits 16-27: encode the temperature. The system of encoding decimal digits seems to be ditched in favor of a more elegant one: apply a NOT (change 1 to 0 and 0 to 1), convert to base 10, divide by 10 (into a float), subtract 50, and the result is the temperature in C.
 *     Bits 28-35: encode the relative humidity. Apply a NOT, convert to base 10, and the result is the relative humidity in %.
 *     Bits 36-43: appear to encode a checksum (though I plan to double-check if this is not the dew point, also reported by the weather station).
      
 *     Example:
 *     HHHH 1000 0010 1111 1101 0010 1111 1101 0011 1010 0100
 *     encoding T=22.0C and RH=44%
 *  
 */

/******************************************************************************************************************************************************/
/* ATtiny 84/85 SerialMonitor Test */

//    Senden via "SoftwareSerial" - TX an Pin  4 (= Pin3 am Attiny85-20PU)
//    Senden via "SoftwareSerial" - TX an Pin  7 (= Pin6 am Attiny84-10PU)
// Empfangen via "SoftwareSerial" - RX an Pin 99 (Dummy um Hardwarepin zu sparen)

#include <SoftwareSerial.h>

#include "Narcoleptic.h"
#include "LaCrosse.h"
#include "OneWire.h"

#define SN ("Bodenfeuchte-Sensor-433-Version")
#define SV ("1.0 vom 16.10.2016")

//--- conditionals
//--- zum aktivieren Kommentierung entfernen 
//#define USE_WITH_NANO            
//#define USE_SEPARATE_BATTERIE_ID 
#define USE_WITH_DALLAS_SENSOR          
//#define USE_WITH_LED

#define DALLAS_SENSOR_PIN         0     //   DIP.5 = PB0 = D0 - Achtung: MOSI, Jumper zum Programmieren entfernen.
#define BODENFEUCHTE_POWER_PIN    1     //   
#define BODENFEUCHTE_SENSOR_PIN   2     //
#define TX_433_PIN                3     //   PIN6 = PB1 = D1 - Achtung: MISO.   PIN_SEND in LaCrosse.cpp

#define SENSORID_BODENFEUCHTE     100
#define SENSORID_BATTERIE         101

#define OW_ROMCODE_SIZE           8

#ifdef USE_WITH_DALLAS_SENSOR
   //--- die 18B20-Instanz setzen 
   OneWire  ds(DALLAS_SENSOR_PIN);     // on arduino port (a pullup 4.7K resistor is necessary, between Vcc and DQ-Pin   1=GND 2=DQ 3=Vcc )
#endif

//--- use with leds? 
#define LED_ONTIME       100  // Number of cycles for LEDs to stay on, when only temporary

//-- how many AA Cells do you want? Set 15 per cell, so 1*AA = 15, 2*AA=30
#define MAXBATTERY        15  // Maximum voltage of Battery in 100 mV, for percentage calculation
/*
 * Nothing to normally configure beyond this line, hardware specific configs follow
 */
#define POWERAMV          1   // Output line to provide power for the AMV
#define INPUTFREQ         2   // IRQ input for frequency count. Must be 2 or 3

#define CHILD_ID          0   // Child-Number for Humidity
#define CHILD_TEMP        1   // Child-Number for Temperature, if attached

#define BATTPOWER         1   // ADC Input for Battery-Power, comment out if no Step-Up is used

#define MAIN_PERIOD     998   // How long to count pulses, for a fixed frequency this should be 998 ms (for a total runtime of 1 sec with 1 MHz oscillator)
#define SETTLE_TIME     1000   // Waiting time in ms between powering up AMV and stable frequency

#ifdef WITH_LED
  #define LED_WS            5   // White LED, normal operation
  #define LED_RO            6   // Red LED, Error
  #define LED_MAIN          4   // Mains for LEDs, must be set LOW for LEDs to work
#endif 

//--- globals
SoftwareSerial          softSerial(99, 4); // RX, TX   // bei Tiny 85
OneWire                 dallas(DALLAS_SENSOR_PIN);  // on arduino port pin 2 (a 4.7K resistor is necessary, between Vcc and DQ-Pin   1=GND 2=DQ 3=Vcc )

unsigned int            led_startup;
float                   controller_VCC = 0.0; 
long                    vcc_reading    = 0;

/*
 * Internals to follow, nothing to adjust
 */
volatile unsigned int   pulsecount = 0;   // Counter for pulses
unsigned int            average;                 // IIR floating average filter
unsigned long           statestart;             // Storage for statemachine timer
int                     my_state;                         // actual state of statemachine
byte                    led_temporary = 1;                // Are LEDs jumpered to stay on=0, otherwise 1 
float                   bodenfeuchte     = 0.0;
float                   batteriespannung = 0.0;

//---------------------------------------------------------------------
//--- prototypes 
long  getVcc();
float DoBodenFeuchteMeasurement();
float ReadSingleOneWireSensor(OneWire ds); 
//---------------------------------------------------------------------
void setup() 
{
  //--- setup code here
  
  #if USE_WITH_NANO
      Serial.begin(9600);  
      delay(2000);    
      Serial.println("Start Bodenfeuchte-Sensor.");  
      pinMode(13, OUTPUT);
      digitalWrite(13,LOW); 
   #else
      //--- ATtiny free pin D4
      pinMode(4, OUTPUT);
      digitalWrite(4,LOW);
   #endif 

  //--- make power-line for AMV output and low
  pinMode(POWERAMV, OUTPUT);
  digitalWrite(POWERAMV,LOW);

  //--- preset SensorId & TX instance 
  LaCrosse.bSensorId = SENSORID_BODENFEUCHTE;
  LaCrosse.setTxPinMode(OUTPUT);

  softSerial.begin(38400);
  delay(5000);  
  //softSerial.print  (F("***START ")); softSerial.println();
}
//---------------------------------------------------------------------
void loop() 
{
  // put your main code here, to run repeatedly:
  #if USE_WITH_NANO
    Serial.print("Temp.: ");
    Serial.print(( (float) temp_mittel/10.0) + temp_offset, 1);
    Serial.print(" Druck: ");
    Serial.print(((float)druck_mittel/100.0)+druck_offset, 2);
    Serial.print(" Druck[corr]: ");
    Serial.println((float) luftdruck, 2);  
  #endif
  
  delay(1000); // ms

  #if USE_WITH_NANO
    digitalWrite(13,HIGH);
  #else
    //digitalWrite(4,HIGH);
  #endif 

  //--- Betriebsspannung auslesen  
  vcc_reading   = getVcc(); 
  controller_VCC = 1.1 * 1023 / vcc_reading; 

  softSerial.print("Vcc: ");
  softSerial.print( (float) controller_VCC, 1);
 // softSerial.print("    Vcc_read: ");
 // softSerial.print(vcc_reading);
  softSerial.print("    ");
  
  //--- Bodenfeuchte auslesen 
  bodenfeuchte = DoBodenFeuchteMeasurement(); 
  //bodenfeuchte = 1.0; 
  
  softSerial.print("Feuchte: ");
  softSerial.print( (float) bodenfeuchte, 1);
  softSerial.print("    ");

  float theta = ReadSingleOneWireSensor(dallas);
  softSerial.print("Temp: ");
  softSerial.println( (float) theta, 1);

  //--- transfer measured values to LaCrosse-instance
  LaCrosse.bSensorId = SENSORID_BODENFEUCHTE;
  LaCrosse.t = theta;    //--- alias temperature;  
  LaCrosse.sendTemperature();
  
  LaCrosse.sleep(1);        /* 1 second, no power-reduction! */

  #ifdef USE_SEPARATE_BATTERIE_ID 
    LaCrosse.bSensorId = SENSORID_BATTERIE;
  #endif 
  
  LaCrosse.h = controller_VCC;        //--- alias luftdruck;
  LaCrosse.sendHumidity();

  #if USE_WITH_NANO
    digitalWrite(13,LOW);
  #else
    //digitalWrite(4,LOW);
  #endif

  LaCrosse.sleep(1);        /* 1 second, no power-reduction! */

  #if USE_WITH_NANO
    digitalWrite(13,HIGH);
  
    long vcc = readVcc(); 
    Serial.print("VCC = ");
    Serial.println(vcc,DEC);  
    Serial.println("==============================================="); 
    Serial.println();
  #endif 

  #if USE_WITH_NANO
    digitalWrite(13,LOW);
    delay(60000); // ms
  #else
    ///Narcoleptic.delay_minutes(3);

    delay(10000); // ms
  #endif 
}
//---------------------------------------------------------------------
long getVcc() 
{
  //--- read 1.1V reference against AVcc
  //--- set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

/*
 *        internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
                         = 1.1 * 5126 / 5258 => 1.09 ==> 1.09*1023*1000 = 1097049
*/ 

  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  result   = 1097049L / result; // korrigierter Wert
  
  return result; // Vcc in millivolts
}
//---------------------------------------------------------------------
void myinthandler() // interrupt handler
{
    pulsecount++;
}
//---------------------------------------------------------------------
float DoBodenFeuchteMeasurement()
{
    //--- [1] startUp Multivibrator
    pinMode(INPUTFREQ, INPUT);     
    pinMode(POWERAMV, OUTPUT);      
    digitalWrite(POWERAMV, HIGH);   //--- power up sensor circuit  
  
    //--- [2] wait for settle
    delay( SETTLE_TIME);

    //--- [3] Prepare measurement
    pulsecount = 0;
    #if INPUTFREQ == 2
          attachInterrupt(0, myinthandler, FALLING); // IRQ D2 low to high
    #elif INPUTFREQ == 3
          attachInterrupt(1, myinthandler, FALLING); // IRQ D2 low to high
    #else
      #error "Frequency input must be D2 or D3"
    #endif

    //--- [4] wait for measurement to finalize
    delay (MAIN_PERIOD);

    //--- [5] store actual counter value
    //---     register counts per period (frequency) and calculate IIR
    unsigned long _pulses = pulsecount;
    pulsecount = 0;

    // Simple IIR, floating average
    average += _pulses;
    if(average != _pulses)    // During startup both are equal
      average >>= 1;

    //---[6] stop measuring and AMV
    digitalWrite(POWERAMV, LOW);
    #if INPUTFREQ == 2
          detachInterrupt(0);
    #elif INPUTFREQ == 3
          detachInterrupt(1);
    #else
      #error "Frequency input must be D2 or D3"
    #endif
    
    pinMode(INPUTFREQ, INPUT);

    //--- [7] result set reading  for TX 
   return (average * 1.0); 
      
}
//-------------------------------------------------------------------------
float ReadSingleOneWireSensor(OneWire ds)
{
  //--- 18B20 stuff
  byte i;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius = 12.3;

  if (!ds.search(addr))
  {
    ds.reset_search();
    delay(250);
    return celsius;
  }

  if (OneWire::crc8(addr, 7) != addr[7])
  {
    return celsius;
  }

  //--- the first ROM byte indicates which chip
  switch (addr[0])
  {
  case 0x10:
    //Serial.println("  Chip = DS18S20");  // or old DS1820
    type_s = 1;
    break;
  case 0x28:
    // Serial.println("  Chip = DS18B20");
    type_s = 0;
    break;
  case 0x22:
    // Serial.println("  Chip = DS1822");
    type_s = 0;
    break;
  default:
    // Serial.println("Device is not a DS18x20 family device.");
    return celsius;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1 );  // start conversion, use alternatively ds.write(0x44,1) with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
           // we might do a ds.depower() here, but the reset will take care of it.

  ds.reset();    //--- DS18B20 responds with presence pulse
  //--- match ROM 0x55, sensor sends ROM-code command ommitted here.
  ds.select(addr);
  ds.write(0xBE);         //--- read scratchpad
  for (i = 0; i < 9; i++)
  {
    //--- we need 9 bytes, 9th byte is CRC, first 8 are data
    data[i] = ds.read();
  }

  //--- Convert the data to actual temperature
  //--- because the result is a 16 bit signed integer, it should
  //--- be stored to an "int16_t" type, which is always 16 bits
  //--- even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s)
  {
    raw = raw << 3;     //--- 9 bit resolution default
    if (data[7] == 0x10)
    {
      //--- "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    };
  }
  else
  {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
                        //// default is 12 bit resolution, 750 ms conversion time
  };

  celsius = (float)raw / 16.0;    //fahrenheit = celsius * 1.8 + 32.0;

  //---- Check if any reads failed and exit early (to try again).  
  if (isnan(celsius))
  {
    //--- signalize error condition 
    celsius = -99.9;
  };
  return celsius;
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
// <eof>
//---------------------------------------------------------------------

