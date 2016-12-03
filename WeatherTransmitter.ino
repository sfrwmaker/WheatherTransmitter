#include <WlessOregonV2.h>

/* Humidity/Temperature sensor si7021@atmega328p-pu chip running at 1 MGz powered by the two AA batteries
 * Emulates Oregon V2.1 protocol to send the data
 * The voltage regulator ams1117-adj, supplies constant 1.24 volts to battPIN pin when is powered up through the powerPIN
 * The higher the value read on battPIN the lower the battery.
 * arduine reads 380 at 3.3 volts on battery, and 569 at 2.22 volts on the battery
*/

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <SI7021.h>

const byte ledPin   = 7;                              // Pin for indicator LED
const byte transPin = 11;                             // Pin for the radio transmitter
const byte battPin  = 15;                             // A1, Pin to read battery level
const byte powerPIN = 8;                              // Pin the voltage regulator to be powered
const uint16_t low_battery = 550;                     // The limit for low battery

SI7021 sensor;
OregonSensor os(transPin, 0x20, 0xBA, true);

void enterSleep(void) {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);                // the lowest power consumption
  sleep_enable();
  sleep_mode();                                       // Now enter sleep mode.
  
  // The program will continue from here after the WDT timeout. 
  sleep_disable();                                    // First thing to do is disable sleep.
  power_all_enable();                                 // Re-enable the peripherals.
}

volatile int f_wdt=1;
 
 void setup() {
//  Serial.begin(9600);
  pinMode(ledPin,   OUTPUT);
  pinMode(battPin,  INPUT);
  pinMode(powerPIN, OUTPUT);
  digitalWrite(powerPIN, LOW);                        // Switch off the external voltage regulator on ams1117
  os.init();

  delay(15000);                                       // Sleep to flash new programm

  // Setup the WDT
  MCUSR &= ~(1<<WDRF);                                // Clear the reset flag
  
  // In order to change WDE or the prescaler, we need to
  // set WDCE (This will allow updates for 4 clock cycles).
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 1<<WDP0 | 1<<WDP3;                         // set new watchdog timeout prescaler value 8.0 seconds
  WDTCSR |= _BV(WDIE);                                // Enable the WD interrupt (note no reset).
  
  delay(100);                                         // Allow for serial print to complete.
  sensor.begin();
}
 
 void loop() {
  static int awake_counter = 1;
  static int check_battery = 0;
  static bool batteryOK = true;

  if (f_wdt == 1) {
    f_wdt = 0;
    --awake_counter;
    if (awake_counter <= 0) {                         // Send Weather data
      awake_counter = 5;
      // Check the battery level every 100 wake ups 100*5*8s = 4000 seconds
      if (batteryOK && (check_battery == 0)) {        // The battery cannot repare by itself!
        digitalWrite(powerPIN, HIGH);                 // power up the external voltage regulator that supply 1.24 volts to battPIN
        delay(10);
        uint16_t battLevel = analogRead(battPin);     // 0 <= battLevel <= 1023
        digitalWrite(powerPIN, LOW);                  // switch off the external voltage regulator
        //Serial.println(battLevel, DEC);
        batteryOK = (battLevel < low_battery); 
      }
      ++check_battery; if (check_battery > 100) check_battery = 0;

      int temperature = sensor.getCelsiusHundredths();
      if (temperature > 0)
        temperature += 5;
      else
        temperature -= 5;
      temperature /= 10;
      byte humidity   = sensor.getHumidityPercent();
 
      digitalWrite(ledPin, HIGH);                     // Switch on the check led before transmition
      os.sendTempHumidity(temperature, humidity, batteryOK);
      digitalWrite(ledPin, LOW);
    }
    enterSleep();
  }

}

ISR(WDT_vect) {
  if (f_wdt == 0) f_wdt = 1;
}

