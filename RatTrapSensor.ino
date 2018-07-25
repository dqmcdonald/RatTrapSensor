/*
    Lo-Ra enabled rat trap sensor.
    Designed for a Arduino Pro Mini (3.3V, 8Mhz) attached to
    Lo-Ra radio via SPI.
    A "fast" vibration switch connects pin 2 to ground.
    Most of the time the Mini (and lora radio) will be
    asleep. However then the vibration switch is triggered
    via the rat trap going off pin 2 goes low and the
    mini wakes from sleep. It then sends off the ID (which
    is stored in EEPROM) via the Lo-Ra radio and goes
    back to sleep

    Pin Assignments:
     2 -> Vibration Switch -> GND
     3 -> LoRa DOI0
     9 -> LoRa Reset
    10 -> LoRa Clock Select (NSS)
    11 -> LoRa MISO
    12 -> LoRa MOSI
    13 -> LoRa SCK

    Quentin McDonald
    July 2018
*/


#include "LowPower.h"
#include <SPI.h>
#include <RH_RF95.h>

#define DEBUG 1


#define RFM95_CS 10  // Clock select should be on 10
#define RFM95_RST 9  // Reset on 9
#define RFM95_INT 3  // Interrupt on 3

// Operating at 433 Mhz
#define RF95_FREQ 433.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Use pin 2 as wake up pin
const int WAKE_UP_PIN = 2;

void wakeUp()
{
  // Just a handler for the pin interrupt.
}



// Configure the LoRa radio
void setupLoRa() {

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

#ifdef DEBUG
  Serial.println("Arduino LoRa TX Test!");
#endif

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
#ifdef DEBUG
    Serial.println("LoRa radio init failed");
#endif
    while (1);
  }
#ifdef DEBUG
  Serial.println("LoRa radio init OK!");
#endif

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
#ifdef DEBUG
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
#endif

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  // Set to slow speed for longer range
  rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);



}




void setup() {

#ifdef DEBUG
  while (!Serial);
  Serial.begin(9600);
  delay(100);
#endif

  setupLoRa();

  // Configure wake up pin as input.
  // This will consumes few uA of current.
  pinMode(WAKE_UP_PIN, INPUT);
  digitalWrite(WAKE_UP_PIN, HIGH);

}

int16_t packetnum = 0;  // packet counter, we increment per xmission


void loop() {

#ifdef DEBUG
  Serial.println("About to sleep");
  Serial.flush();
#endif

  // Allow wake up pin to trigger interrupt on low.
  attachInterrupt(0, wakeUp, LOW);

  // Enter power down state with ADC and BOD module disabled.
  // Wake up when wake up pin is low.
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

  // Disable external pin interrupt on wake up pin.
  detachInterrupt(0);


  Serial.begin(9600);
#ifdef DEBUG
  Serial.println("Awake");
#endif

  delay(1000);


  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket + 13, 10);
#ifdef DEBUG
  Serial.print("Sending "); Serial.println(radiopacket);
#endif
  radiopacket[19] = 0;

#ifdef DEBUG
  Serial.println("Sending...");
#endif

  delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

#ifdef DEBUG
  Serial.println("Waiting for packet to complete..."); delay(10);
#endif

  rf95.waitPacketSent();



  delay(1000);

}
