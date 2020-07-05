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
     4 -> LED (Optional)
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

#include <EEPROM.h>

#define RFM95_CS 10  // Clock select should be on 10
#define RFM95_RST 9  // Reset on 9
#define RFM95_INT 3  // Interrupt on 3

// Operating at 433 Mhz
#define RF95_FREQ 433.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

char radiopacket[25] = "Trap Sprung:      ";

void flashLED( int numflash, int on_time, int off_time );

// Use pin 2 as wake up pin
const int WAKE_UP_PIN = 2;

const int ID_LEN = 6;
// The station ID
char id[ID_LEN + 1];

const int MAX_RETRIES = 3; // Try to send three times:

const int OPT_LED_PIN = 4;

void wakeUp()
{
  // Just a handler for the pin interrupt.
}



// If there is a serial connection then allow configuration of the
// ID. The ID is a six character code which is stored in EEPROM
void configureID() {

  Serial.setTimeout(10000); // Will wait 10 seconds for input
  Serial.println("To configure ID enter 'y'");

  char answer;
  int i;

  int bytes_read = Serial.readBytes(&answer, 1);
  if ( bytes_read == 1 && answer == 'y') {
    Serial.println("Enter six character ID");
    bytes_read = Serial.readBytes(id, 6);
    if ( bytes_read == 6 ) {
      Serial.print("Id = ");
      Serial.println(id);
      for (  i = 0; i < ID_LEN; i++ ) {
        EEPROM.write(i, id[i]);
      }
    }
  }
  for (  i = 0; i < ID_LEN; i++ ) {
    id[i] = EEPROM.read(i);
  }

  id[ID_LEN] = '\0';
  Serial.print("Using station ID:" );
  Serial.println(id);

}



// Configure the LoRa radio
void setupLoRa() {

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);



  Serial.println("Initializing LoRa radio");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  Serial.println("LoRa radio init OK");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);


  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  // Set to slow speed for longer range
  rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);
}




void setup() {

  pinMode(OPT_LED_PIN, OUTPUT);


  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Rat Trap Sensor");

  configureID();




  setupLoRa();

  flashLED( 5, 250, 50);

  // Configure wake up pin as input.
  // This will consumes few uA of current.
  pinMode(WAKE_UP_PIN, INPUT);
  digitalWrite(WAKE_UP_PIN, HIGH);

}

void loop() {


  Serial.println("About to sleep");
  Serial.flush();

  rf95.sleep();

  // Allow wake up pin to trigger interrupt on low.
  attachInterrupt(0, wakeUp, LOW);

  // Enter power down state with ADC and BOD module disabled.
  // Wake up when wake up pin is low.
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

  // Disable external pin interrupt on wake up pin.
  detachInterrupt(0);


  Serial.begin(9600);

  Serial.println("Awake");

  rf95.setModeIdle();

  delay(1000);

  // Flash the LED 15 times with a second delay. This will allow time to open up the box and switch
  // off the device before getting spurious reports when resetting or rebaiting the trap
  flashLED(15, 500, 1000 );


  setupLoRa();



  memcpy( radiopacket + 12, id, 6 );

  radiopacket[19] = '\0';
  Serial.print("Sending "); Serial.println(radiopacket);



  for ( int attempt = 0; attempt < MAX_RETRIES; attempt++ ) {

    Serial.print("Sending in attempt ");
    Serial.print(attempt + 1, DEC);
    Serial.print(" of ");
    Serial.println(MAX_RETRIES, DEC);


    delay(10);

    long int send_time = millis();
    rf95.send((uint8_t *)radiopacket, 20);


    Serial.println("Waiting for packet to complete..."); delay(10);


    rf95.waitPacketSent();

    Serial.print("Time to send (ms) = ");
    Serial.println(millis() - send_time);


    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    Serial.println("Waiting for reply..."); delay(100);
    if (rf95.waitAvailableTimeout(4000))
    {
      // Should be a reply message for us now
      if (rf95.recv(buf, &len))
      {
        Serial.print("Got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);
        break;
      }
      else
      {
        Serial.println("Receive failed");
      }
    }
    else
    {
      Serial.println("No reply, is there a listener around?");
    }



    delay(1000);

  }
  flashLED( 5, 400, 100);
}


void flashLED( int numflash, int on_time, int off_time ) {
  // Flash the builtin LED numflash times with on_time and off_time between each one
  int i;
  for ( i = 0; i < numflash; i++) {
    digitalWrite(OPT_LED_PIN, HIGH);
    delay(on_time);
    digitalWrite(OPT_LED_PIN, LOW);
    delay(off_time);
  }
}
