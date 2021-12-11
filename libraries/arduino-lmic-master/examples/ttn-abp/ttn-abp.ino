/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//#define ONECHANNEL 1
//#define OLED 1
#define SF_TX 0
// 0 for SF12, 5 for SF7
//From lorabase.h: enum _dr_eu868_t { DR_SF12=0, DR_SF11, DR_SF10, DR_SF9, DR_SF8, DR_SF7, DR_SF7B, DR_FSK, DR_NONE }

//#define HELTEC 1
//#define HELTECNODE 1
#define HALLARD 1
#define HALLARDNODE 1

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#ifdef OLED
#include <Wire.h>
#include "SSD1306.h"
#include "images.h"

#ifdef HELTEC
SSD1306 display(0x3c, 4, 15);
#endif
#ifdef HALLARD
SSD1306 display(0x3c, 4, 5);
#endif
String rssi = "RSSI --";
String packSize = "--";
String packet ;
#endif

#ifdef HALLARD
#include <ESP8266WiFi.h> //not using WiFi but need for some of the sleep commands
#endif

// LoRaWAN DevAddr (end-device address), NwkSKey and AppSKey (network and application session keys).
#ifdef HELTEC
#if HELTECNODE==1
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011C35;
static const PROGMEM u1_t NWKSKEY[16] = { 0xC6, 0x6C, 0xA4, 0xCE, 0x8C, 0x9A, 0x4A, 0xD9, 0x6B, 0x93, 0x98, 0x17, 0xE0, 0x19, 0x7F, 0x18 };
static const u1_t PROGMEM APPSKEY[16] = { 0x32, 0x69, 0x0A, 0xE7, 0xFE, 0xB5, 0x20, 0x7A, 0xC1, 0x04, 0x62, 0x8C, 0x20, 0x10, 0xED, 0xE6 };
#endif
#endif
#ifdef HALLARD
#if HALLARDNODE==1
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x260117BF;
static const PROGMEM u1_t NWKSKEY[16] = { 0xE4, 0x39, 0x0B, 0x49, 0xA8, 0x82, 0xF5, 0xEE, 0xFD, 0x55, 0x5C, 0x86, 0xBB, 0x03, 0xA9, 0x5A };
static const u1_t PROGMEM APPSKEY[16] = { 0xF3, 0x7E, 0xBB, 0x47, 0x18, 0xE9, 0x55, 0x46, 0xC0, 0xC3, 0xEE, 0x21, 0x13, 0xD9, 0xB3, 0x6C };
#elif HALLARDNODE==2
static const u4_t DEVADDR = 0x26011C74;
static const PROGMEM u1_t NWKSKEY[16] = { 0x34, 0x7F, 0x18, 0x94, 0xC3, 0x73, 0xC6, 0x70, 0x82, 0xB6, 0x4B, 0x0F, 0x80, 0x6A, 0xC1, 0xEE };
static const u1_t PROGMEM APPSKEY[16] = { 0xE5, 0xFC, 0xF3, 0x6B, 0xF6, 0x8D, 0x8A, 0xB7, 0x63, 0x5C, 0x35, 0x3E, 0xC9, 0xB6, 0x3A, 0xDF };
#endif
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

unsigned int counter = 0;
#ifdef HELTEC
static uint8_t mydata[] = "HELTEC #" STR(HELTECNODE);
#endif
#ifdef HALLARD
static uint8_t mydata[] = "HALLARD RFM95 #" STR(HALLARDNODE);
#endif
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
/*const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};*/
#ifdef HELTEC
// For Heltec WIFI_LoRa_32 board
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
};
#endif
#ifdef HALLARD
// For WeMoS D1 mini with a Hallard's board (RFM95)
const lmic_pinmap lmic_pins = {
  .nss = 16,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {15, 15, LMIC_UNUSED_PIN},
};
#endif

#ifdef OLED
void logo(){
  display.clear();
  display.drawXbm(35,12,LoRa_Logo_width,LoRa_Logo_height,LoRa_Logo_bits);
  display.display();
}
#endif

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
        counter++;

#ifdef OLED
#ifdef HELTEC
//  digitalWrite(16, LOW);  // set GPIO16 low to reset OLED
//  delay(50);
//  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
#endif
//  display.clear();
//  display.drawString(0, 0, "Sending packet: ");
//  display.drawString(90, 0, String(counter));
//  display.display();

#endif
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
#ifdef OLED
#ifdef HELTEC
  pinMode(16,OUTPUT);
  pinMode(25,OUTPUT);

  digitalWrite(16, LOW);  // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
#endif
#endif

    Serial.begin(115200);
    Serial.println("");
#ifdef HELTEC
    uint64_t chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
    Serial.printf("Startup (ESP32 Chip ID: %04X%08X)\n",(uint16_t)(chipid>>32),(uint32_t)chipid);
    Serial.println("Node HELTEC #" STR(HELTECNODE));
#endif
#ifdef HALLARD
    Serial.println("Startup (ESP8266 Chip ID: " + String(ESP.getChipId()) + ")");
    Serial.println("Node HALLARD RFM95 #" STR(HALLARDNODE));
#endif

#ifdef HALLARD
    WiFi.forceSleepBegin(0); //This function turns on modem sleep mode (turns off RF but not CPU)
    delay(1);                //For some reason the modem won't go to sleep unless you do a delay(non-zero-number) -- no delay, no sleep and delay(0), no sleep - See more at: http://www.esp8266.com/viewtopic.php?p=38984#sthash.yr6I171c.dpuf
#endif

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
#ifdef ONECHANNEL
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868100000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868100000, DR_RANGE_MAP(DR_SF7, DR_SF7),  BAND_CENTI);      // g-band
#else
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
#endif
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(SF_TX,14);

    // Start job
    do_send(&sendjob);

#ifdef OLED
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    logo();
    delay(1500);
#endif
}

void loop() {
    os_runloop_once();

#ifdef OLED
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
#ifdef HELTEC
    display.drawString(0, 0, "Node HELTEC #" STR(HELTECNODE));
    uint64_t chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
    display.drawString(0, 10, "CID: " + String((uint16_t)(chipid>>32)) + String((uint32_t)chipid));
#endif
#ifdef HALLARD
    display.drawString(0, 0, "Node HALLARD RFM95 #" STR(HALLARDNODE));
    display.drawString(0, 10, "CID: " + String(ESP.getChipId()));
#endif
    display.drawString(0, 30, "Sending packet: ");
    display.drawString(90, 30, String(counter));
    display.display();
/*    digitalWrite(25, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(25, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);                       // wait for a second
*/
#endif
}
