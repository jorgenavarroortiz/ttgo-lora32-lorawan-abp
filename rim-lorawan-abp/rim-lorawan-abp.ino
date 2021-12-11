/*******************************************************************************
 * Copyright (c) 2019 Jorge Navarro Ortiz, University of Granada
 * Based on the work from Thomas Telkamp and Matthijs Kooijman
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// Periodic packets or one packet per button press? Comment or uncomment as desired
#define BUTTONENABLED

const String ABPVersion = "v1.0";
_dr_eu868_t selectedSF = DR_SF12;   // Spreading factor for the transmissions
unsigned int TX_INTERVAL = 10;      // Schedule TX every this many seconds (might become longer due to duty cycle limitations).
unsigned long lastTxTime=-1.0;
bool queuedPacket=false;

#define TTGONODE 17
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#include <Wire.h>
#include "SSD1306.h"
#include "images.h"

SSD1306 display(0x3c, 4, 15);
String rssi = "RSSI --";
String packSize = "--";
String packet;

#ifdef BUTTONENABLED
#include <EasyButton.h>
// Arduino pin where the button is connected to.
#define BUTTON_PIN 0
// Instance of the button.
EasyButton button(BUTTON_PIN);
#endif

// LoRaWAN end-device address (DevAddr) and session keys
static const u4_t DEVADDR = 0x26011CC6;
static const PROGMEM u1_t NWKSKEY[16] = { 0xC5, 0x66, 0xE4, 0x82, 0x5B, 0x85, 0x0D, 0x73, 0x44, 0x88, 0x2F, 0x3C, 0x14, 0xB8, 0xA3, 0x5C };
static const u1_t PROGMEM APPSKEY[16] = { 0x46, 0x38, 0x47, 0x0F, 0xDC, 0xBA, 0x91, 0x46, 0xF6, 0x63, 0x6B, 0xC9, 0x77, 0xF5, 0x91, 0x8A };

// These callbacks are only used in over-the-air activation, so they are left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

unsigned int counter = 0;
static uint8_t mydata[] = "TTGO #" STR(TTGONODE);
static osjob_t sendjob;

// Pin mapping for the Heltec ESP32 Lora board / TTGO Lora32 with 3D metal antenna
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}
};

void logo(){
  display.clear();
  display.drawXbm(35,12,LoRa_Logo_width,LoRa_Logo_height,LoRa_Logo_bits);
  display.display();
}

#ifdef BUTTONENABLED
// Callback function to be called when the button is pressed.
void onPressed() {
  Serial.println("Button has been pressed!");
  do_send(&sendjob);
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
            counter++;
            queuedPacket = false;
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission (JNa: only if periodic packets, i.e. button is not enabled)
#ifndef BUTTONENABLED
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
#endif
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
        if (lastTxTime > 0) {
            unsigned long aux = millis();
            Serial.println("Time between last two packets: " + String(aux-lastTxTime) + " msec");
            lastTxTime=aux;
        } else {
            lastTxTime = millis();
        }
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
        queuedPacket = true;
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    // For OLED
    pinMode(16,OUTPUT);
    pinMode(25,OUTPUT);
    digitalWrite(16, LOW);  // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high

    Serial.begin(115200);
    Serial.println("");

    uint64_t chipid=ESP.getEfuseMac(); // The chip ID is essentially its MAC address (length: 6 bytes).
    Serial.printf("Startup (ESP32 Chip ID: %04X%08X)\n",(uint16_t)(chipid>>32),(uint32_t)chipid);
    Serial.println("Node TTGO #" STR(TTGONODE) ", ABP " + ABPVersion);

#ifdef BUTTONENABLED
    Serial.println("Initializing button...");
    // Initialize the button.
    button.begin();
    // Add the callback function to be called when the button is pressed.
    button.onPressed(onPressed);
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

    // Set up the channels used in Europe (868 MHz band)
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_SF12,  DR_FSK),  BAND_MILLI);      // g2-band

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(selectedSF,14);

    // Start job (JNa: only if periodic packets, i.e. button is not enabled)
#ifndef BUTTONENABLED
    do_send(&sendjob);
#endif

    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    logo();
    delay(1500);
}

void loop() {
    os_runloop_once();

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Node TTGO #" STR(TTGONODE) ", ABP " + ABPVersion);
    uint64_t chipid=ESP.getEfuseMac(); // The chip ID is essentially its MAC address (length: 6 bytes).
    //display.drawString(0, 10, "CID: " + String((uint16_t)(chipid>>32)) + String((uint32_t)chipid));
    display.drawString(0, 30, "Packets sent: " + String (counter));
    if (queuedPacket) {
        display.drawString(0, 40, "Packet queued!");
    }
    display.display();

#ifdef BUTTONENABLED
    // Continuously read the status of the button. 
    button.read();
#endif
}
