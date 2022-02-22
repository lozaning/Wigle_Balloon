/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
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
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "WiFi.h"
#include <Wire.h>
#include <axp20x.h>
#include <TinyGPS++.h>

#define SCK_GPIO        5
#define MISO_GPIO       19
#define MOSI_GPIO       27
#define NSS_GPIO        18
#if defined(T_BEAM_V10)
#define RESET_GPIO      14
#else
#define RESET_GPIO      23
#endif
#define DIO0_GPIO       26
#define DIO1_GPIO       33 // Note: not really used on this board
#define DIO2_GPIO       32 // Note: not really used on this board

//gps pins
#define UBLOX_GPS_OBJECT()  TinyGPSPlus gps
#define GPS_BAUD_RATE 9600
#define GPS_RX_PIN 34
#define GPS_TX_PIN 12

#define I2C_SDA 21
#define I2C_SCL 22
#define AXP192_SLAVE_ADDRESS 0x34

UBLOX_GPS_OBJECT();

AXP20X_Class axp;

bool axp192_found = false;

char buff[5][256];
uint64_t gpsSec = 0;

int           letters                 = 0;
int           arrayspot = 0;
int           otherletters                 = 0;
uint32_t LatitudeBinary;
uint32_t LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
uint8_t sats;
char t[32]; // used to sprintf for Serial output
int send_gps = 1;


void scanI2Cdevice(void)
{
    byte err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("I2C device found at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.print(addr, HEX);
            Serial.println(" !");
            nDevices++;

            if (addr == AXP192_SLAVE_ADDRESS) {
                axp192_found = true;
                Serial.println("axp192 PMU found");
            }
        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
                Serial.print("0");
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}


// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= { sEcReTs };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= {sEcReTs };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { sEcReTs };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

uint8_t mydata[33];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = NSS_GPIO,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RESET_GPIO,
    .dio = {DIO0_GPIO, DIO1_GPIO, DIO2_GPIO},
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

String Wifi_Name() {
  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("No networks found");
  } else {
      return WiFi.SSID(1);

      
    }
  }








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
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
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
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
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
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        int n = WiFi.scanNetworks();
        if (n == 0) {
          Serial.println("No networks found");
      } else {
        /*if(send_gps == 0){
            Serial.println(WiFi.BSSIDstr(1));
            char wifiMACBuf[WiFi.BSSIDstr(1).length()];
            WiFi.BSSIDstr(1).toCharArray(wifiMACBuf, WiFi.BSSIDstr(1).length());
            //
            otherletters = 0;
            arrayspot = 0;
            while (otherletters < (WiFi.BSSIDstr(1).length()+3)) {
            mydata[arrayspot] = wifiMACBuf[otherletters];
              otherletters++;
              arrayspot++;
            }
            LMIC_setTxData2(3, mydata, sizeof(wifiMACBuf)+3, 1);
            Serial.println(F("MAC Packet Queed"));
            Serial.println("");
            Serial.println("");
            Serial.println("");
            send_gps = 1;
            Serial.print("gps_send is now: ");
            Serial.println(send_gps);
          }
          */  

          if(send_gps == 1){
            Serial.println("send_gps is equal to 1 so sending SSID");
            char wifiBuf[WiFi.SSID(1).length()];
            WiFi.SSID(1).toCharArray(wifiBuf, WiFi.SSID(1).length());
            //
            otherletters = 0;
            arrayspot = 0;
            while (otherletters < (WiFi.SSID(1).length()+2)) {
              mydata[arrayspot] = wifiBuf[otherletters];
              otherletters++;
              arrayspot++;
            }
            Serial.print("Wifi Name: ");
            Serial.println(Wifi_Name());
            Serial.println("About to Send SSID packet");
            LMIC_setTxData2(2, mydata, sizeof(wifiBuf)+2, 1);
            Serial.println("SSID Packet Sent");
            send_gps = 2;
            Serial.print("gps_send is now: ");
            Serial.println(send_gps);
            Serial.println("");
            Serial.println("");
            Serial.println("");
            }
            
          if(send_gps == 2){
            Serial.println("Gps was equal to 2 so we're sending GPS Data");
            Serial.println("");
            //start second gps packet builder and brodcast
            LatitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
            LongitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;
            altitudeGps = gps.altitude.meters();
            hdopGps = gps.hdop.value() / 10;
            sats = gps.satellites.value();

            Serial.println("GPS Data");
            sprintf(t, "Lat: %f", gps.location.lat());
            Serial.println(t);
            sprintf(t, "Lng: %f", gps.location.lng());
            Serial.println(t);
            sprintf(t, "Alt: %d", altitudeGps);
            Serial.println(t);
            sprintf(t, "Hdop: %d", hdopGps);
            Serial.println(t);
            sprintf(t, "Sats: %d", sats);
            Serial.println(t);
            Serial.println("");
            Serial.println("");
        
            //set buffer to contain data
            mydata[0] = ( LatitudeBinary >> 16 ) & 0xFF;
            mydata[1] = ( LatitudeBinary >> 8 ) & 0xFF;
            mydata[2] = LatitudeBinary & 0xFF;
            mydata[3] = ( LongitudeBinary >> 16 ) & 0xFF;
            mydata[4] = ( LongitudeBinary >> 8 ) & 0xFF;
            mydata[5] = LongitudeBinary & 0xFF;
            mydata[6] = ( altitudeGps >> 8 ) & 0xFF;
            mydata[7] = altitudeGps & 0xFF;
            mydata[8] = hdopGps & 0xFF;
            mydata[9] = sats & 0xFF;

            //sending the gps data
        
            Serial.println("Sending the GPS Packet");
            LMIC_setTxData2(1, mydata, 10, 1);
            Serial.println("GPS Packet Sent");
            send_gps = 1;
            Serial.print("gps_send is now: ");
            Serial.println(send_gps);
            Serial.println("");
            Serial.println("");
            Serial.println("");
            
            }
      }
    // Next TX is scheduled after TX_COMPLETE event.
    }
    }


void setup() {
    delay(1000);
    Serial.begin(9600);
    Serial.println(F("Starting"));
    Wire.begin(I2C_SDA, I2C_SCL);
  
    scanI2Cdevice();
  
    if (axp192_found) {
      if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
          Serial.println("AXP192 Begin PASS");
          // power on ESP32 & GPS
          axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
          axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
          axp.setDCDC1Voltage(3300);  //esp32 core VDD    3v3
          axp.setLDO3Voltage(3300);   //GPS VDD      3v3

          Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
          Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
          Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
          Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
          Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
          Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");
               
          // Set mode of blue onboard LED (OFF, ON, Blinking 1Hz, Blinking 4 Hz)
          // axp.setChgLEDMode(AXP20X_LED_OFF);
          //axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
          axp.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
          //axp.setChgLEDMode(AXP20X_LED_BLINK_4HZ);
      } else {
          Serial.println("AXP192 Begin FAIL");
      }
      
  } else {
      Serial.println("AXP192 not found");
  }
      
  Serial1.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
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

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
    while (Serial1.available())
      gps.encode(Serial1.read());
    /*static uint64_t gpsMap = 0;

  while (Serial1.available())
  gps.encode(Serial1.read());

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    snprintf(buff[0], sizeof(buff[0]), "T-Beam GPS");
    snprintf(buff[1], sizeof(buff[1]), "No GPS detected");
    Serial.println(buff[1]);
    return;
  }
  if (!gps.location.isValid()) {
    if (millis() - gpsMap > 1000) {
        snprintf(buff[0], sizeof(buff[0]), "T-Beam GPS");
        snprintf(buff[1], sizeof(buff[1]), "Positioning(%llu)", gpsSec++);
        Serial.println(buff[1]);
        gpsMap = millis();
    }
  } else {
    if (millis() - gpsMap > 1000) {
        snprintf(buff[0], sizeof(buff[0]), "UTC:%d:%d:%d", gps.time.hour(), gps.time.minute(), gps.time.second());
        snprintf(buff[1], sizeof(buff[1]), "LNG:%.4f", gps.location.lng());
        snprintf(buff[2], sizeof(buff[2]), "LAT:%.4f", gps.location.lat());
        snprintf(buff[3], sizeof(buff[3]), "satellites:%u", gps.satellites.value());
        Serial.println(buff[0]);
        Serial.println(buff[1]);
        Serial.println(buff[2]);
        Serial.println(buff[3]);
        gpsMap = millis();
    }
  }*/
}
