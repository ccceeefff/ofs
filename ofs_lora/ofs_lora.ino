#include "OFSLookup.h"
#include <SPI.h>
#include <AltSoftSerial.h>
#include <SD.h>
#include <avr/pgmspace.h>
#include <ctype.h>

#include "src/lmic/src/lmic.h"
#include "src/lmic/src/hal/hal.h"
#include "src/tinygpsplus/TinyGPS++.h"

#define USE_RANDOM_DATA 1

//IR sensor analog pins
#define irSensorPin0   A0  
#define irSensorPin1   A2 
uint8_t irSensor0=0;
uint8_t irSensor1=0;

//GPS communiction setup
static const int RXPin = 8,TXPin = 9;
static const uint32_t GPSBaud = 38400;
TinyGPSPlus gps;                        // The TinyGPS++ object
AltSoftSerial ss(RXPin, TXPin);         // The serial connection to the GPS device

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = 4;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//static const u1_t PROGMEM DEVEUI[8]={ 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0xEF, 0xBE, 0xEF };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t samplejob;

#define LORA_DATA_PORT 1
#define NUM_BUFFERS 2
#define SAMPLES_PER_BUFFER 10

static uint8_t dataBuffer[NUM_BUFFERS][SAMPLES_PER_BUFFER];
static uint8_t currentBuffer = 0;
static uint8_t currentIndex = 0;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned SAMPLE_INTERVAL = 1;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 4,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 2,
    .dio = {3, 5, LMIC_UNUSED_PIN},
};

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

            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);

            
            // Start job (sending automatically starts OTAA too)
            do_sample(&samplejob);
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

void do_send(uint8_t bufferIndex){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(LORA_DATA_PORT, dataBuffer[bufferIndex], SAMPLES_PER_BUFFER, 0);
        Serial.println(F("Packet queued"));
    }
}

void do_sample(osjob_t *j){
  // sample every second
  // send up 5 (x2) samples every 5 seconds
  Serial.println(F("Take sample"));

#if USE_RANDOM_DATA
  irSensor0 = random(255);
  irSensor1 = random(255);
#else
  irSensor0 = analogRead(irSensorPin0);
  irSensor1 = analogRead(irSensorPin1);
#endif
  
  irSensor0 = pgm_read_word(&Sensor0[irSensor0]);
  irSensor1 = pgm_read_word(&Sensor1[irSensor1]); 

  // place data in buffer
  dataBuffer[currentBuffer][currentIndex++] = irSensor0;
  dataBuffer[currentBuffer][currentIndex++] = irSensor1;

  if(currentIndex >= SAMPLES_PER_BUFFER){
    // send out buffer and move to next index
    do_send(currentBuffer);
    currentIndex = 0;
    currentBuffer = (currentBuffer+1) % NUM_BUFFERS;
  }
  
  os_setTimedCallback(&samplejob, os_getTime()+sec2osticks(SAMPLE_INTERVAL), do_sample);

  // write to SD Card
}

void setup() {
    Serial.begin(115200);
//    ss.begin(GPSBaud);
//    
//    pinMode(10, OUTPUT);
//     // see if the card is present and can be initialized:
//    if (!SD.begin(chipSelect)) {
//      Serial.println("Card failed, or not present");
//      // don't do anything more:
//      return;
//    }
//    Serial.println("card initialized.");

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
//    LMIC_enableSubBand(1);
    LMIC_selectSubBand(0);
    LMIC_startJoining();

    // will start sampling after joining the network
}

void loop() {
    os_runloop_once();
}
