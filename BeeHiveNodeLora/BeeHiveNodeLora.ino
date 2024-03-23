/*******************************************************************************
 * Based on example from MCCI LoRaWAN LMIC Library (https://github.com/mcci-catena/arduino-lmic/tree/master)
 * and project from https://github.com/Christophe45/ESP32-Lorawa
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_BME280.h>
#include <HX711.h>
#include <WiFi.h>
#include <driver/adc.h>


/*******************************************************************************
 * LORA Setup
 *******************************************************************************/

#include "device_config.h"
#include "lmic_esp32_sleep.h"

// Pin mapping for LORA
const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 13,
  // LBT cal for the Adafruit Feather M0 LoRa, in dB
  //  .dio = {27, 14, 17}
  .dio = { 27, 14, LMIC_UNUSED_PIN }
};

// Init addresses
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// Setup jobs
static uint8_t txBuffer[22]; //message buffer
static osjob_t sendjob; //job variable
const unsigned TX_INTERVAL = 60; //TX in every this many secconds (or more)


/*******************************************************************************
 * Deep sleep
 *******************************************************************************/
 
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
//#define TIME_TO_SLEEP 7200      /* Time ESP32 will go to sleep (in seconds) */
#define TIME_TO_SLEEP 30 /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;
int wakeup_reason;


/*******************************************************************************
 * Sensors
 *******************************************************************************/

// DS18B20 sensor
#define ONE_WIRE_BUS 33
#define TEMPERATURE_PRECISION 100
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
float tempruche;

//BME sensor
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;  // I2C

// supply control
const int transistorPin = 16;

//scale
#define DOUT 25  //for ESP32 wemos
#define CLK 26
float weight = 0;
float weightcorrection = -5.6;
float calibration_factor = -20800;  //You must change this factor depends on your scale,sensors and etc.

// Vbat is connected to GPIO 32
const int batPin = 32;

// variable for storing the Vbat value
int batValue = 0;
float volt = 0.00;
float vbat = 0.00;


void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
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
              otaaDevAddr = LMIC.devaddr;
              memcpy_P(otaaNetwKey, LMIC.nwkKey, 16);
              memcpy_P(otaaApRtKey, LMIC.artKey, 16);


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
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);


            storeFrameCounters();
            // Schedule next transmission
            Serial.println("Good night...");
            adc_power_off();
            digitalWrite(transistorPin, LOW);
            Deep_Sleep_Now();
            do_send(&sendjob);
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
        esp_sleep_wakeup_cause_t wakeup_reason;
        wakeup_reason = esp_sleep_get_wakeup_cause();

        Serial.println("Wakeup was caused by deep sleep or starting");

        // Prepare upstream data transmission at the next possible time.
        // read the temperature from the BME280
        float temp = bme.readTemperature();
        Serial.print("Temperature: ");
        Serial.print(temp);
        Serial.println(" *C");
        float humidity = bme.readHumidity();
        Serial.print("humidité: ");
        Serial.print(humidity);
        Serial.println("% ");

        // read weight from the HX711
        HX711 scale;
        scale.begin(DOUT, CLK);
        scale.set_scale(calibration_factor);  //Adjust to this calibration factor
        Serial.print("Reading: ");
        weight = scale.get_units();
        weight = weight + weightcorrection;
        Serial.print(weight, 2);
        Serial.println(" kg");  // You can change this to other type of weighing value and re-adjust the calibration factor.
        Serial.print(" calibration_factor: ");
        Serial.println(calibration_factor);

        Serial.println("Boot number: " + String(bootCount));

        // Reading potentiometer value
        batValue = analogRead(batPin);
        volt = batValue;
        vbat = volt / 732;
        Serial.print("Voltage = ");
        Serial.println(vbat);

        // reading DS18B20 value
        DS18B20.requestTemperatures();
        tempruche = DS18B20.getTempCByIndex(0);  // Sensor 0 will capture Temp in Celcius
        Serial.print("tempruche: ");
        Serial.println(tempruche);

        // Alarm!!
        int alarm;
        alarm = 0;
        Serial.print("alarm:");
        Serial.println(alarm);

        // Prepare payload for transmission

        uint8_t txBuffer[14];

        int16_t Celcius = LMIC_f2sflt16(temp);
        byte tempLow = lowByte(Celcius);
        byte tempHigh = highByte(Celcius);
        txBuffer[0] = tempLow;
        txBuffer[1] = tempHigh;

        int16_t Humidity = LMIC_f2sflt16(humidity);
        byte humLow = lowByte(Humidity);
        byte humHigh = highByte(Humidity);
        txBuffer[2] = humLow;
        txBuffer[3] = humHigh;

        int16_t Weight = LMIC_f2sflt16(weight);
        byte weightLow = lowByte(Weight);
        byte weightHigh = highByte(Weight);
        txBuffer[4] = weightLow;
        txBuffer[5] = weightHigh;

        uint16_t BootCount = bootCount;
        byte bootLow = lowByte(BootCount);
        byte bootHigh = highByte(BootCount);
        txBuffer[6] = bootLow;
        txBuffer[7] = bootHigh;
        
        uint16_t Vbat = LMIC_f2sflt16(vbat);
        byte vBatLow = lowByte(Vbat);
        byte vBatHigh = highByte(Vbat);
        txBuffer[8] = vBatLow;
        txBuffer[9] = vBatHigh;

        int16_t Tempruche = LMIC_f2sflt16(tempruche);
        byte temp2Low = lowByte(Tempruche);
        byte temp2High = highByte(Tempruche);
        txBuffer[10] = temp2Low;
        txBuffer[11] = temp2High;

        uint16_t Alarm = LMIC_f2sflt16(alarm);
        byte alarmLow = lowByte(Alarm);
        byte alarmHigh = highByte(Alarm);
        txBuffer[12] = alarmLow;
        txBuffer[13] = alarmHigh;

        LMIC_setTxData2(1, txBuffer, sizeof(txBuffer)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    //Turn off WiFi and Bluetooth
    WiFi.mode(WIFI_OFF);
    btStop();

    pinMode(transistorPin, OUTPUT);
    digitalWrite(transistorPin, HIGH);
    delay(1000);
    //Serial.println(F("TTN Mapper"));
    bme.begin(0x76);
    DS18B20.begin();
    ++bootCount;
    delay(300);
    delay(500);
    setCpuFrequencyMhz(80);

    esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);
    delay(1500);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    esp_reset_reason_t reason = esp_reset_reason();
    if ((reason == ESP_RST_DEEPSLEEP) || (reason == ESP_RST_SW)) {
      LMIC_setSession(0x1, otaaDevAddr, otaaNetwKey, otaaApRtKey);
    }
    setOrRestorePersistentCounters();

    LMIC_setDrTxpow(DR_SF7, 14);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

void Deep_Sleep_Now() { // New function - moded code out of void loop so that the sleep function can be called if we fail to connect to Wi-Fi or Blynk
  esp_sleep_enable_timer_wakeup((uint64_t)(TIME_TO_SLEEP)*uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
  Serial.println("Going to sleep now");
  Serial.flush();
  esp_deep_sleep_start();

  delay(2000);
}