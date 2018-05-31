/*
 * This uses https://github.com/VekotinVerstas/arduino-lmic as lora library. Install it to your arduino/library first.
 */
 
#include "mhz19.h"
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
#include "settings.h"
#ifdef useBME
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
Adafruit_BME280 bme;
bool bmestatus;
#endif
#ifdef useDHT
#include <DHT.h>
#define DHTPIN 15
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
#endif

#define LEDPIN 2  // Used to blink blue led while sending data

int wt = 0;
const int wdtTimeout = 14000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

HardwareSerial sensor(1);

static char esp_id[16];
unsigned int counter = 0;
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 5*60;
char TTN_response[30];

void IRAM_ATTR resetModule() {
  Serial.print(F("SW watchdog time out: "));
  ets_printf("reboot\n");
  esp_restart_noos();
}

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  timer = timerBegin(0, 8000, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);
  sensor.begin(9600, SERIAL_8N1, 23, 22);
#ifdef useDHT
  dht.begin();
#endif

  bmestart(13, 15);
  sprintf(s_id, "ESP_%02X", BOXNUM);
  Serial.print("Co2TTGO name: ");
  Serial.println(s_id);

  Serial.print("Co2TTGO version: ");
  Serial.println(version);

  uint64_t chipid;
  chipid = ESP.getEfuseMac();
  Serial.printf("ESP32 Chip ID = %04X", (uint16_t)(chipid >> 32));
  Serial.printf("%08X\n", (uint32_t)chipid);
  sprintf(esp_id, "%08X", (uint32_t)chipid);

  Serial.printf("LORA dev id: 0x%08X\n", (uint64_t)DEVADDR);
  Serial.printf("LORA dev eui: %01X%01X%01X%01X%01X%01X%01X%01X\n", DEVEUI[0], DEVEUI[1], DEVEUI[2], DEVEUI[3], DEVEUI[4], DEVEUI[5], DEVEUI[6], DEVEUI[7]);
  // Use the Blue pin to signal transmission.
  pinMode(LEDPIN, OUTPUT);
  
  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  //LMIC_setDrTxpow(DR_SF11,14);
  LMIC_setDrTxpow(DR_SF9, 14);

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}

static bool exchange_command(uint8_t cmd, uint8_t data[], int timeout) {
  // create command buffer
  uint8_t buf[9];
  int len = prepare_tx(cmd, data, buf, sizeof(buf));

  // send the command
  sensor.write(buf, len);

  // wait for response
  long start = millis();
  while ((millis() - start) < timeout) {
    if (sensor.available() > 0) {
      uint8_t b = sensor.read();
      if (process_rx(b, cmd, data)) {
        return true;
      }
    }
  }
  return false;
}

static bool read_temp_co2(int *co2, int *temp) {
  uint8_t data[] = {0, 0, 0, 0, 0, 0};
  bool result = exchange_command(0x86, data, 3000);
  if (result) {
    *co2 = (data[0] << 8) + data[1];
    *temp = data[2] - 40;
    char raw[32];
    sprintf(raw, "Raw co2 sensor data: %02X %02X %02X %02X %02X %02X", data[0], data[1], data[2], data[3], data[4], data[5]);
    Serial.println(raw);
  }
  return result;
}

void bmestart(int pin1, int pin2) {
#ifdef useBME
  Wire.begin(pin1, pin2); //13, 15
  bmestatus = bme.begin(0x76);
  if (!bmestatus) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
#endif
}

void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

void os_getArtEui (u1_t* buf) { }

void os_getDevKey (u1_t* buf) { }

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void do_send(osjob_t* j) {
  // Payload to send (uplink)
  digitalWrite(2, HIGH);
  timerWrite(timer, 0); //reset timer (feed watchdog)

  int co2=0, temp=0;
  float Temp=0, hum=0;

#ifdef useBME
  if (bmestatus) {
    Temp = bme.readTemperature();
    hum = bme.readHumidity();
    }
#endif

#ifdef useDHT
  Temp = dht.readTemperature();
  hum = dht.readHumidity();
  if (isnan(Temp) || isnan(hum)) {
    Serial.println("Failed to read from DHT sensor!");
    Temp = 0;
    hum = 0;
  }
#endif

  if (!read_temp_co2(&co2, &temp)) {
    Serial.println("Co2 read failed.");
    co2=0;
    temp=0;
    //return;
  }

  Serial.println("*****************************************************************");
  Serial.print("CO2:");
  Serial.println(co2, DEC);
  Serial.print("TEMP(CO2-sensori)");
  Serial.println(temp);
  Serial.print("TEMP: ");
  Serial.println(Temp);
  Serial.print("HUM: ");
  Serial.println(hum, DEC);
  Serial.println("*****************************************************************");

  hum = round(hum);
  char message[110];

  DynamicJsonBuffer jsonBuffer(200);
  JsonObject& root = jsonBuffer.createObject();
  root["id"] = s_id;
  root["sensor"] = "BKS";

  JsonObject& data = root.createNestedObject("data");
  data["co2"] = co2;
  data["temp1"] = temp;
  data["temp2"] = Temp;
  data["hum"] = hum;

  root.printTo(message);

  Serial.println(message);
  digitalWrite(2, LOW);
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, (uint8_t*)message, strlen(message), 0);
    //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
    Serial.println(F("Sending uplink packet..."));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
  if (ev == EV_TXCOMPLETE) {
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK) {
      Serial.println(F("Received ack"));
    }
    if (LMIC.dataLen) {
      int i = 0;
      // data received in rx slot after tx
      Serial.print(F("Data Received: "));
      Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
      Serial.println();

      for ( i = 0 ; i < LMIC.dataLen ; i++ )
        TTN_response[i] = LMIC.frame[LMIC.dataBeg + i];
      TTN_response[i] = 0;
      LMIC.dataLen = 0;
    }
    // Schedule next transmission
    Serial.println("Schedule next transmission");
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
  }
}
