#include <ArduinoJson.h>

#include <Adafruit_Sensor.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

#include <Ticker.h>
#include <lmic.h>
#include <hal/hal.h>
//#include <DHT.h> //library for DHT-sensor
#include "settings.h"
#include "mhz19.h"
//#include <SSD1306Wire.h>
//#include <soc/efuse_reg.h>
#define LEDPIN 2
#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15


HardwareSerial sensor(1);

static bool exchange_command(uint8_t cmd, uint8_t data[], int timeout)
{
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

static bool read_temp_co2(int *co2, int *temp)
{
    uint8_t data[] = {0, 0, 0, 0, 0, 0};
    bool result = exchange_command(0x86, data, 3000);
    if (result) {
        *co2 = (data[0] << 8) + data[1];
        *temp = data[2] - 40;
#if 1
        char raw[32];
        sprintf(raw, "RAW: %02X %02X %02X %02X %02X %02X", data[0], data[1], data[2], data[3], data[4], data[5]);
        Serial.println(raw);
#endif
    }
    return result;
}

Adafruit_BME280 bme;

void bmestart() {
  bool bmestatus;
  Wire.begin(13,15);
  bmestatus = bme.begin(0x76);
      if (!bmestatus) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
}

Ticker secondTick;
      int wt = 0;
#include "esp_system.h"
const int wdtTimeout = 300000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

void IRAM_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart_noos();
}

void wtl(){
  wt++;
  if (wt == 61200){
    Serial.println();
    Serial.println("wt reset");
    ESP.restart();
  }
}
static uint8_t mydata[] = "Hello, world!";
static char esp_id[16];
unsigned int counter = 0;

//SSD1306Wire display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

 //  rxPin = 9; txPin = 10;

/*********************************
 * TODO: Change the following keys
 * NwkSKey: network session key, AppSKey: application session key, and DevAddr: end-device address
 *********************************/
//These are in settings

void os_getDevEui(u1_t* buf){ memcpy_P(buf, DEVEUI, 8); }
void os_getArtEui (u1_t* buf) { }
//void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 300;
char TTN_response[30];

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};
void do_send(osjob_t* j){
    // Payload to send (uplink)
    digitalWrite(2, HIGH);
         timerWrite(timer, 0); //reset timer (feed watchdog)
    int co2, temp;

    float Temp = bme.readTemperature();
    float hum = bme.readHumidity();
    
    
    if (read_temp_co2(&co2, &temp)) {
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
    }

     hum = round(hum);   
     char message[110];
  /*  snprintf(message, sizeof(message), "{\"chipid\":%s,\"sensor\":\"BKS\",\"millis\":%d,\"data\":[\"%s\\
",%d,\"_\",0,\"_\",0]}", esp_id, millis(), "co2", co2 );*/

DynamicJsonBuffer jsonBuffer(200);
JsonObject& root = jsonBuffer.createObject();
root["id"] = esp_id;
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
          Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
          Serial.println();
          
          for ( i = 0 ; i < LMIC.dataLen ; i++ )
            TTN_response[i] = LMIC.frame[LMIC.dataBeg+i];
          TTN_response[i] = 0; 
          LMIC.dataLen = 0;     
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
    }
}
/*
int getChipRevision()
{
  return (REG_READ(EFUSE_BLK0_RDATA3_REG) » (EFUSE_RD_CHIP_VER_RESERVE_S)&&EFUSE_RD_CHIP_VER_RESERVE_V) ;
}

void printESPRevision() {
  Serial.print("REG_READ(EFUSE_BLK0_RDATA3_REG) ");
  Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG), BIN);

  Serial.print("EFUSE_RD_CHIP_VER_RESERVE_S ");
  Serial.println(EFUSE_RD_CHIP_VER_RESERVE_S, BIN);

  Serial.print("EFUSE_RD_CHIP_VER_RESERVE_V ");
  Serial.println(EFUSE_RD_CHIP_VER_RESERVE_V, BIN);

  Serial.println();

  Serial.print("Chip Revision (official version): ");
  Serial.println(getChipRevision());

  Serial.print("Chip Revision from shift Operation ");
  //Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG) » 15, BIN);
}
*/
void setup() {
    Serial.begin(115200);
    pinMode(2, OUTPUT);
    bmestart();
    //secondTick.attach(1,wtl);
    timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);      
    sensor.begin(9600, SERIAL_8N1, 23, 22);
    delay(1500);   // Give time for the seral monitor to start up
    Serial.println(F("Starting..."));

    //printESPRevision();

    uint64_t chipid;  
    chipid=ESP.getEfuseMac();
    Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));
    Serial.printf("%08X\n",(uint32_t)chipid);
    sprintf(esp_id, "%08X", (uint32_t)chipid);

    // Use the Blue pin to signal transmission.
    pinMode(LEDPIN,OUTPUT);

   // reset the OLED
   pinMode(OLED_RESET,OUTPUT);
   delay(50);
   
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
    LMIC_setDrTxpow(DR_SF9,14);

    // Start job
    do_send(&sendjob);
}
void loop() {
    os_runloop_once();
}
