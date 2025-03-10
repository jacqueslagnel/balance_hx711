#include "CubeCell_NeoPixel.h"
#include "FRAM.h"
// #include "FRAM_MULTILANGUAGE.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <HX711.h>
#include <LoRaWan_APP.h>
#include <OneWire.h>

#define VEXT_PIN GPIO11
#define ONE_WIRE_INT GPIO12
#define ONE_WIRE_EXT GPIO10
#define NEOPIXEL_DIN_PIN GPIO5

#define VEXT1_PIN GPIO7
#define VEXT_VBAT_PIN GPIO15
#define INT_GPIO GPIO6

// HX711 circuit wiring
// const int LOADCELL_DOUT_PIN = GPIO9; // blue data
// const int LOADCELL_SCK_PIN = GPIO8; // violet clk
#define LOADCELL_DOUT_PIN GPIO9
#define LOADCELL_SCK_PIN GPIO8

#define COEFVBAT 0.07100591716f
#define TEMPFACTOR 0.230800f
// *****************************************************************************
// ********************* neopixel colors ***************************************
// *****************************************************************************
CubeCell_NeoPixel mypixels(1, NEOPIXEL_DIN_PIN, NEO_GRB + NEO_KHZ800);
typedef enum mycolor {
    RED,
    GREEN,
    BLUE,
    YELLOW,
    VIOLET,
} mycolor_t;

// *****************************************************************************
// ************************** LoRaWAN ******************************************
// *****************************************************************************

/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

#define LORAINRAE 1
#ifdef LORAINRAE
// OTAA  chip id and ssl key OK
// hh=$(openssl rand -hex 8);echo $hh;echo $hh|sed 's/\(..\)/0x\1, /g'
// 2824b13d35e0f2ab
uint8_t devEui[] = { 0x28, 0x24, 0xb1, 0x3d, 0x35, 0xe0, 0xf2, 0xab };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// hh=$(openssl rand -hex 16);echo $hh;echo $hh|sed 's/\(..\)/0x\1, /g'
// 3d4363ebad23b1dd4c8d3d8536cfdea6
uint8_t appKey[] = { 0x3d, 0x43, 0x63, 0xeb, 0xad, 0x23, 0xb1, 0xdd, 0x4c, 0x8d, 0x3d, 0x85, 0x36, 0xcf, 0xde, 0xa6 };
#else
// OTAA : maison joining ok but where? But no TTN
uint8_t devEui[] = { 0x22, 0x32, 0x33, 0x00, 0x00, 0x88, 0x88, 0x02 }; // ori 0x02
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x66, 0x01 };
#endif

/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67 };
uint32_t devAddr = (uint32_t)0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 30000; // 30 sec

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
 * Number of trials to transmit the frame, if the LoRaMAC layer did not
 * receive an acknowledgment. The MAC performs a datarate adaptation,
 * according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
 * to the following table:
 *
 * Transmission nb | Data Rate
 * ----------------|-----------
 * 1 (first)       | DR
 * 2               | DR
 * 3               | max(DR-1,0)
 * 4               | max(DR-1,0)
 * 5               | max(DR-2,0)
 * 6               | max(DR-2,0)
 * 7               | max(DR-3,0)
 * 8               | max(DR-3,0)
 *
 * Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
 * the datarate, in case the LoRaMAC layer did not receive an acknowledgment
 */
uint8_t confirmedNbTrials = 4;

// *****************************************************************************
// ********************** 1 wire ***********************************************
// *****************************************************************************

// Data wire is plugged into port 4 on the Arduino
// UART_TX2 P4_5

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
// int16_t tempint = -127;
// int16_t tempext = -127;
// ds count: 1     ADR: 28 4B 62 A5 C 0 0 92       CRC OK
// ds count: 2     ADR: 28 77 B FF D 0 0 88        CRC OK
// Addr: 28 77 B FF D 0 0 88       Time: 77 ms     Data = 1 60 1 0 0 1F FF 10 10 DD  CRC=DD        Temperature = 22.00
// Addr: 28 4B 62 A5 C 0 0 92      Time: 75 ms     Data = 1 B0 1 0 0 1F FF 10 10 52  CRC=52        Temperature = 27.00
// TempInt:        22      TempExt:        27

OneWire ds18b20_int(ONE_WIRE_INT); // on pin GPIO1 PIN 6 (a 4.7K resistor is necessary)
OneWire ds18b20_ext(ONE_WIRE_EXT); // on pin GPIO1 PIN 6 (a 4.7K resistor is necessary)
// OneWire ds18b20_ext(ONE_WIRE_BUS_EXT); // on digital pin 2

// *****************************************************************************
// ************************* hx711 *********************************************
// *****************************************************************************

// in arduino.h:
// comment #define pinMode(pin,mode) PINMODE_ ## mode(pin)
// put in the HX711.cpp:
// #define DOUT_MODE INPUT
HX711 scale;
// int16_t poids_length = 0;
// long hx711_adc = 0;
int16_t poids = 0;
// uint16_t tout = 0;
float units10 = 0.0;
// long offset_rtc = 0;
// int16_t offset_vbat = 0;
int16_t delta_vbat = 0;
#define DELTAMV 507.0
#define DELTAADC 36.0
float weight_scale = 112.867886670f;
struct hx711_data_t {
    int32_t offset_adc;
    int16_t offset_vbat;
    int16_t offset_tempint;
    int16_t offset_tempext;
    int32_t poids_int; // float *10
    // int32_t poids_int_cor;
    int32_t adc;
    int16_t tempint;
    int16_t tempext;
    int16_t vbat;
    int8_t fault;
    ;
};
hx711_data_t hx711_data = { 0, 0, -12700, -12700, 0, 0, -12700, -12700, 0, 0 };
// ********************* variables 4 derivative comp Temp *********************
// int16_t temp_old = 0;
// float derivative = 0.0;
// float derivative_old = 0.0;

// *****************************************************************************
// **************************** FRAM *******************************************
// *****************************************************************************
/* Example code for the Adafruit I2C EEPROM/FRAM breakout */
/* Connect SCL    to SCL
   Connect SDA    to SDA
   Connect VDD    to 3 - 5V DC
   Connect GROUND to common ground */

FRAM fram;

// Adafruit_EEPROM_I2C i2ceeprom;
//  Adafruit_FRAM_I2C i2ceeprom;
#define EEPROM_ADDR 0X50
// b1010000 (0x50) the default address with A0=A1=A2=0

// *****************************************************************************
// ******************** global vars ***********************************
// *****************************************************************************
boolean isallreadyjoined = false;
TimerSysTime_t sysTimeCurrent;
uint8_t global_fault = 0;
boolean send_parameters = true;
boolean setzero = false;
// if true, next uplink will add MOTE_MAC_DEVICE_TIME_REQ
bool timeReq = false;
uint16_t vbat_mv = 0;
int8_t action = 0;
float poidsf = 0.0;

// reset timer
//  sysTimeCurrent.Seconds=0;
//  sysTimeCurrent.SubSeconds=0;
//  TimerSetSysTime(sysTimeCurrent);

// *****************************************************************************
// ******************** functions declaration **********************************
// *****************************************************************************
void dev_time_updated()
{
    Serial.println("Once device time updated, this function run\r\n");
}

void VextON(void);
void VextOFF(void);
void Vhx711ON(void);
void Vhx711OFF(void);
void set_color(mycolor_t cc, uint8_t bright, uint16_t ontimems);
static void prepareTxFrame(uint8_t port);
uint16_t readBatLevel(void);
int16_t round_float(float mm);
void onWakeUp(void);

// int8_t get_temperature(void);
// int16_t get_temperature(byte addr[8]);
// int8_t OneWireScan(void);
// int16_t get_temperature_int(void);
// int16_t get_temperature_ext(void);
int16_t get_temperature(OneWire& myds, uint8_t mypin);

void downLinkDataHandle(McpsIndication_t* mcpsIndication);
void scale_init(void);
int16_t get_weight_g(void);
long hx711_read_ave(void);
uint8_t get_weight_vbat_corrected(void);
bool fram_write(uint16_t adr, int16_t data);
void fram_dump(void);
void fram_test(void);

// *****************************************************************************
// ********************* SETUP *************************************************
// *****************************************************************************

// ************************** timout ********************************
volatile uint16_t time_sec_cycle = 60; // 5 * 60; // seconds
// ************************************************************************************

// ************************** if debug we use print ********************
#define DEBUGPRINT 1
 #define TESTING 1
//    *********************************************************************

void setup()
{
    Serial.begin(115200);
    global_fault = 0;
    delay(100);
    pinMode(NEOPIXEL_DIN_PIN, OUTPUT);
    pinMode(VEXT_PIN, OUTPUT);
    pinMode(VEXT1_PIN, OUTPUT);
    pinMode(VEXT_VBAT_PIN, OUTPUT);

    digitalWrite(VEXT_PIN, HIGH);
    digitalWrite(VEXT1_PIN, HIGH);
    digitalWrite(VEXT_VBAT_PIN, HIGH);

    // ------------------------------ wake up by GPIO ---------------------------
    pinMode(INT_GPIO, INPUT);
    attachInterrupt(INT_GPIO, onWakeUp, FALLING);
    scale_init();
    if (hx711_data.offset_adc == 0) {
        scale_init();
    }

    // ---------------- test sensors --------------------------------------------

#ifdef TESTING
    while (1) {
#endif
        global_fault = 0;
        set_color(VIOLET, 50, 125);
        vbat_mv = readBatLevel();
        get_weight_vbat_corrected();

        hx711_data.tempint = get_temperature(ds18b20_int, ONE_WIRE_INT);
        hx711_data.tempext = get_temperature(ds18b20_ext, ONE_WIRE_EXT);
        byte yy = 0;
        while (hx711_data.tempint < -120 && yy < 3) {
            hx711_data.tempint = get_temperature(ds18b20_int, ONE_WIRE_INT);
            yy++;
            delay(100);
        }
        yy = 0;
        while (hx711_data.tempext < -120 && yy < 3) {
            hx711_data.tempext = get_temperature(ds18b20_ext, ONE_WIRE_EXT);
            yy++;
            delay(100);
        }
        // temp_old = tempext;
        // derivative = 0.00;
        // derivative_old = 0.00;
#ifdef DEBUGPRINT
        Serial.print("global_fault:\t");
        Serial.print(global_fault);
        Serial.print("\tADC-offset:\t");
        Serial.print(hx711_data.offset_adc);
        Serial.print("\tVbat-offset:\t");
        Serial.print(hx711_data.offset_vbat);
        Serial.print("\tTempint-offset:\t");
        Serial.print(hx711_data.offset_tempint / 100.0);
        Serial.print("\tTempExt-offset:\t");
        Serial.print(hx711_data.offset_tempext / 100.0);

        Serial.print("\tTempInt:\t");
        Serial.print(hx711_data.tempint / 100.0);
        Serial.print("\tTempExt:\t");
        Serial.print(hx711_data.tempext / 100.0);

        Serial.print("\tVbat:\t");
        Serial.print(vbat_mv);

        Serial.print("\tADC:\t");
        Serial.print(hx711_data.adc);
        Serial.print("\tpoids:\t");
        Serial.println(hx711_data.poids_int / 10.0);

        // poids = round_float(((double)(hx711_data.adc - hx711_data.offset_adc) / hx711_data.scale));
        // units10 = roundf(((double)(hx711_data.adc - hx711_data.offset_adc) / hx711_data.scale) * 10.000) / 10.000;
        // Serial.printf("%0.1f\t%d", units10, poids);
        // Serial.print("\tpoids cor:\t");
        // Serial.printf("%0.1f\t%d\terrors: %d\n", hx711_data.poids_float, hx711_data.poids_int, global_fault);
#endif
        Serial.flush();
        VextOFF();
        Vhx711OFF();
#ifdef TESTING
        delay(5000);
    }
#endif
    // ------------------------------ set LoRaWAN -------------------------------
    send_parameters = true; // we send the parameters on port 3
    setzero = false; // we already init the hx711
    deviceState = DEVICE_STATE_INIT;
    LoRaWAN.ifskipjoin();
}

// *****************************************************************************
// ********************************* main loop *********************************
// *****************************************************************************
void loop()
{
    // ------------------------------ Lora -------------------------------------------
    switch (deviceState) {

    case DEVICE_STATE_INIT: {
#ifdef DEBUGPRINT
        printDevParam();
#endif
        detachInterrupt(INT_GPIO);
        pinMode(INT_GPIO, OUTPUT);
        set_color(VIOLET, 150, 125);
        isallreadyjoined = false;
        LoRaWAN.init(loraWanClass, loraWanRegion);
        deviceState = DEVICE_STATE_JOIN;
        break;
    }

    case DEVICE_STATE_JOIN: {
        LoRaWAN.join();
        set_color(VIOLET, 50, 125);
        pinMode(INT_GPIO, INPUT);
        attachInterrupt(INT_GPIO, onWakeUp, FALLING);
        break;
    }

    case DEVICE_STATE_SEND: {
        detachInterrupt(INT_GPIO);
        pinMode(INT_GPIO, OUTPUT);
        global_fault = 0;
        if (isallreadyjoined == false) {
            if (IsLoRaMacNetworkJoined == true) {
                set_color(GREEN, 50, 125);
                isallreadyjoined = true;
            } else {
                set_color(RED, 150, 500);
                isallreadyjoined = false;
            }
        }

        if (IsLoRaMacNetworkJoined == true) {
            set_color(BLUE, 50, 125);
        }

        if (hx711_data.offset_adc == 0 || setzero == true) {
            scale_init();
            send_parameters = true;
            setzero = false;
        }

        if (send_parameters == true) {
            appPort = 3;
            prepareTxFrame(appPort);
            send_parameters = false;
        } else {
            vbat_mv = readBatLevel();
            get_weight_vbat_corrected();
            hx711_data.tempint = get_temperature(ds18b20_int, ONE_WIRE_INT);
            hx711_data.tempext = get_temperature(ds18b20_ext, ONE_WIRE_EXT);
            byte yy = 0;
            while (hx711_data.tempint < -120 && yy < 3) {
                hx711_data.tempint = get_temperature(ds18b20_int, ONE_WIRE_INT);
                yy++;
                delay(100);
            }
            yy = 0;
            while (hx711_data.tempext < -120 && yy < 3) {
                hx711_data.tempext = get_temperature(ds18b20_ext, ONE_WIRE_EXT);
                yy++;
                delay(100);
            }
#ifdef DEBUGPRINT
            Serial.print("Timout:\t");
            Serial.print(time_sec_cycle);
            Serial.print("\tFault:\t");
            Serial.print(global_fault, HEX);
            //            Serial.print("\thx711 g float:\t");
            //            Serial.print(hx711_data.poids_float, 1);
            Serial.print("\tg Int:\t");
            Serial.print(hx711_data.poids_int);
            Serial.print("\tTempInt:\t");
            Serial.print(hx711_data.tempint / 100.0);
            Serial.print("\tTempExt:\t");
            Serial.print(hx711_data.tempext / 100.0);
            Serial.print("\tVbat:\t");
            Serial.println(vbat_mv);
            // Serial.printf("Timeout\t%d,%02X,%d,%d,%0.3f\n",time_sec_cycle,global_fault,poids,tempint,(float)vbat_mv/1000.000);
            Serial.flush();
#endif

            Serial.print(">");
            Serial.print(global_fault);
            Serial.print(",");
            Serial.print(hx711_data.offset_adc);
            Serial.print(",");
            Serial.print(hx711_data.offset_vbat);
            Serial.print(",");
            Serial.print(hx711_data.offset_tempint / 100.0);
            Serial.print(",");
            Serial.print(hx711_data.offset_tempext / 100.0);

            Serial.print(",");
            Serial.print(hx711_data.tempint / 100.0);
            Serial.print(",");
            Serial.print(hx711_data.tempext / 100.0);

            Serial.print(",");
            Serial.print(vbat_mv);

            Serial.print(",");
            Serial.print(hx711_data.adc);
            Serial.print(",");
            Serial.println(poidsf);

            appPort = 2;
            prepareTxFrame(appPort);
        }

        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        // TODO: uncomment the 2 lines:
        pinMode(INT_GPIO, INPUT);
        attachInterrupt(INT_GPIO, onWakeUp, FALLING);
        break;
    }
    case DEVICE_STATE_CYCLE: {
        if (!IsLoRaMacNetworkJoined) {
            set_color(RED, 150, 250);
        }

        pinMode(NEOPIXEL_DIN_PIN, OUTPUT);
        pinMode(VEXT_PIN, OUTPUT);
        pinMode(VEXT1_PIN, OUTPUT);
        pinMode(VEXT_VBAT_PIN, OUTPUT);
        digitalWrite(VEXT_PIN, HIGH);
        digitalWrite(VEXT1_PIN, HIGH);
        digitalWrite(VEXT_VBAT_PIN, HIGH);

        // Schedule next packet transmission
        appTxDutyCycle = (uint32_t)(time_sec_cycle) * 1000;
        txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
    }
    case DEVICE_STATE_SLEEP: {
        LoRaWAN.sleep();
        break;
    }
    default: {
        deviceState = DEVICE_STATE_INIT;
        break;
    }
    }
}

// *****************************************************************************
// ********************************* Functions *********************************
// *****************************************************************************

void VextON(void)
{
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
    delay(10);
}

void VextOFF(void)
{
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, HIGH);
}

void Vhx711ON(void)
{
    pinMode(VEXT1_PIN, OUTPUT);
    digitalWrite(VEXT1_PIN, LOW);
    delay(10);
}

void Vhx711OFF(void)
{
    pinMode(VEXT1_PIN, OUTPUT);
    digitalWrite(VEXT1_PIN, HIGH);
}
void set_color(mycolor_t cc, uint8_t bright, uint16_t ontimems)
{
    uint8_t i = 255;
    VextON();
    // pinMode(VEXT_LED_PIN, OUTPUT);
    // digitalWrite(VEXT_LED_PIN, LOW); // SET POWER
    // delay(10);
    mypixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
    mypixels.clear(); // Set all pixel colors to ‘off’
    // mypixels.show();
    mypixels.setBrightness(bright);

    switch (cc) {
    case RED:
        mypixels.setPixelColor(0, mypixels.Color(i, 0, 0));
        mypixels.show(); // Send the updated pixel colors to the hardware.
        break;
    case GREEN:
        mypixels.setPixelColor(0, mypixels.Color(0, i, 0));
        mypixels.show(); // Send the updated pixel colors to the hardware.
        break;
    case BLUE:
        mypixels.setPixelColor(0, mypixels.Color(0, 0, i));
        mypixels.show(); // Send the updated pixel colors to the hardware.
        break;
    case YELLOW:
        mypixels.setPixelColor(0, mypixels.Color(i, i, 0));
        mypixels.show(); // Send the updated pixel colors to the hardware.
        break;
    case VIOLET:
        mypixels.setPixelColor(0, mypixels.Color(i, 0, i));
        mypixels.show(); // Send the updated pixel colors to the hardware.
        break;

    default:
        mypixels.clear();
        mypixels.show();
        mypixels.setPixelColor(0, mypixels.Color(0, 0, 0));
        mypixels.clear(); // off
        mypixels.setBrightness(0);
        mypixels.show();
        delay(1);
        break;
    }
    delay(ontimems);
    mypixels.clear();
    mypixels.show();
    mypixels.setPixelColor(0, mypixels.Color(0, 0, 0));
    mypixels.clear(); // off
    mypixels.setBrightness(0);
    mypixels.show();
    VextOFF();
}

int16_t round_float(float mm)
{
    int16_t poids = 0;
    if (mm < 0) {
        poids = int16_t(mm - 0.5);
    } else {
        poids = int16_t(mm + 0.5);
    }
    return poids;
}

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port)
{
    if (port == 3) { // send the parameters on port 3
        appDataSize = 10;
        // ADC offset
        appData[0] = (hx711_data.offset_adc >> 24) & 0xFF;
        appData[1] = (hx711_data.offset_adc >> 16) & 0xFF;
        appData[2] = (hx711_data.offset_adc >> 8) & 0xFF;
        appData[3] = hx711_data.offset_adc & 0xFF;
        appData[4] = ((hx711_data.offset_vbat) >> 8); // 5
        appData[5] = ((hx711_data.offset_vbat) & 0xFF); // 6
        appData[6] = ((hx711_data.offset_tempint) >> 8); // 5
        appData[7] = ((hx711_data.offset_tempint) & 0xFF); // 5
        appData[8] = ((hx711_data.offset_tempext) >> 8); // 5
        appData[9] = ((hx711_data.offset_tempext) & 0xFF); // 5
        appData[10] = '\0';
    } else if (port == 2) { // send measures in the loop
        appDataSize = 15;
        // code retruned fault
        appData[0] = global_fault; // fault; 0
        appData[1] = (hx711_data.adc >> 24) & 0xFF;
        appData[2] = (hx711_data.adc >> 16) & 0xFF;
        appData[3] = (hx711_data.adc >> 8) & 0xFF;
        appData[4] = hx711_data.adc & 0xFF;
        // code the integer 32 poids float *10
        /*
        appData[5] = (hx711_data.poids_int_cor >> 24) & 0xFF;
        appData[6] = (hx711_data.poids_int_cor >> 16) & 0xFF;
        appData[7] = (hx711_data.poids_int_cor >> 8) & 0xFF;
        appData[8] = hx711_data.poids_int_cor & 0xFF;
        */

        appData[5] = (hx711_data.poids_int >> 24) & 0xFF;
        appData[6] = (hx711_data.poids_int >> 16) & 0xFF;
        appData[7] = (hx711_data.poids_int >> 8) & 0xFF;
        appData[8] = hx711_data.poids_int & 0xFF;

        // code vBat
        appData[9] = ((vbat_mv) >> 8); // 3
        appData[10] = ((vbat_mv) & 0xFF); // 4

        // code Temp ds18b20
        appData[11] = ((hx711_data.tempint) >> 8); // 5
        appData[12] = ((hx711_data.tempint) & 0xFF); // 5
        appData[13] = ((hx711_data.tempext) >> 8); // 5
        appData[14] = ((hx711_data.tempext) & 0xFF); // 5

        appData[15] = '\0';
    } else {
        appDataSize = 0;
        appData[0] = '\0';
    }
}

uint16_t readBatLevel(void)
{
    uint16_t voltage = 0;
    VextOFF();
    detachInterrupt(INT_GPIO);
    pinMode(VEXT_VBAT_PIN, OUTPUT);
    digitalWrite(VEXT_VBAT_PIN, LOW);
    delay(50);

    voltage = getBatteryVoltage();
    voltage = getBatteryVoltage();
    // float vf = (float)voltage * 1.634;
    float vf = (float)voltage * 1.0000;
    voltage = (uint16_t)round_float(vf);
    digitalWrite(VEXT_VBAT_PIN, HIGH);
    pinMode(INT_GPIO, INPUT);
    attachInterrupt(INT_GPIO, onWakeUp, FALLING);
    delay(1);
    return (voltage);
}

void onWakeUp(void)
{
    uint16_t x = 0;
    uint16_t mycnt = 0;
    delay(10);
    if (digitalRead(INT_GPIO) == 1)
        return;
    while (x < 600 && digitalRead(INT_GPIO) == 0) { // max 5Sec
        x++;
        delay(10);
    }

    if ((x > 50) && (x < 350)) { // 1 to 3 sec => send new payload
#ifdef DEBUGPRINT
        Serial.print("Btn send new payload: ");
        Serial.println(x);
        Serial.flush();
#endif
        deviceState = DEVICE_STATE_SEND;
    } else if (x > 500) { // il more than 4Sec => join
#ifdef DEBUGPRINT
        Serial.print("Btn JOIN again:");
        Serial.println(x);
        Serial.flush();
#endif
        deviceState = DEVICE_STATE_INIT;
    } else { // sleep again
#ifdef DEBUGPRINT
        Serial.print("Btn <1Sec:");
        Serial.println(x);
        Serial.flush();
#endif
        // if (txDutyCycleTime < 10000) {
        // deviceState = DEVICE_STATE_CYCLE;
        //} else {
        //    deviceState = DEVICE_STATE_SLEEP;
        //}
        // deviceState = DEVICE_STATE_CYCLE;
    }
}

/*
// https://www.dekloo.net/arduino-raspberry/arduino-capteurs-de-temperature-ds18s20/1020
int8_t OneWireScan(void)
{
    byte i;
    int8_t nbds = 0;
    byte present = 0;
    byte data[12];
    byte addr[8];
    int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;

    VextON();
    delay(150);
    pinMode(ONE_WIRE_BUS, INPUT);
    ds18b20.begin(ONE_WIRE_BUS);

    ds18b20.reset_search();

    while (ds18b20.search(addr)) {
        nbds++;

        Serial.print("ds count: ");
        Serial.print(nbds);
        Serial.print("\tADR: ");
        for (i = 0; i < 8; i++) {
            Serial.print(addr[i], HEX);
            Serial.print(" ");
        }
        if (OneWire::crc8(addr, 7) == addr[7]) {
            Serial.print("\tCRC OK");
        } else {
            Serial.print("\tCRC ERROR");
        }
        Serial.println();
    }

    VextOFF();
    return nbds;

        // if (!ds18b20.search(addr)) {
        //     Serial.print("No more addresses.\n");
        //     ds18b20.reset_search();
        //     VextOFF();
        //     return;
        // }

        // Serial.print("R=");
        // for (i = 0; i < 8; i++) {
        //     Serial.print(addr[i], HEX);
        //     Serial.print(" ");
        // }

        // if (OneWire::crc8(addr, 7) != addr[7]) {
        //     Serial.print("CRC is not valid!\n");
        //     VextOFF();
        //     return;
        // }

        // if (addr[0] == 0x10) {
        //     Serial.print("Device is a DS18S20 family device.\n");
        // } else if (addr[0] == 0x28) {
        //     Serial.print("Device is a DS18B20 family device.\n");
        // } else {
        //     Serial.print("Device family is not recognized: 0x");
        //     Serial.println(addr[0], HEX);
        //     VextOFF();
        //     return;
        // }
        // VextOFF();

}
*/
/*
int16_t get_temperature(byte addr[8])
{
    // https://forum.arduino.cc/t/ds18b20-temperature-sensor-using-onewire-library/699347/10
    float tp = -127.0;
    byte i;
    byte present = 0;
    byte type_s;
    byte data[9];
    // byte addr[8];
    // 9 bits
    // byte dsRes[] = { 0x00, 0x00, 0x1F }; //, 0x1F(0 R1 R0 11111)/0x3F/0x5F/0x7F for 9-10-11-12-bit Resolutio 0.5 deg
    // 10 bits
    byte dsRes[] = { 0x00, 0x00, 0x3F }; //, 0x1F(0 R1 R0 11111)/0x3F/0x5F/0x7F for 9-10-11-12-bit Resolutio 0.25 deg
    // 12bits
    // byte dsRes[] = { 0x00, 0x00,  0x7F}; //, 0x1F(0 R1 R0 11111)/0x3F/0x5F/0x7F for 9-10-11-12-bit Resolutio 0.0625 deg

    VextON();
    pinMode(ONE_WIRE_BUS, INPUT);
    delay(150);
    ds18b20.begin(ONE_WIRE_BUS);

    // Serial.print(" present?:");
    // Serial.println(ds18b20.reset());
    // ds18b20.reset_search();
    // ds18b20.search(addr);
    // ds18b20.reset_search();


    //    ds18b20.reset();
    //    if (OneWire::crc8(addr, 7) != addr[7]) {
    //       // Serial.println("CRC is not valid!");
    //        //  global_fault = global_fault | 0b00000001;
    //        //  VextOFF();
     //       return -127;
     //   }

    // the first ROM byte indicates which chip
    // switch (addr[0]) {
    // case 0x10:
    //     //Serial.println("  Chip = DS18S20"); // or old DS1820
    //     type_s = 1;
    //     break;
    // case 0x28:
    //     //Serial.println("  Chip = DS18B20");
    //     type_s = 0;
    //     break;
    // case 0x22:
    //     //Serial.println("  Chip = DS1822");
    //     type_s = 0;
    //     break;
    // default:
    //     //Serial.println("Device is not a DS18x20 family device.");
    //     return -127;
    // }

    type_s = 0; // DS18B20
    ds18b20.reset();
    ds18b20.select(addr);
    /*
  //   Serial.print("Addr: ");
  //   for (i = 0; i < 8; i++) { // we need 9 bytes
  //       Serial.print(addr[i], HEX);
   //      Serial.print(" ");
   //  }

    // Serial.println(" ");
    ds18b20.write(0x4E);
    ds18b20.write_bytes(dsRes, 3, 1); // set resolution bit
    ds18b20.reset();
    ds18b20.select(addr);
    ds18b20.write(0x44, 1); // start conversion, with parasite power on at the end
    unsigned long prMillis = millis();
    byte busStatus = 0;
    do // keep reading the ststus word until conversion is complete
    {
        busStatus = ds18b20.read(); // keep reading until conversion is done
    } while (busStatus != 0xFF && (millis() - prMillis) < 900); // busStatus = 0xFF means conversion done
    //---------------------------

  //  Serial.print("\tTime: ");
  //  Serial.print(millis() - prMillis);
  //  Serial.print(" ms");

    // delay(250); // 1000mS, maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.
    present = ds18b20.reset();
    if (present < 1) {
        VextOFF();
        return -12700;
    }
    ds18b20.select(addr);
    ds18b20.write(0xBE); // Read Scratchpad

      //  Serial.print("\tData = ");
      //  Serial.print(present, HEX);
      //  Serial.print(" ");

    for (i = 0; i < 9; i++) { // we need 9 bytes
        data[i] = ds18b20.read();
        //  Serial.print(data[i], HEX);
        // Serial.print(" ");
    }
    ds18b20.depower();
    // Serial.print(" CRC=");
    // Serial.print(OneWire::crc8(data, 8), HEX);
    //  Serial.println();

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - data[6];
        }
    } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00)
            raw = raw & ~7; // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20)
            raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40)
            raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
    }
    tp = (float)raw / 16.0;
    // Serial.print("\tTemperature = ");
    // Serial.println(tp);

    // pinMode(ONE_WIRE_BUS, OUTPUT);
    VextOFF();
    return (int16_t)(tp * 100.0);
} */

/*
int16_t get_temperature_int(void)
{
    // https://forum.arduino.cc/t/ds18b20-temperature-sensor-using-onewire-library/699347/10
    float tp = 0;
    byte i;
    byte present = 0;
    byte type_s;
    byte data[9];
    byte addr[8];
    // 9 bits
    // byte dsRes[] = { 0x00, 0x00, 0x1F }; //, 0x1F(0 R1 R0 11111)/0x3F/0x5F/0x7F for 9-10-11-12-bit Resolutio
    // 10 bits
    byte dsRes[] = { 0x00, 0x00, 0x3F }; //, 0x1F(0 R1 R0 11111)/0x3F/0x5F/0x7F for 9-10-11-12-bit Resolutio 0.25 deg
    // 12bits
    // byte dsRes[] = { 0x00, 0x00,  0x7F}; //, 0x1F(0 R1 R0 11111)/0x3F/0x5F/0x7F for 9-10-11-12-bit Resolutio

    // float celsius, fahrenheit;

    VextON();
    delay(150);
    pinMode(ONE_WIRE_INT, INPUT);
    ds18b20_int.begin(ONE_WIRE_INT);

    Serial.print(" present?:");
    Serial.println(ds18b20_int.reset());
    ds18b20_int.reset_search();
    ds18b20_int.search(addr);
    ds18b20_int.reset_search();
    global_fault = global_fault & 0b11111110;

    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        global_fault = global_fault | 0b00000001;
        VextOFF();
        return -127;
    }

    // the first ROM byte indicates which chip
    // switch (addr[0]) {
    // case 0x10:
    //     //Serial.println("  Chip = DS18S20"); // or old DS1820
    //     type_s = 1;
    //     break;
    // case 0x28:
    //     //Serial.println("  Chip = DS18B20");
    //     type_s = 0;
    //     break;
    // case 0x22:
    //     //Serial.println("  Chip = DS1822");
    //     type_s = 0;
    //     break;
    // default:
    //     //Serial.println("Device is not a DS18x20 family device.");
    //     return -127;
    // }

    type_s = 0; // DS18B20
    ds18b20_int.reset();
    ds18b20_int.select(addr);
    for (i = 0; i < 8; i++) { // we need 9 bytes
        Serial.print(addr[i], HEX);
        Serial.print(" ");
    }
    Serial.println(" ");
    ds18b20_int.write(0x4E);
    ds18b20_int.write_bytes(dsRes, 3, 1); // set resolution bit
    ds18b20_int.reset();
    ds18b20_int.select(addr);
    ds18b20_int.write(0x44, 1); // start conversion, with parasite power on at the end
    unsigned long prMillis = millis();
    byte busStatus = 0;
    do // keep reading the ststus word until conversion is complete
    {
        busStatus = ds18b20_int.read(); // keep reading until conversion is done
    } while (busStatus != 0xFF && (millis() - prMillis) < 900); // busStatus = 0xFF means conversion done
    //---------------------------
    Serial.print("Conversion time: ");
    Serial.print(millis() - prMillis);
    Serial.println(" ms");

    // delay(250); // 1000mS, maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.
    present = ds18b20_int.reset();
    ds18b20_int.select(addr);
    ds18b20_int.write(0xBE); // Read Scratchpad

    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    for (i = 0; i < 9; i++) { // we need 9 bytes
        data[i] = ds18b20_int.read();
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    ds18b20_int.depower();
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - data[6];
        }
    } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00)
            raw = raw & ~7; // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20)
            raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40)
            raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
    }
    tp = (float)raw / 16.0;
    Serial.print("  Temperature = ");
    Serial.println(tp);

    // pinMode(ONE_WIRE_BUS, OUTPUT);
    VextOFF();
    // return (int8_t)(round_float(tp));
    return (int16_t)(tp * 100.0);
}

int16_t get_temperature_ext(void)
{
    // https://forum.arduino.cc/t/ds18b20-temperature-sensor-using-onewire-library/699347/10
    float tp = 0;
    byte i;
    byte present = 0;
    byte type_s;
    byte data[9];
    byte addr[8];
    // 9 bits
    // byte dsRes[] = { 0x00, 0x00, 0x1F }; //, 0x1F(0 R1 R0 11111)/0x3F/0x5F/0x7F for 9-10-11-12-bit Resolutio
    // 10 bits
    byte dsRes[] = { 0x00, 0x00, 0x3F }; //, 0x1F(0 R1 R0 11111)/0x3F/0x5F/0x7F for 9-10-11-12-bit Resolutio 0.25 deg
    // 12bits
    // byte dsRes[] = { 0x00, 0x00,  0x7F}; //, 0x1F(0 R1 R0 11111)/0x3F/0x5F/0x7F for 9-10-11-12-bit Resolutio

    // float celsius, fahrenheit;

    VextON();
    delay(150);
    pinMode(ONE_WIRE_EXT, INPUT);
    ds18b20_ext.begin(ONE_WIRE_EXT);

    Serial.print(" present?:");
    Serial.println(ds18b20_ext.reset());
    ds18b20_ext.reset_search();
    ds18b20_ext.search(addr);
    ds18b20_ext.reset_search();
    global_fault = global_fault & 0b11111110;

    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        global_fault = global_fault | 0b00000001;
        VextOFF();
        return -127;
    }

    type_s = 0; // DS18B20
    ds18b20_ext.reset();
    ds18b20_ext.select(addr);
    for (i = 0; i < 8; i++) { // we need 9 bytes
        Serial.print(addr[i], HEX);
        Serial.print(" ");
    }
    Serial.println(" ");
    ds18b20_ext.write(0x4E);
    ds18b20_ext.write_bytes(dsRes, 3, 1); // set resolution bit
    ds18b20_ext.reset();
    ds18b20_ext.select(addr);
    ds18b20_ext.write(0x44, 1); // start conversion, with parasite power on at the end
    unsigned long prMillis = millis();
    byte busStatus = 0;
    do // keep reading the ststus word until conversion is complete
    {
        busStatus = ds18b20_ext.read(); // keep reading until conversion is done
    } while (busStatus != 0xFF && (millis() - prMillis) < 900); // busStatus = 0xFF means conversion done
    //---------------------------
    Serial.print("Conversion time: ");
    Serial.print(millis() - prMillis);
    Serial.println(" ms");

    // delay(250); // 1000mS, maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.
    present = ds18b20_ext.reset();
    ds18b20_ext.select(addr);
    ds18b20_ext.write(0xBE); // Read Scratchpad

    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    for (i = 0; i < 9; i++) { // we need 9 bytes
        data[i] = ds18b20_ext.read();
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    ds18b20_ext.depower();
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - data[6];
        }
    } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00)
            raw = raw & ~7; // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20)
            raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40)
            raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
    }
    tp = (float)raw / 16.0;
    Serial.print("  Temperature = ");
    Serial.println(tp);

    VextOFF();
    return (int16_t)(tp * 100.0);
}
*/

int16_t get_temperature(OneWire& myds, uint8_t mypin)
{
    // https://forum.arduino.cc/t/ds18b20-temperature-sensor-using-onewire-library/699347/10
    float tp = 0;
    byte i;
    byte present = 0;
    byte type_s;
    byte data[9];
    byte addr[8];
    // 9 bits
    // byte dsRes[] = { 0x00, 0x00, 0x1F }; //, 0x1F(0 R1 R0 11111)/0x3F/0x5F/0x7F for 9-10-11-12-bit Resolutio
    // 10 bits
    byte dsRes[] = { 0x00, 0x00, 0x3F }; //, 0x1F(0 R1 R0 11111)/0x3F/0x5F/0x7F for 9-10-11-12-bit Resolutio 0.25 deg
    // 12bits
    // byte dsRes[] = { 0x00, 0x00,  0x7F}; //, 0x1F(0 R1 R0 11111)/0x3F/0x5F/0x7F for 9-10-11-12-bit Resolutio

    // float celsius, fahrenheit;

    VextON();
    delay(150);
    pinMode(mypin, INPUT);
    global_fault = global_fault & 0b11111100;
    myds.begin(mypin);

    // Serial.println(myds.reset());
    if (myds.reset() < 1) {
        if (mypin == ONE_WIRE_INT) {
            // Serial.println("TEMP INT NOT present");
            global_fault = global_fault | 0b00000001;
        } else {
            //  Serial.println("TEMP EXT NOT present");
            global_fault = global_fault | 0b00000010;
        }
        VextOFF();
        return -12700;
    }

    myds.reset_search();
    myds.search(addr);
    myds.reset_search();
    global_fault = global_fault & 0b11111110;

    if (OneWire::crc8(addr, 7) != addr[7]) {
        // Serial.println("CRC is not valid!");
        if (mypin == ONE_WIRE_INT) {
            global_fault = global_fault | 0b00000001;
        } else {
            global_fault = global_fault | 0b00000010;
        }
        VextOFF();
        hx711_data.fault = global_fault;
        return -12700;
    }

    type_s = 0; // DS18B20
    myds.reset();
    myds.select(addr);
    /*
    for (i = 0; i < 8; i++) { // we need 9 bytes
        Serial.print(addr[i], HEX);
        Serial.print(" ");
    }
    Serial.println(" ");
    */
    myds.write(0x4E);
    myds.write_bytes(dsRes, 3, 1); // set resolution bit
    myds.reset();
    myds.select(addr);
    myds.write(0x44, 1); // start conversion, with parasite power on at the end
    unsigned long prMillis = millis();
    byte busStatus = 0;
    do // keep reading the ststus word until conversion is complete
    {
        busStatus = myds.read(); // keep reading until conversion is done
    } while (busStatus != 0xFF && (millis() - prMillis) < 900); // busStatus = 0xFF means conversion done
    //---------------------------
    /*
    Serial.print("Conversion time: ");
    Serial.print(millis() - prMillis);
    Serial.println(" ms");
*/
    // delay(250); // 1000mS, maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.
    present = myds.reset();
    myds.select(addr);
    myds.write(0xBE); // Read Scratchpad
    /*
        Serial.print("  Data = ");
        Serial.print(present, HEX);
        Serial.print(" ");
     */
    for (i = 0; i < 9; i++) { // we need 9 bytes
        data[i] = myds.read();
        // Serial.print(data[i], HEX);
        // Serial.print(" ");
    }
    myds.depower();
    // Serial.print(" CRC=");
    // Serial.print(OneWire::crc8(data, 8), HEX);
    // Serial.println();

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - data[6];
        }
    } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00)
            raw = raw & ~7; // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20)
            raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40)
            raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
    }
    tp = (float)raw / 16.0;
    //  Serial.print("  Temperature = ");
    //  Serial.println(tp);

    VextOFF();
    return (int16_t)(tp * 100.0);
}

// downlink data handle function example
void downLinkDataHandle(McpsIndication_t* mcpsIndication)
{
#ifdef DEBUGPRINT
    Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
    Serial.print("+REV DATA:");

    for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
        Serial.printf("%02X ", mcpsIndication->Buffer[i]);
    }
    Serial.println();
    Serial.flush();
#endif

    action = mcpsIndication->Buffer[0];
    // 0x03=> setZERO
    // 0x30 => sent params
    // no needed: 0x33 => setZERO & sent params
    // EEPROM.write(0, model);
    // EEPROM.commit();

    time_sec_cycle = mcpsIndication->Buffer[1] << 8 | mcpsIndication->Buffer[2];
    if (time_sec_cycle > 29 && time_sec_cycle < 7201) { // min 30 sec,  max 2H
        // EEPROM.writeUShort(1, time_sec_cycle);
        // EEPROM.commit();
    } else {
        time_sec_cycle = 300;
    }

    switch (action) {
    case 0x03: { // zero hx711
        setzero = true;
        send_parameters = true;
        deviceState = DEVICE_STATE_SEND;
        action = 0;
        break;
    }
    case 0x30: { //  sent params
        setzero = false;
        send_parameters = true;
        deviceState = DEVICE_STATE_SEND;
        action = 0;
        break;
    }
    case 0xC0: { //  sent data
        setzero = false;
        send_parameters = false;
        deviceState = DEVICE_STATE_SEND;
        action = 0;
        break;
    }

    default: {
        action = 0;
        break;
    }
    }

    set_color(YELLOW, 150, 250);

#ifdef DEBUGPRINT
    Serial.print("Action: ");
    Serial.println(action);
    Serial.print("\tTx sec: ");
    Serial.println(time_sec_cycle);
    Serial.flush();
    delay(2000);
#endif
}

void myturnOnRGB(uint32_t color, uint32_t time)
{
    uint8_t red, green, blue;
    red = (uint8_t)(color >> 16);
    green = (uint8_t)(color >> 8);
    blue = (uint8_t)color;
    VextON();
    delay(1);

    mypixels.begin(); // INITIALIZE RGB strip object (REQUIRED)
    mypixels.clear(); // Set all pixel colors to 'off'
    mypixels.setPixelColor(0, mypixels.Color(red, green, blue));
    mypixels.show(); // Send the updated pixel colors to the hardware.
    if (time > 0) {
        delay(time);
    }
}

void myturnOffRGB(void)
{
    myturnOnRGB(0, 0);
    VextOFF();
}

// ------------------ init HX711 ------------------------------
void scale_init(void)
{
    byte x = hx711_data.offset_vbat = readBatLevel();
    hx711_data.offset_tempint = get_temperature(ds18b20_int, ONE_WIRE_INT);
    hx711_data.offset_tempext = get_temperature(ds18b20_ext, ONE_WIRE_EXT);
    byte yy = 0;
    while (hx711_data.offset_tempint < -1200 && yy < 3) {
        hx711_data.offset_tempint = get_temperature(ds18b20_int, ONE_WIRE_INT);
        yy++;
        delay(100);
    }
    yy = 0;
    while (hx711_data.offset_tempext < -1200 && yy < 3) {
        hx711_data.offset_tempext = get_temperature(ds18b20_ext, ONE_WIRE_EXT);
        yy++;
        delay(100);
    }
    VextOFF();
    delay(10);
    Vhx711ON();
    delay(200);
    // temp_old = tempext;
    // derivative = 0.00;
    // derivative_old = 0.00;
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN,64);
scale.set_gain(64);
    global_fault = global_fault & 0b11110011;
    if (scale.wait_ready_retry(4, 250)) {
        long mylong = scale.read_average(10);
#ifdef DEBUGPRINT
        Serial.println(mylong);
#endif
    } else {
#ifdef DEBUGPRINT
        Serial.println("NO HX711 1");
#endif
        global_fault = global_fault | 0b00000100;
    }

    if (hx711_data.offset_adc == 0) {
        global_fault = global_fault | 0b00001000;
        if (scale.wait_ready_retry(4, 250)) {
            scale.tare(70);
            hx711_data.offset_adc = scale.get_offset();
#ifdef DEBUGPRINT
            Serial.print("INIT rtc RAM offset: ");
            Serial.println(hx711_data.offset_adc);
#endif
        } else {
#ifdef DEBUGPRINT
            Serial.println("NO HX711 2");
#endif
            global_fault = global_fault | 0b00000100;
        }
    } else {
        scale.set_offset(hx711_data.offset_adc);
#ifdef DEBUGPRINT
        Serial.print("rtc RAM offset ok: ");
        Serial.println(hx711_data.offset_adc);
#endif
    }

    // INIT rtc RAM offset: 322587
    // scale.set_scale(112.867886670);
    // scale.set_scale(67.9375000);
    // scale.set_scale(1.00000000f);

    scale.set_scale(weight_scale); //=(ave 75 measures sans offset)/2000 grammes

    scale.power_down();
    Vhx711OFF();
    hx711_data.fault = global_fault;
    hx711_data.poids_int = 0;
}
/*
int16_t get_weight_g(void)
{
    VextOFF();
    delay(10);
    Vhx711ON();
    global_fault = global_fault & 0b11111101;
    delay(200);
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    units10 = 0.0;
    scale.power_up();
    delay(150);
    if (scale.wait_ready_retry(4, 200)) {
        units10 = scale.get_units(2);
    } else {
        global_fault = global_fault | 0b00000010;
    }

    if (scale.wait_ready_retry(4, 200)) {
        units10 = scale.get_units(2);
        units10 = scale.get_units(25);
    } else {
        global_fault = global_fault | 0b00000010;
    }

    scale.power_down();
    Vhx711OFF();
    return round_float(units10);
} */

uint8_t get_weight_vbat_corrected(void)
{
    VextOFF();
    delay(1);
    Vhx711ON();
    global_fault = global_fault & 0b11110101; // clear hx711 fault
    delay(200);
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN,64);
    //scale.set_gain(64);
    hx711_data.adc = 0;
    scale.power_up();
    delay(150);
    if (scale.wait_ready_retry(4, 200)) {
        units10 = scale.get_units(2);
    } else {
        global_fault = global_fault | 0b00000010;
    }
    if (scale.wait_ready_retry(4, 200)) {
        hx711_data.adc = scale.read_average(2);
        hx711_data.adc = scale.read_average(25);
    } else {
        global_fault = global_fault | 0b00000010; // set hx711 fault
    }

    scale.power_down();

    Vhx711OFF();

    delta_vbat = hx711_data.offset_vbat - vbat_mv;
    poidsf = float(COEFVBAT) * float(delta_vbat);
    poidsf = poidsf + float(hx711_data.adc);
    poidsf = (poidsf - float(hx711_data.offset_adc)) / weight_scale;
    // derivative = float(tempext - temp_old) * TEMPFACTOR + derivative_old;
    //  poidsf = poidsf + derivative;
    poidsf = roundf(poidsf * 10.000);
    hx711_data.poids_int = int32_t(poidsf);
    poidsf = poidsf / 10;
    // hx711_data.poids_int_cor = int32_t(roundf(poidsf * 10.000));
    //  temp_old = tempext;
    //  derivative_old = derivative;
    return global_fault;
}
/*
long hx711_read_ave(void)
{
    VextOFF();
    delay(10);
    Vhx711ON();
    delay(200);
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    hx711_data.adc = 0;
    scale.power_up();
    delay(100);
    if (scale.wait_ready_retry(4, 200)) {
        hx711_data.adc = scale.read_average(2);
        hx711_data.adc = scale.read_average(25);
    }
    scale.power_down();
    Vhx711OFF();
    return hx711_data.adc;
} */

// https://github.com/RobTillaart/FRAM_I2C/blob/master/examples/FRAM_clear/FRAM_clear.ino

bool fram_write(uint16_t adr, int16_t data)
{
    bool vr = false;
    VextON();
    delay(5);
    if (fram.begin(EEPROM_ADDR)) { // you can stick the new i2c addr in here, e.g. begin(0x51);
        // fram.writeObject(0,hx711_data);
        fram.write16(adr, data);
        if (data == fram.read16(adr)) {
            vr = true;
        }
    }
    VextOFF();
    return vr;
}

void fram_test(void)
{
    VextON();
    delay(10);
    if (fram.begin(EEPROM_ADDR)) { // you can stick the new i2c addr in here, e.g. begin(0x51);
        Serial.println("Found I2C EEPROM");
    } else {
        Serial.println("I2C EEPROM not identified ... check your connections?\r\n");
        return;
        // while (1)
        //     delay(10);
    }

    float f = 3.141592;
    uint8_t buffer[4]; // floats are 4 bytes!
    buffer[0] = 1;
    buffer[1] = 2;
    buffer[2] = 3;
    buffer[3] = 4;
    memcpy(buffer, &f, 4);
    Serial.print("Writing float to address 0x00: ");
    for (uint8_t x = 0; x < 4; x++) {
        Serial.print(", ");
        Serial.print(buffer[x], HEX);
    }
    Serial.println();
    fram.write(0x00, buffer, 4);
    delay(10);
    fram.read(0x00, buffer, 4);
    memcpy(&f, buffer, 4);
    Serial.print("Read back float value: ");
    for (uint8_t x = 0; x < 4; x++) {
        Serial.print(", ");
        Serial.print(buffer[x], HEX);
    }
    Serial.println();
    Serial.println(f, 8);
    VextOFF();
}

void fram_dump(void)
{
    VextON();
    delay(10);
    Wire.begin();
    int rv = fram.begin(EEPROM_ADDR);
    if (rv != 0) {
        Serial.print("INIT ERROR: ");
        Serial.println(rv);
    }

    uint32_t sizeInBytes = fram.getSizeBytes();
    Serial.print("FRAM SIZE:\t");
    Serial.println(sizeInBytes);
    Serial.println();
    delay(1);

    for (uint32_t addr = 0; addr < 1024; addr += 16) {
        //  VERTICAL SEPARATOR
        if (addr % 320 == 0)
            Serial.println();
        //  ADDRESS
        Serial.println();
        Serial.print(addr);
        Serial.print("\t");
        //  HEX
        for (uint32_t i = 0; i < 16; i++) {
            uint8_t b = fram.read8(addr + i);
            if (b < 16)
                Serial.print(0);
            Serial.print(b, HEX);
            Serial.print("  ");
        }
        Serial.print("  ");
        //  ASCII
        for (uint32_t i = 0; i < 16; i++) {
            uint8_t b = fram.read8(addr + i);
            if (isprint(b))
                Serial.print((char)b);
            else
                Serial.print(".");
        }
    }
    VextOFF();
}
