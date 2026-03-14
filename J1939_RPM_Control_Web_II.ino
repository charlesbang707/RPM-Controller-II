#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include "driver/twai.h"

#define CAN_TX_PIN GPIO_NUM_22
#define CAN_RX_PIN GPIO_NUM_21

Preferences prefs;
byte mySA = 0x0B; 
int currentBaud = 500;
bool addressConfirmed = false;
unsigned long claimTimer = 0;
unsigned char myNAME[8] = {0x01, 0x00, 0x00, 0xE8, 0x00, 0x21, 0x00, 0x80};

BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
volatile int targetRPM = 0;   
volatile int currentRPM = 0;

enum ControlState { IDLE, RUNNING, STOPPING };
volatile ControlState currentState = IDLE;

void sendAddressClaim() {
    twai_message_t msg;
    msg.identifier = 0x18EEFF00 | mySA;
    msg.extd = 1; msg.data_length_code = 8;
    memcpy(msg.data, myNAME, 8);
    twai_transmit(&msg, pdMS_TO_TICKS(10));
    claimTimer = millis();
}

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() >= 2) {
            uint8_t type = value[0];
            uint8_t val  = value[1];

            if (type == 0xA1) { // Baud
                int b = (val == 250) ? 250 : 500;
                if(currentBaud != b) { prefs.putInt("baud", b); ESP.restart(); }
            } 
            else if (type == 0xA2) { // SA
                mySA = val;
                prefs.putUChar("mySA", mySA);
                addressConfirmed = false;
                sendAddressClaim();
            }
            else if (type == 0xA3) { // Start/Stop
                if (val == 1) { currentState = RUNNING; targetRPM = 800; }
                else { currentState = STOPPING; }
            }
            else if (type == 0xA4) { // RPM Up/Down
                if (val == 1) targetRPM += 100;
                else targetRPM -= 100;
                targetRPM = constrain(targetRPM, 600, 2500);
            }
        }
    }
};

void CAN_Task(void *pvParameters) {
    twai_message_t rx_msg;
    while (1) {
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(5)) == ESP_OK) {
            if ((rx_msg.identifier & 0x00FFFF00) == 0x00F00400) {
                currentRPM = (int)((rx_msg.data[4] | (rx_msg.data[5] << 8)) * 0.125);
            }
        }
        if (!addressConfirmed && (millis() - claimTimer > 250)) addressConfirmed = true;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup() {
    Serial.begin(115200);
    prefs.begin("j1939", false);
    mySA = prefs.getUChar("mySA", 0x0B);
    currentBaud = prefs.getInt("baud", 500);

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = (currentBaud == 250) ? TWAI_TIMING_CONFIG_250KBITS() : TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();

    BLEDevice::init("ESP32_RPM_CTRL");
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
    pCharacteristic = pService->createCharacteristic("6e400002-b5a3-f393-e0a9-e50e24dcca9e", BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    pServer->getAdvertising()->start();

    xTaskCreatePinnedToCore(CAN_Task, "CAN", 4096, NULL, 1, NULL, 0);
    sendAddressClaim();
}

void loop() {
    static unsigned long lastTX = 0;
    if (millis() - lastTX >= 10 && currentState == RUNNING && addressConfirmed) {
        uint16_t r = (uint16_t)(targetRPM / 0.125);
        twai_message_t tx;
        tx.identifier = 0x0C000000 | (0x00 << 8) | mySA;
        tx.extd = 1; tx.data_length_code = 8;
        tx.data[0] = 0x01; tx.data[1] = (uint8_t)(r & 0xFF); tx.data[2] = (uint8_t)(r >> 8);
        for(int i=3; i<8; i++) tx.data[i] = 0xFF;
        twai_transmit(&tx, pdMS_TO_TICKS(5));
        lastTX = millis();
    }
    
    static unsigned long lastBLE = 0;
    if (millis() - lastBLE >= 100) {
        uint8_t d[2] = { (uint8_t)(currentRPM & 0xFF), (uint8_t)(currentRPM >> 8) };
        pCharacteristic->setValue(d, 2);
        pCharacteristic->notify();
        lastBLE = millis();
    }
}
