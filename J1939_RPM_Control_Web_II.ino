#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include "driver/twai.h"

// 핀 정의
#define CAN_TX_PIN GPIO_NUM_22
#define CAN_RX_PIN GPIO_NUM_21

// 전역 변수
Preferences prefs;
byte mySA = 0xE5; 
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

// J1939 주소 주장 함수
void sendAddressClaim() {
    twai_message_t msg;
    msg.identifier = 0x18EEFF00 | mySA;
    msg.extd = 1; msg.data_length_code = 8;
    memcpy(msg.data, myNAME, 8);
    twai_transmit(&msg, pdMS_TO_TICKS(10));
    claimTimer = millis();
}

// BLE 콜백 클래스
class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() >= 2) {
            uint8_t type = value[0];
            uint8_t val  = value[1];

            if (type == 0xA1) { // Baud Rate 변경
                int newBaud = (val == 250) ? 250 : 500;
                if (currentBaud != newBaud) {
                    prefs.putInt("baud", newBaud);
                    Serial.println("Baud changed. Restarting...");
                    delay(500);
                    ESP.restart();
                }
            } 
            else if (type == 0xA2) { // Source Address 변경
                if (mySA != val) {
                    mySA = val;
                    prefs.putUChar("mySA", mySA);
                    addressConfirmed = false;
                    sendAddressClaim();
                    Serial.printf("SA changed to: 0x%02X\n", mySA);
                }
            }
            else if (type == 0xA3) { // Start/Stop 제어
                if (val == 1) { currentState = RUNNING; targetRPM = 1200; }
                else { currentState = STOPPING; }
            }
        }
    }
};

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; }
    void onDisconnect(BLEServer* pServer) { 
        deviceConnected = false;
        pServer->getAdvertising()->start();
    }
};

void CAN_Task(void *pvParameters) {
    twai_message_t rx_msg;
    while (1) {
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            // 주소 주장 요청(PGN 59904) 대응
            if ((rx_msg.identifier & 0xFFFF00) == 0x18EAFF00) {
                if (rx_msg.data[0] == 0x00 && rx_msg.data[1] == 0xEE && rx_msg.data[2] == 0x00) {
                    sendAddressClaim();
                }
            }
            // 엔진 RPM 수신 (PGN 61444 - EEC1)
            if ((rx_msg.identifier & 0x00FFFF00) == 0x00F00400) {
                currentRPM = (int)((rx_msg.data[4] | (rx_msg.data[5] << 8)) * 0.125);
            }
        }
        
        // 주소 확정 로직 (주장 후 250ms 동안 충돌 없으면 확정)
        if (!addressConfirmed && (millis() - claimTimer > 250)) {
            addressConfirmed = true;
            Serial.println("Address Claim Confirmed.");
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup() {
    Serial.begin(115200);
    prefs.begin("j1939", false);
    mySA = prefs.getUChar("mySA", 0x0B); // 기본값 11 (Mack/Volvo)
    currentBaud = prefs.getInt("baud", 500);

    // CAN 초기화
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = (currentBaud == 250) ? TWAI_TIMING_CONFIG_250KBITS() : TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();

    // BLE 초기화
    BLEDevice::init("Truck_RPM_Control");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    BLEService *pService = pServer->createService("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
    pCharacteristic = pService->createCharacteristic("6e400002-b5a3-f393-e0a9-e50e24dcca9e",
                        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new MyCallbacks());
    pService->start();
    pServer->getAdvertising()->start();

    xTaskCreatePinnedToCore(CAN_Task, "CAN_Task", 4096, NULL, 1, NULL, 0);
    sendAddressClaim();
}

void loop() {
    unsigned long now = millis();
    static unsigned long last100 = 0;
    if (now - last100 >= 100) {
        if (currentState == STOPPING) {
            targetRPM -= 50;
            if (targetRPM <= 600) { currentState = IDLE; targetRPM = 0; }
        }
        if (deviceConnected) {
            uint8_t tx[4] = { (uint8_t)(currentRPM & 0xFF), (uint8_t)(currentRPM >> 8), 0, 0 };
            pCharacteristic->setValue(tx, 4);
            pCharacteristic->notify();
        }
        last100 = now;
    }

    static unsigned long last10 = 0;
    if (now - last10 >= 10 && currentState == RUNNING && addressConfirmed) {
        // TSC1 송신 (PGN 0)
        uint16_t val = (uint16_t)(targetRPM / 0.125);
        twai_message_t tx_msg;
        tx_msg.identifier = 0x0C000000 | (0x00 << 8) | mySA;
        tx_msg.extd = 1; tx_msg.data_length_code = 8;
        tx_msg.data[0] = 0x01; // Speed Control Mode
        tx_msg.data[1] = (uint8_t)(val & 0xFF);
        tx_msg.data[2] = (uint8_t)(val >> 8);
        tx_msg.data[3] = 0xFF;
        tx_msg.data[4] = 0xFF; tx_msg.data[5] = 0xFF; tx_msg.data[6] = 0xFF; tx_msg.data[7] = 0xFF;
        twai_transmit(&tx_msg, pdMS_TO_TICKS(5));
        last10 = now;
    }
}
