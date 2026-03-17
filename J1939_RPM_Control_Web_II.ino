#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "driver/twai.h"

// 핀 정의
#define CAN_TX_PIN GPIO_NUM_22
#define CAN_RX_PIN GPIO_NUM_21
const int RESUME_PIN = 32, UP_PIN = 33, DOWN_PIN = 25, STOP_PIN = 26;

// 전역 변수
byte mySA = 0xE5; 
volatile int targetRPM = 0, currentRPM = 0;
enum ControlState { IDLE, RUNNING, STOPPING };
volatile ControlState currentState = IDLE;
bool is500k = false;
bool deviceConnected = false;
BLECharacteristic *pCharacteristic;

// CAN 초기화 (3항 연산자 에러 수정 버전)
void initCAN(bool use500k) {
    twai_stop();
    twai_driver_uninstall();
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config;
    if (use500k) { t_config = TWAI_TIMING_CONFIG_500KBITS(); }
    else { t_config = TWAI_TIMING_CONFIG_250KBITS(); }
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
    Serial.printf("CAN Initialized: %s\n", use500k ? "500K" : "250K");
}

// BLE 콜백 설정
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; Serial.println("BLE Connected"); };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; Serial.println("BLE Disconnected"); pServer->getAdvertising()->start(); }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String rx = pChar->getValue();
        if (rx.length() < 2) return;
        uint16_t val = (uint8_t)rx[0] | ((uint8_t)rx[1] << 8);

        if (val == 1000) { currentState = RUNNING; targetRPM = 1000; Serial.println("Command: START/RESUME"); }
        else if (val == 600) { currentState = STOPPING; Serial.println("Command: STOP"); }
        else if (val == 1) { targetRPM = min(targetRPM + 25, 2500); Serial.printf("Command: UP (%d)\n", targetRPM); }
        else if (val == 2) { targetRPM = max(targetRPM - 25, 1000); Serial.printf("Command: DOWN (%d)\n", targetRPM); }
        else if (val == 0x5000) { is500k = true; initCAN(true); }
        else if (val == 0x2500) { is500k = false; initCAN(false); }
        else if ((val & 0xFF00) == 0xA500) { mySA = val & 0xFF; Serial.printf("SA Changed: 0x%02X\n", mySA); }
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("System Starting...");

    pinMode(RESUME_PIN, INPUT_PULLUP); pinMode(UP_PIN, INPUT_PULLUP);
    pinMode(DOWN_PIN, INPUT_PULLUP); pinMode(STOP_PIN, INPUT_PULLUP);
    
    initCAN(is500k);

    BLEDevice::init("Truck_RPM_Control");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
    pCharacteristic = pService->createCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8", 
                BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    pServer->getAdvertising()->start();
    Serial.println("BLE Advertising Started. Ready!");
}

void loop() {
    unsigned long now = millis();
    // 하드웨어 버튼 감지
    if (digitalRead(RESUME_PIN) == LOW && currentState == IDLE) { currentState = RUNNING; targetRPM = 1000; delay(200); }
    if (digitalRead(STOP_PIN) == LOW && currentState == RUNNING) { currentState = STOPPING; delay(200); }
    if (currentState == RUNNING) {
        if (digitalRead(UP_PIN) == LOW) { targetRPM = min(targetRPM + 25, 2500); delay(150); }
        if (digitalRead(DOWN_PIN) == LOW) { targetRPM = max(targetRPM - 25, 1000); delay(150); }
    }

    static unsigned long lastUpdate = 0;
    if (now - lastUpdate >= 100) {
        if (currentState == STOPPING) {
            targetRPM -= 25;
            if (targetRPM <= 700) { currentState = IDLE; targetRPM = 0; }
        }
        
        // 데이터 전송 (웹으로 보고)
        if (deviceConnected) {
            uint8_t tx[5] = { (uint8_t)(currentRPM & 0xFF), (uint8_t)(currentRPM >> 8), 
                             (uint8_t)(targetRPM & 0xFF), (uint8_t)(targetRPM >> 8), (uint8_t)(is500k ? 1 : 0) };
            pCharacteristic->setValue(tx, 5);
            pCharacteristic->notify();
        }
        lastUpdate = now;
    }
}
