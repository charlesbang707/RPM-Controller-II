#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include "driver/twai.h"

// --- 핀 설정 ---
#define CAN_TX_PIN   GPIO_NUM_22
#define CAN_RX_PIN   GPIO_NUM_21
const int STATUS_LED = 27; // 통신 상태 LED
const int RESUME_PIN = 32; // START 버튼
const int STOP_PIN   = 26; // STOP 버튼
const int UP_PIN     = 33; // RPM UP 버튼
const int DOWN_PIN   = 25; // RPM DOWN 버튼

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

unsigned long lastBtnTime = 0;
const int debounceDelay = 250;

void sendAddressClaim() {
    twai_message_t msg;
    msg.identifier = 0x18EEFF00 | mySA;
    msg.extd = 1; msg.data_length_code = 8;
    memcpy(msg.data, myNAME, 8);
    twai_transmit(&msg, pdMS_TO_TICKS(10));
    claimTimer = millis();
    digitalWrite(STATUS_LED, HIGH); // 송신 시 LED 켬
}

// 명령 처리 함수
void processCommand(uint8_t type, uint8_t val) {
    if (type == 0xA1) { // Baud Rate
        int b = (val == 250) ? 250 : 500;
        if(currentBaud != b) { prefs.putInt("baud", b); ESP.restart(); }
    } 
    else if (type == 0xA2) { // SA 설정
        mySA = val;
        prefs.putUChar("mySA", mySA);
        addressConfirmed = false;
        sendAddressClaim();
    }
    else if (type == 0xA3) { // START/STOP
        if (val == 1) { currentState = RUNNING; if(targetRPM < 800) targetRPM = 800; }
        else { currentState = STOPPING; }
    }
    else if (type == 0xA4) { // RPM UP/DOWN
        if (val == 1) targetRPM += 100;
        else targetRPM -= 100;
        targetRPM = constrain(targetRPM, 600, 2500);
    }
}

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() >= 2) {
            processCommand(value[0], value[1]);
        }
    }
    void onConnect(BLEServer* pServer) { deviceConnected = true; }
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; BLEDevice::startAdvertising(); }
};

void CAN_Task(void *pvParameters) {
    twai_message_t rx_msg;
    while (1) {
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(5)) == ESP_OK) {
            digitalWrite(STATUS_LED, HIGH); // 메시지 수신 시 LED 켬
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
    
    // 핀 모드 설정
    pinMode(STATUS_LED, OUTPUT);
    pinMode(RESUME_PIN, INPUT_PULLUP);
    pinMode(STOP_PIN,   INPUT_PULLUP);
    pinMode(UP_PIN,     INPUT_PULLUP);
    pinMode(DOWN_PIN,   INPUT_PULLUP);

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
    pCharacteristic = pService->createCharacteristic("6e400002-b5a3-f393-e0a9-e50e24dcca9e", 
                        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    pServer->getAdvertising()->start();

    xTaskCreatePinnedToCore(CAN_Task, "CAN", 4096, NULL, 1, NULL, 0);
    sendAddressClaim();
}

void loop() {
    unsigned long now = millis();

    // 물리 버튼 스캔
    if (now - lastBtnTime > debounceDelay) {
        if (digitalRead(RESUME_PIN) == LOW) { processCommand(0xA3, 1); lastBtnTime = now; }
        else if (digitalRead(STOP_PIN) == LOW) { processCommand(0xA3, 0); lastBtnTime = now; }
        else if (digitalRead(UP_PIN) == LOW) { processCommand(0xA4, 1); lastBtnTime = now; }
        else if (digitalRead(DOWN_PIN) == LOW) { processCommand(0xA4, 0); lastBtnTime = now; }
    }

    // CAN 송신 및 LED 제어 (10ms 간격)
    static unsigned long lastTX = 0;
    if (now - lastTX >= 10) {
        if (currentState == RUNNING && addressConfirmed) {
            digitalWrite(STATUS_LED, HIGH); // 송신 시 LED 켬
            uint16_t r = (uint16_t)(targetRPM / 0.125);
            twai_message_t tx;
            tx.identifier = 0x0C000000 | (0x00 << 8) | mySA;
            tx.extd = 1; tx.data_length_code = 8;
            tx.data[0] = 0x01;
            tx.data[1] = (uint8_t)(r & 0xFF);
            tx.data[2] = (uint8_t)(r >> 8);
            for(int i=3; i<8; i++) tx.data[i] = 0xFF;
            twai_transmit(&tx, pdMS_TO_TICKS(5));
        } 
        else if (currentState == STOPPING) {
            targetRPM -= 10;
            if (targetRPM <= 600) { currentState = IDLE; targetRPM = 0; }
        }
        lastTX = now;
    }
    
    // BLE 데이터 전송 및 LED 끔 (100ms 간격)
    static unsigned long lastBLE = 0;
    if (now - lastBLE >= 100) {
        if (pCharacteristic) {
            uint8_t d[4] = { 
                (uint8_t)(currentRPM & 0xFF), (uint8_t)(currentRPM >> 8),
                (uint8_t)(targetRPM & 0xFF), (uint8_t)(targetRPM >> 8) 
            };
            pCharacteristic->setValue(d, 4);
            pCharacteristic->notify();
        }
        digitalWrite(STATUS_LED, LOW); // 깜빡임 효과를 위해 주기적으로 끔
        lastBLE = now;
    }
}
