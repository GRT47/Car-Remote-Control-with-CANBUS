#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <mcp2515.h>
#include <queue>
#include <vector>

// WiFi 및 MQTT 설정
const char* ssid = "test";        // Wi-Fi 네트워크 이름
const char* password = "test";    // Wi-Fi 암호
const char* mqttServer = "test.coim"; // MQTT 브로커 주소
const int mqttPort = 1883;         // MQTT 브로커 포트
const char* mqttUser = "test";     // MQTT 사용자 이름
const char* mqttPassword = "test"; // MQTT 비밀번호

WiFiClient espClient;
PubSubClient mqttClient(espClient);

MCP2515 mcp2515(D8);

struct can_frame canMsg;
// can_frame에 대한 우선순위 비교 함수 정의
struct CompareCANMessage {
    bool operator()(const can_frame& lhs, const can_frame& rhs) {
        // 예시: 더 낮은 ID를 더 높은 우선순위로 간주
        return lhs.can_id > rhs.can_id;
    }
};

// 우선순위 큐 정의
std::priority_queue<can_frame, std::vector<can_frame>, CompareCANMessage> canPriorityQueue;

void checkCANMessages();
void readVoltageAndPublish();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnect();
void Engine_Status();
void Door_Security_Status();

void setup() {
  Serial.begin(115200);
  SPI.begin();

  // MCP2515 초기화
  mcp2515.reset();
  mcp2515.setBitrate(CAN_100KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  // WiFi 연결
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // MQTT 클라이언트 설정
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);

  // MQTT 브로커에 연결
  reconnect();

  // CAN 메시지 구조체 초기화
  canMsg.can_id = 0x000;
  canMsg.can_dlc = 8;
  memset(canMsg.data, 0, sizeof(canMsg.data));
}

void loop() {
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();

    checkCANMessages(); // 이제 우선순위 큐에 메시지를 추가합니다.
  
    // 우선순위 큐를 처리합니다.
    while (!canPriorityQueue.empty()) {
        canMsg = canPriorityQueue.top();
        canPriorityQueue.pop();

        // 메시지 ID에 따라 CAN 메시지 처리
        switch (canMsg.can_id) {
            case 0x000:
                Engine_Status();
                break;
            case 0x000:
                Door_Security_Status();
                break;
            // 기타 메시지 ID에 대한 케이스를 추가할 수 있습니다.
        }
    }

    readVoltageAndPublish();
}

void checkCANMessages() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // 우선순위 큐에 메시지를 추가
    canPriorityQueue.push(canMsg);
    switch (canMsg.can_id) {
      case 0x000:
        // 엔진 시작/정지 확인
        Engine_Status();
        break;
      case 0x000:
        // 문 잠금/잠금 해제 확인
        Door_Security_Status();
        break;
    }
  }
}

void Engine_Status() {
  uint8_t expectedDataStart[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t expectedDataStop[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  if (memcmp(canMsg.data, expectedDataStart, sizeof(expectedDataStart)) == 0) {
    mqttClient.publish("VEHICLE/ENGSTART", "Engine started");
  }
  else if (memcmp(canMsg.data, expectedDataStop, sizeof(expectedDataStop)) == 0) {
    mqttClient.publish("VEHICLE/ENGSTOP", "Engine stopped");
  }
}
void Door_Security_Status() {
  uint8_t expectedDatalocked1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t expectedDatalocked2[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //트렁크OPEN
  uint8_t expectedDatalocked3[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t expectedDatalocked4[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //트렁크OPEN
  uint8_t expectedDataunlocked1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t expectedDataunlocked2[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //트렁크OPEN
  uint8_t expectedDataunlocked3[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  if (memcmp(canMsg.data, expectedDatalocked1, sizeof(expectedDatalocked1)) == 0) {
    mqttClient.publish("VEHICLE/DOORLOCK", "trunkclose");
  }
  else if (memcmp(canMsg.data, expectedDatalocked2, sizeof(expectedDatalocked2)) == 0) {
    mqttClient.publish("VEHICLE/DOORLOCK", "trunkopen");
  }
  else if (memcmp(canMsg.data, expectedDatalocked3, sizeof(expectedDatalocked3)) == 0) {
    mqttClient.publish("VEHICLE/DOORLOCK", "trunkclose");
  }
  else if (memcmp(canMsg.data, expectedDatalocked4, sizeof(expectedDatalocked4)) == 0) {
    mqttClient.publish("VEHICLE/DOORLOCK", "trunkopen");
  }
  else if (memcmp(canMsg.data, expectedDataunlocked1, sizeof(expectedDataunlocked1)) == 0) {
    mqttClient.publish("VEHICLE/DOORUNLOCK", "trunkclose");
  }
  else if (memcmp(canMsg.data, expectedDataunlocked2, sizeof(expectedDataunlocked2)) == 0) {
    mqttClient.publish("VEHICLE/DOORUNLOCK", "trunkopen");
  }
  else if (memcmp(canMsg.data, expectedDataunlocked3, sizeof(expectedDataunlocked3)) == 0) {
    mqttClient.publish("VEHICLE/DOORUNLOCK", "trunkclose");
  }
}

void readVoltageAndPublish() {
  static unsigned long lastPublishTime = 0;
  if (millis() - lastPublishTime >= 1000) {
    lastPublishTime = millis();
    // 전압 읽기 및 계산
    float V_ADC = analogRead(A0);
    float V = V_ADC * 5.0 / 1024.0 / 0.31;
    // 전압 값을 문자열로 변환
    char voltageStr[8];
    dtostrf(V, 1, 2, voltageStr);
    mqttClient.publish("VEHICLE/VOLTAGE", voltageStr);
  }
}

// CAN데이터 전송
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, "VEHICLE/LOCK") == 0) {
    uint8_t lockData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canMsg.can_id = 0x000;
    memcpy(canMsg.data, lockData, sizeof(lockData));
    mcp2515.sendMessage(&canMsg);
  } else if (strcmp(topic, "VEHICLE/UNLOCK") == 0) {
    uint8_t unlockData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canMsg.can_id = 0x000;
    memcpy(canMsg.data, unlockData, sizeof(unlockData));
    mcp2515.sendMessage(&canMsg);
  } else if (strcmp(topic, "VEHICLE/TRUNK") == 0) {
    uint8_t trunkData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canMsg.can_id = 0x000;
    memcpy(canMsg.data, trunkData, sizeof(trunkData));
    mcp2515.sendMessage(&canMsg);
  } else if (strcmp(topic, "VEHICLE/HORN") == 0) {
    uint8_t hornData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canMsg.can_id = 0x000;
    memcpy(canMsg.data, hornData, sizeof(hornData));
    mcp2515.sendMessage(&canMsg);
  } else if (strcmp(topic, "VEHICLE/EML") == 0) { // 비상등
    uint8_t emlData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canMsg.can_id = 0x000;
    memcpy(canMsg.data, emlData, sizeof(emlData));
    mcp2515.sendMessage(&canMsg);
  } else if (strcmp(topic, "VEHICLE/START") == 0) {
    uint8_t startData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canMsg.can_id = 0x000;
    memcpy(canMsg.data, startData, sizeof(startData));
    for (int i = 0; i < 3; i++) {  //메시지 3번 전송
      mcp2515.sendMessage(&canMsg);
      delay(500);
    }
  } else if (strcmp(topic, "VEHICLE/STOP") == 0) {
    uint8_t stopData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    canMsg.can_id = 0x000;
    memcpy(canMsg.data, stopData, sizeof(stopData));
    for (int i = 0; i < 3; i++) {  //메시지 3번 전송
      mcp2515.sendMessage(&canMsg);
      delay(500);
    }
  } 
}

void reconnect() {
  // MQTT 브로커에 재연결하는 로직
  while (!mqttClient.connected()) {
    if (mqttClient.connect("ESP8266Client", mqttUser, mqttPassword)) {
      // 주제 구독
      mqttClient.subscribe("VEHICLE/LOCK");
      mqttClient.subscribe("VEHICLE/UNLOCK");
      mqttClient.subscribe("VEHICLE/TRUNK");
      mqttClient.subscribe("VEHICLE/HORN");
      mqttClient.subscribe("VEHICLE/START");
      mqttClient.subscribe("VEHICLE/STOP");
      mqttClient.subscribe("VEHICLE/EML");
    } else {
      // 연결 실패 시 재시도 전 5초 대기
      delay(5000);
    }
  }
}
