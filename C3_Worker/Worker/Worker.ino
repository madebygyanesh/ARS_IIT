/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  WORKER_ARS_IIT.ino  —  ARS IIT v11.2  (WORKER / SENSOR BOARD)
 *  Board  : ESP32-C3 Mini
 *  Partner: ESP32 Display at MAC  70:4B:CA:4A:3D:54
 *
 *  HARDWARE:
 *   MPU-6050/6500  SDA → GPIO8   SCL → GPIO9   VCC → 3.3V   GND → GND
 *   RELAY 1        → GPIO2
 *   RELAY 2        → GPIO3
 *   RELAY 3        → GPIO4
 *   RELAY 4        → GPIO5
 *
 *  COMMUNICATION FIXES APPLIED:
 *   1. Correct peer MAC = Display STA MAC  {0x70,0x4B,0xCA,0x4A,0x3D,0x54}
 *   2. Channel forced to 1 BEFORE esp_now_init()
 *   3. IDF v4 / v5 send/recv callback wrappers
 *   4. DataPacket struct identical (packed) to display side
 *   5. ACK never sent in response to another ACK
 *   6. MPU data packed as:
 *        val1=AccX(g)  val2=AccY(g)  val3=AccZ(g)
 *        val4=GyrX(°/s) val5=GyrY(°/s) val6=GyrZ(°/s)
 *        val7=chipTemp(°C)
 *   7. I2C timeout + retry + bus-recovery for reliable MPU reads
 *   8. Relay state pushed to display on every relay change
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ─── Pin definitions ──────────────────────────────────────────────────────────
#define I2C_SDA_PIN   8
#define I2C_SCL_PIN   9

#define RELAY1_PIN    2
#define RELAY2_PIN    3
#define RELAY3_PIN    4
#define RELAY4_PIN    5
static const uint8_t relayPin[4] = {RELAY1_PIN, RELAY2_PIN, RELAY3_PIN, RELAY4_PIN};

// ─── MPU-6050 registers ───────────────────────────────────────────────────────
#define MPU6050_ADDR      0x68
#define REG_PWR_MGMT_1    0x6B
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_SMPRT_DIV     0x19
#define REG_USER_CTRL     0x6A
#define REG_WHO_AM_I      0x75
#define REG_ACCEL_XOUT_H  0x3B
#define REG_GYRO_XOUT_H   0x43
#define REG_TEMP_OUT_H    0x41

// WHO_AM_I accepted values
#define WHO_AM_I_MPU6050        0x68
#define WHO_AM_I_MPU9250        0x71
#define WHO_AM_I_MPU9250_CLONE1 0x73
#define WHO_AM_I_MPU9250_CLONE2 0x98
#define WHO_AM_I_MPU9250_CLONE3 0x70

// ─── I2C reliability settings ─────────────────────────────────────────────────
#define I2C_TIMEOUT_MS          50
#define MPU_MAX_RETRIES         3
#define MPU_RETRY_DELAY_MS      5
#define MAX_CONSECUTIVE_FAILURES 10

// ─── Accel/Gyro sensitivity (set in initializeMPU) ───────────────────────────
// Accel ±4g  → 8192 LSB/g
// Gyro ±500°/s → 65.5 LSB/(°/s)
#define ACCEL_SENSITIVITY  8192.0f
#define GYRO_SENSITIVITY   65.5f

// ═════════════════════════════════════════════════════════════════════════════
//  ESP-NOW PACKET  —  must be identical (same packing) to display
// ═════════════════════════════════════════════════════════════════════════════
#define MSG_TYPE_CHAT         1
#define MSG_TYPE_COMMAND      2
#define MSG_TYPE_ACK          3
#define MSG_TYPE_MPU_DATA     4
#define MSG_TYPE_RELAY_STATUS 5

#define CMD_RELAY1_ON      10
#define CMD_RELAY1_OFF     11
#define CMD_RELAY2_ON      12
#define CMD_RELAY2_OFF     13
#define CMD_RELAY3_ON      14
#define CMD_RELAY3_OFF     15
#define CMD_RELAY4_ON      16
#define CMD_RELAY4_OFF     17
#define CMD_RELAY_STATUS   22
#define CMD_SEQ_START      30
#define CMD_SEQ_STOP       31
#define CMD_MPU_START      40
#define CMD_MPU_STOP       41
#define CMD_MPU_ONCE       42
#define CMD_MPU_CALIBRATE  43
#define CMD_PING           50

typedef struct __attribute__((packed)) {
  uint8_t  msgType;
  char     senderName[16];
  uint32_t messageId;
  int32_t  commandId;
  float    val1, val2, val3, val4, val5, val6, val7;
  char     text[128];
  uint32_t timestamp;
} DataPacket;

static_assert(sizeof(DataPacket) == (1 + 16 + 4 + 4 + 7*4 + 128 + 4),
              "DataPacket size mismatch");

// ─── Display board MAC (STA MAC printed on display serial at boot) ────────────
uint8_t displayMAC[6] = {0x70, 0x4B, 0xCA, 0x4A, 0x3D, 0x54};

bool      espnowReady = false;
uint32_t  msgCounter  = 0;

DataPacket sendPkt;
DataPacket recvPkt;
volatile bool newCmdRx = false;

// ─── Relay + MPU state ────────────────────────────────────────────────────────
bool relayState[4]       = {false, false, false, false};
bool mpuStreamActive     = false;   // CMD_MPU_START enables continuous stream
bool mpuOnceRequested    = false;   // CMD_MPU_ONCE fires single sample
bool mpuCalRequested     = false;

uint8_t  mpuAddress      = MPU6050_ADDR;
bool     mpuHealthy      = false;
int      consFailures    = 0;
unsigned long lastSuccessMs = 0;

unsigned long lastStreamMs = 0;
#define STREAM_INTERVAL_MS  50     // 20 Hz when streaming

// ═════════════════════════════════════════════════════════════════════════════
//  I2C / MPU LOW-LEVEL
// ═════════════════════════════════════════════════════════════════════════════

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
  delay(5);
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)mpuAddress, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

uint8_t readRegisterFrom(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)address, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void recoverI2CBus() {
  Serial.println("[I2C] Bus recovery...");
  Wire.end();
  pinMode(I2C_SDA_PIN, OUTPUT);
  pinMode(I2C_SCL_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, HIGH);
  for(int i=0;i<9;i++){
    digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, LOW);  delayMicroseconds(5);
  }
  // STOP condition
  digitalWrite(I2C_SDA_PIN, LOW);  delayMicroseconds(5);
  digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(5);
  digitalWrite(I2C_SDA_PIN, HIGH); delayMicroseconds(5);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  delay(10);
  Serial.println("[I2C] Recovery done");
}

bool isKnownWhoAmI(uint8_t id){
  return (id == WHO_AM_I_MPU6050   ||
          id == WHO_AM_I_MPU9250   ||
          id == WHO_AM_I_MPU9250_CLONE1 ||
          id == WHO_AM_I_MPU9250_CLONE2 ||
          id == WHO_AM_I_MPU9250_CLONE3);
}

bool detectMPU() {
  Serial.println("[MPU] Scanning...");
  for(uint8_t addr : {0x68, 0x69}){
    Wire.beginTransmission(addr);
    if(Wire.endTransmission() == 0){
      uint8_t who = readRegisterFrom(addr, REG_WHO_AM_I);
      Serial.printf("[MPU] Found 0x%02X → WHO_AM_I=0x%02X\n", addr, who);
      if(isKnownWhoAmI(who)){
        mpuAddress = addr;
        Serial.printf("[MPU] Confirmed at 0x%02X\n", addr);
        return true;
      }
    }
  }
  Serial.println("[MPU] Not found! Check wiring SDA=GPIO8 SCL=GPIO9");
  return false;
}

bool initializeMPU() {
  Serial.println("[MPU] Initializing...");
  writeRegister(REG_PWR_MGMT_1, 0x80);  // Reset
  delay(200);
  writeRegister(REG_PWR_MGMT_1, 0x01);  // Wake, PLL with X-gyro
  delay(100);
  writeRegister(REG_SMPRT_DIV,  0x04);  // 200 Hz sample rate
  writeRegister(REG_CONFIG,     0x03);  // DLPF 44 Hz
  writeRegister(REG_GYRO_CONFIG,  0x08);  // ±500 °/s
  writeRegister(REG_ACCEL_CONFIG, 0x08);  // ±4 g
  writeRegister(REG_USER_CTRL,  0x00);
  delay(100);
  Serial.println("[MPU] OK");
  return true;
}

// ── Read 6-byte burst with retry ──────────────────────────────────────────────
bool readRawAccel(int16_t *ax, int16_t *ay, int16_t *az) {
  for(int attempt=0; attempt<MPU_MAX_RETRIES; attempt++){
    Wire.beginTransmission(mpuAddress);
    Wire.write(REG_ACCEL_XOUT_H);
    uint8_t err = Wire.endTransmission(false);
    if(err == 0){
      Wire.requestFrom((uint8_t)mpuAddress, (uint8_t)6);
      if(Wire.available() >= 6){
        *ax = (Wire.read()<<8)|Wire.read();
        *ay = (Wire.read()<<8)|Wire.read();
        *az = (Wire.read()<<8)|Wire.read();
        consFailures=0; lastSuccessMs=millis(); mpuHealthy=true;
        return true;
      }
    }
    if(attempt < MPU_MAX_RETRIES-1){
      delay(MPU_RETRY_DELAY_MS);
      if(err != 0) recoverI2CBus();
    }
  }
  consFailures++; mpuHealthy=false;
  Serial.println("[MPU] Accel read failed");
  return false;
}

bool readRawGyro(int16_t *gx, int16_t *gy, int16_t *gz) {
  for(int attempt=0; attempt<MPU_MAX_RETRIES; attempt++){
    Wire.beginTransmission(mpuAddress);
    Wire.write(REG_GYRO_XOUT_H);
    uint8_t err = Wire.endTransmission(false);
    if(err == 0){
      Wire.requestFrom((uint8_t)mpuAddress, (uint8_t)6);
      if(Wire.available() >= 6){
        *gx = (Wire.read()<<8)|Wire.read();
        *gy = (Wire.read()<<8)|Wire.read();
        *gz = (Wire.read()<<8)|Wire.read();
        consFailures=0; lastSuccessMs=millis(); mpuHealthy=true;
        return true;
      }
    }
    if(attempt < MPU_MAX_RETRIES-1){
      delay(MPU_RETRY_DELAY_MS);
      if(err != 0) recoverI2CBus();
    }
  }
  consFailures++; mpuHealthy=false;
  Serial.println("[MPU] Gyro read failed");
  return false;
}

// Read internal temperature register (16-bit)
// Formula: Temp°C = RawValue / 340.0 + 36.53  (MPU-6050 datasheet)
float readChipTemp() {
  Wire.beginTransmission(mpuAddress);
  Wire.write(REG_TEMP_OUT_H);
  if(Wire.endTransmission(false) != 0) return 25.0f;
  Wire.requestFrom((uint8_t)mpuAddress, (uint8_t)2);
  if(Wire.available() < 2) return 25.0f;
  int16_t raw = (Wire.read()<<8) | Wire.read();
  return (float)raw / 340.0f + 36.53f;
}

void checkMPUHealth() {
  if(consFailures >= MAX_CONSECUTIVE_FAILURES){
    Serial.println("[MPU] Too many failures — recovering");
    recoverI2CBus();
    initializeMPU();
    consFailures=0;
  }
  if(mpuHealthy && (millis()-lastSuccessMs) > 5000){
    Serial.println("[MPU] Timeout — recovering");
    mpuHealthy=false;
    recoverI2CBus();
    initializeMPU();
    lastSuccessMs=millis();
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  RELAY HELPERS
// ═════════════════════════════════════════════════════════════════════════════

void setRelay(int idx, bool on){
  if(idx<0||idx>3) return;
  relayState[idx] = on;
  // Active-LOW relays (common for relay modules); change to HIGH if yours differ
  digitalWrite(relayPin[idx], on ? LOW : HIGH);
  Serial.printf("[RELAY] %d = %s\n", idx+1, on?"ON":"OFF");
}

// Build and send relay status string: "R1:ON R2:OFF R3:ON R4:OFF"
void sendRelayStatus(){
  if(!espnowReady) return;
  memset(&sendPkt, 0, sizeof(sendPkt));
  sendPkt.msgType   = MSG_TYPE_RELAY_STATUS;
  strncpy(sendPkt.senderName, "ESP32-C3", 15);
  sendPkt.messageId = ++msgCounter;
  sendPkt.commandId = CMD_RELAY_STATUS;
  sendPkt.timestamp = millis();
  snprintf(sendPkt.text, sizeof(sendPkt.text),
           "R1:%s R2:%s R3:%s R4:%s",
           relayState[0]?"ON":"OFF",
           relayState[1]?"ON":"OFF",
           relayState[2]?"ON":"OFF",
           relayState[3]?"ON":"OFF");
  esp_now_send(displayMAC, (uint8_t*)&sendPkt, sizeof(DataPacket));
}

// ═════════════════════════════════════════════════════════════════════════════
//  MPU CALIBRATION
//  Simple DC offset calibration: averages 200 samples and stores as bias
// ═════════════════════════════════════════════════════════════════════════════
float calBiasAX=0,calBiasAY=0,calBiasAZ=0;
float calBiasGX=0,calBiasGY=0,calBiasGZ=0;
bool  calDone = false;

void performCalibration(){
  Serial.println("[CAL] Calibrating MPU — keep sensor still...");
  long sumAX=0,sumAY=0,sumAZ=0,sumGX=0,sumGY=0,sumGZ=0;
  int  n=0;
  for(int i=0;i<200;i++){
    int16_t ax,ay,az,gx,gy,gz;
    if(readRawAccel(&ax,&ay,&az) && readRawGyro(&gx,&gy,&gz)){
      sumAX+=ax; sumAY+=ay; sumAZ+=az;
      sumGX+=gx; sumGY+=gy; sumGZ+=gz;
      n++;
    }
    delay(5);
  }
  if(n>0){
    calBiasAX = (float)sumAX/n / ACCEL_SENSITIVITY;
    calBiasAY = (float)sumAY/n / ACCEL_SENSITIVITY;
    calBiasAZ = (float)sumAZ/n / ACCEL_SENSITIVITY - 1.0f; // subtract gravity on Z
    calBiasGX = (float)sumGX/n / GYRO_SENSITIVITY;
    calBiasGY = (float)sumGY/n / GYRO_SENSITIVITY;
    calBiasGZ = (float)sumGZ/n / GYRO_SENSITIVITY;
    calDone   = true;
    Serial.printf("[CAL] Done: biasA=%.3f %.3f %.3f biasG=%.2f %.2f %.2f\n",
                  calBiasAX,calBiasAY,calBiasAZ,
                  calBiasGX,calBiasGY,calBiasGZ);
  } else {
    Serial.println("[CAL] Failed — no valid samples");
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  SEND MPU DATA PACKET TO DISPLAY
// ═════════════════════════════════════════════════════════════════════════════

void sendMPUData(){
  checkMPUHealth();
  int16_t ax,ay,az,gx,gy,gz;
  if(!readRawAccel(&ax,&ay,&az)) return;
  if(!readRawGyro(&gx,&gy,&gz))  return;
  float chipTemp = readChipTemp();

  // Convert to physical units
  float fax = (float)ax / ACCEL_SENSITIVITY;
  float fay = (float)ay / ACCEL_SENSITIVITY;
  float faz = (float)az / ACCEL_SENSITIVITY;
  float fgx = (float)gx / GYRO_SENSITIVITY;
  float fgy = (float)gy / GYRO_SENSITIVITY;
  float fgz = (float)gz / GYRO_SENSITIVITY;

  // Apply calibration offsets if available
  if(calDone){
    fax -= calBiasAX; fay -= calBiasAY; faz -= calBiasAZ;
    fgx -= calBiasGX; fgy -= calBiasGY; fgz -= calBiasGZ;
  }

  // Serial debug (raw + calibrated)
  Serial.printf("AX:%6d AY:%6d AZ:%6d | GX:%6d GY:%6d GZ:%6d | "
                "AX:%6.3fg AY:%6.3fg AZ:%6.3fg | GX:%7.2f GY:%7.2f GZ:%7.2f | T:%.1fC\n",
                ax,ay,az,gx,gy,gz,fax,fay,faz,fgx,fgy,fgz,chipTemp);

  if(!espnowReady) return;

  memset(&sendPkt, 0, sizeof(sendPkt));
  sendPkt.msgType   = MSG_TYPE_MPU_DATA;
  strncpy(sendPkt.senderName, "ESP32-C3", 15);
  sendPkt.messageId = ++msgCounter;
  sendPkt.commandId = 0;
  // val1-val6: AccXYZ (g), GyrXYZ (°/s);  val7: chip temperature °C
  sendPkt.val1 = fax;
  sendPkt.val2 = fay;
  sendPkt.val3 = faz;
  sendPkt.val4 = fgx;
  sendPkt.val5 = fgy;
  sendPkt.val6 = fgz;
  sendPkt.val7 = chipTemp;
  sendPkt.timestamp = millis();
  snprintf(sendPkt.text, sizeof(sendPkt.text),
           "A:%.2f,%.2f,%.2f G:%.1f,%.1f,%.1f T:%.1f",
           fax,fay,faz,fgx,fgy,fgz,chipTemp);

  esp_err_t err = esp_now_send(displayMAC, (uint8_t*)&sendPkt, sizeof(DataPacket));
  if(err != ESP_OK)
    Serial.printf("[ESP-NOW] MPU send err %d\n", err);
}

// ═════════════════════════════════════════════════════════════════════════════
//  ESP-NOW — SEND ACK
// ═════════════════════════════════════════════════════════════════════════════

void sendAck(uint32_t rxMsgId, int32_t rxCmdId, const char* tag){
  if(!espnowReady) return;
  memset(&sendPkt, 0, sizeof(sendPkt));
  sendPkt.msgType   = MSG_TYPE_ACK;
  strncpy(sendPkt.senderName, "ESP32-C3", 15);
  sendPkt.messageId = ++msgCounter;
  sendPkt.commandId = rxCmdId;
  sendPkt.val1      = (float)rxMsgId;
  sendPkt.timestamp = millis();
  snprintf(sendPkt.text, sizeof(sendPkt.text), "ACK:%lu:%s",
           (unsigned long)rxMsgId, tag ? tag : "OK");
  esp_now_send(displayMAC, (uint8_t*)&sendPkt, sizeof(DataPacket));
}

// ═════════════════════════════════════════════════════════════════════════════
//  ESP-NOW CALLBACKS  —  IDF v4 + v5 compatible
// ═════════════════════════════════════════════════════════════════════════════

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
void IRAM_ATTR onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len){
#else
void IRAM_ATTR onDataRecv(const uint8_t *mac, const uint8_t *data, int len){
#endif
  if(len != sizeof(DataPacket)) return;
  memcpy(&recvPkt, data, sizeof(DataPacket));
  newCmdRx = true;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t st){
  if(st != ESP_NOW_SEND_SUCCESS) Serial.println("[ESP-NOW] Send failed");
}
#else
void onDataSent(const uint8_t *mac, esp_now_send_status_t st){
  if(st != ESP_NOW_SEND_SUCCESS) Serial.println("[ESP-NOW] Send failed");
}
#endif

void espnowInit(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  // Lock channel BEFORE esp_now_init — must match display
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if(esp_now_init() != ESP_OK){
    Serial.println("[ESP-NOW] Init failed");
    return;
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  espnowReady = true;

  // Add display as peer
  if(esp_now_is_peer_exist(displayMAC)) esp_now_del_peer(displayMAC);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, displayMAC, 6);
  peer.channel = 1;
  peer.encrypt = false;
  esp_err_t err = esp_now_add_peer(&peer);
  if(err == ESP_OK)
    Serial.printf("[ESP-NOW] Peer added %02X:%02X:%02X:%02X:%02X:%02X ch=1\n",
      displayMAC[0],displayMAC[1],displayMAC[2],
      displayMAC[3],displayMAC[4],displayMAC[5]);
  else
    Serial.printf("[ESP-NOW] Peer add error %d\n", err);
}

// ═════════════════════════════════════════════════════════════════════════════
//  PROCESS RECEIVED COMMAND
// ═════════════════════════════════════════════════════════════════════════════
void processCommand(const DataPacket& pkt){
  // Never ACK an ACK
  if(pkt.msgType == MSG_TYPE_ACK) return;

  // Send ACK first
  sendAck(pkt.messageId, pkt.commandId, "OK");

  int32_t cmd = pkt.commandId;
  Serial.printf("[CMD] Received cmd=%d\n", (int)cmd);

  switch(cmd){
    // ── Relay on/off ──────────────────────────────────────────────────────
    case CMD_RELAY1_ON:  setRelay(0,true);  sendRelayStatus(); break;
    case CMD_RELAY1_OFF: setRelay(0,false); sendRelayStatus(); break;
    case CMD_RELAY2_ON:  setRelay(1,true);  sendRelayStatus(); break;
    case CMD_RELAY2_OFF: setRelay(1,false); sendRelayStatus(); break;
    case CMD_RELAY3_ON:  setRelay(2,true);  sendRelayStatus(); break;
    case CMD_RELAY3_OFF: setRelay(2,false); sendRelayStatus(); break;
    case CMD_RELAY4_ON:  setRelay(3,true);  sendRelayStatus(); break;
    case CMD_RELAY4_OFF: setRelay(3,false); sendRelayStatus(); break;

    // ── Relay status query ────────────────────────────────────────────────
    case CMD_RELAY_STATUS:
      sendRelayStatus();
      break;

    // ── MPU streaming ─────────────────────────────────────────────────────
    case CMD_MPU_START:
      mpuStreamActive = true;
      Serial.println("[MPU] Stream ON");
      break;
    case CMD_MPU_STOP:
      mpuStreamActive = false;
      Serial.println("[MPU] Stream OFF");
      break;
    case CMD_MPU_ONCE:
      mpuOnceRequested = true;
      break;

    // ── Calibration ───────────────────────────────────────────────────────
    case CMD_MPU_CALIBRATE:
      mpuCalRequested = true;
      break;

    // ── Ping / keep-alive ─────────────────────────────────────────────────
    case CMD_PING:
      // ACK already sent above — nothing more needed
      break;

    default:
      Serial.printf("[CMD] Unknown command %d\n", (int)cmd);
      break;
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║  ARS IIT Worker  —  ESP32-C3 Mini v11.2  ║");
  Serial.println("╠══════════════════════════════════════════╣");
  Serial.println("║  SDA→GPIO8  SCL→GPIO9  VCC→3.3V  GND→GND║");
  Serial.println("╚══════════════════════════════════════════╝");

  // Print own MAC so it can be pasted into display code
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("\n[WORKER MAC] %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Serial.printf("[DISPLAY MAC target] %02X:%02X:%02X:%02X:%02X:%02X\n",
                displayMAC[0],displayMAC[1],displayMAC[2],
                displayMAC[3],displayMAC[4],displayMAC[5]);
  Serial.printf("[IDF] %s\n", esp_get_idf_version());

  // ── Relay pins ──────────────────────────────────────────────────────────
  for(int i=0;i<4;i++){
    pinMode(relayPin[i], OUTPUT);
    digitalWrite(relayPin[i], HIGH);   // relays OFF (active-LOW module)
  }
  Serial.println("[RELAY] All pins initialized (OFF)");

  // ── I2C + MPU ───────────────────────────────────────────────────────────
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  delay(100);
  Serial.println("[I2C] Initialized GPIO8=SDA GPIO9=SCL 400kHz");

  if(detectMPU()){
    initializeMPU();
    lastSuccessMs = millis();
  } else {
    Serial.println("[MPU] WARN: No MPU found — continuing without sensor");
  }

  // ── ESP-NOW ─────────────────────────────────────────────────────────────
  espnowInit();

  // Announce ready
  Serial.println("\n[WORKER] Ready. Waiting for commands...\n");
  Serial.println("─────────────────────────────────────────────────────────────────");
  Serial.println(" AX(raw)  AY(raw)  AZ(raw)  |  GX(raw)  GY(raw)  GZ(raw)  | AX(g)   AY(g)   AZ(g)   | GX(°/s)  GY(°/s)  GZ(°/s)  | Temp");
  Serial.println("─────────────────────────────────────────────────────────────────");
}

// ═════════════════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── Handle received commands (safe outside ISR) ──────────────────────────
  if(newCmdRx){
    newCmdRx = false;
    DataPacket pktCopy;
    memcpy(&pktCopy, &recvPkt, sizeof(DataPacket));
    processCommand(pktCopy);
  }

  // ── Calibration (deferred from ISR context) ──────────────────────────────
  if(mpuCalRequested){
    mpuCalRequested = false;
    performCalibration();
    // Send ACK with CAL result so display clears the "CAL pending" badge
    sendAck(0, CMD_MPU_CALIBRATE, calDone?"DONE":"FAIL");
  }

  // ── Single-shot MPU read ─────────────────────────────────────────────────
  if(mpuOnceRequested){
    mpuOnceRequested = false;
    sendMPUData();
  }

  // ── Continuous MPU stream at ~20 Hz ─────────────────────────────────────
  if(mpuStreamActive && now-lastStreamMs >= STREAM_INTERVAL_MS){
    lastStreamMs = now;
    sendMPUData();
  }
}
// ═══════════════════════════════ END OF FILE ═══════════════════════════════
