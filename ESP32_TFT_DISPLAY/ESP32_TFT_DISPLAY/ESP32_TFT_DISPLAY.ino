/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  DISPLAY_ARS_IIT.ino  —  ARS IIT v11.2  (DISPLAY / CONTROLLER BOARD)
 *  Board  : ESP32 (38-pin, with 2.8" TFT 320×240 + XPT2046 touch)
 *  Partner: ESP32-C3 Mini (WORKER) at MAC  1C:DB:D4:38:BD:FC
 *
 *  COMMUNICATION FIXES APPLIED:
 *   1. Correct peer MAC = Worker STA MAC  {0x1C,0xDB,0xD4,0x38,0xBD,0xFC}
 *   2. Both boards forced to WiFi channel 1 before esp_now_init()
 *   3. esp_now_register_recv_cb uses IDF-version-safe wrapper
 *   4. DataPacket struct is __attribute__((packed)) — same on both sides
 *   5. Peer re-added after every reconnect attempt
 *   6. ACK sent only for non-ACK packets (no ACK loop)
 *   7. MPU data unpacked: val1-val6 = ax,ay,az,gx,gy,gz  val7=chipTemp
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <WiFi.h>
#include <Preferences.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include "qrcode.h"
#include <math.h>

// ── Touchscreen (VSPI bus) ────────────────────────────────────────────────────
#define XPT2046_IRQ   36
#define XPT2046_MOSI  32
#define XPT2046_MISO  39
#define XPT2046_CLK   25
#define XPT2046_CS    33

// ── Display — 2.8-inch TFT 320×240 ───────────────────────────────────────────
#define SW  320
#define SH  240
#define M     6

// ── Timing ────────────────────────────────────────────────────────────────────
#define SLEEP_TIMEOUT_MS   120000UL
#define ESPNOW_TIMEOUT_MS   15000UL
#define VIBRATION_POLL_MS     200UL
#define TOUCH_WAKEUP_PIN    GPIO_NUM_36
#define GRAPH_FPS_MS           80UL

// ═════════════════════════════════════════════════════════════════════════════
//  RGB565 COLOR PALETTE
// ═════════════════════════════════════════════════════════════════════════════
#define BG_DEEP     0x0000
#define BG_NAVY     0x000D
#define BG_DARK     0x0012
#define BG_MID      0x0831
#define BG_CARD     0x0832
#define BG_CARD_HI  0x1053
#define BG_RAISED   0x18B4

#define CYAN        0x07FF
#define CYAN_HI     0x6FFF
#define CYAN_MID    0x0596
#define CYAN_DIM    0x034A
#define CYAN_DARK   0x0129
#define CYAN_GLOW   0x0022

#define CORAL       0xFB8A
#define CORAL_HI    0xFDAF
#define CORAL_MID   0xF806
#define CORAL_DIM   0x6000
#define CORAL_GLOW  0x1800

#define NGREEN      0x07E0
#define NGREEN_HI   0x87F0
#define NGREEN_MID  0x0400
#define NGREEN_DIM  0x0200
#define NGREEN_GLOW 0x0040

#define AMBER       0xFD60
#define AMBER_HI    0xFEE0
#define AMBER_MID   0xFB00
#define AMBER_DIM   0x8280

#define PURPLE_V    0xC01F
#define PURPLE_HI   0xD89F
#define PURPLE_MID  0x6008
#define PURPLE_DIM  0x3004

#define METAL_WHITE 0xFFFF
#define METAL_SPEC  0xDEFB
#define METAL_HI    0xCE59
#define METAL_MID   0x9CF3
#define METAL_LOW   0x6B6D
#define METAL_SH    0x4A49
#define METAL_DARK  0x2104

#define TXT_PRI     0xFFFF
#define TXT_SEC     0xCE79
#define TXT_DIM     0x8C51
#define TXT_VDIM    0x4A29

#define ZONE_A_COL  0x07E0
#define ZONE_B_COL  0xFFE0
#define ZONE_C_COL  0xFD60
#define ZONE_D_COL  0xF800

#define F1 1
#define F2 2
#define F4 4

// ── Cylinder geometry ─────────────────────────────────────────────────────────
#define SC_Y0        52
#define SC_Y1       168
#define SC_FLOOR_H    8
#define STAND_X0      8
#define STAND_X1     28
#define STAND_W      (STAND_X1-STAND_X0)
#define RAIL_Y       SC_Y0
#define RAIL_H        8
#define C2_BX0       STAND_X1
#define C2_BX1       92
#define C2_BW        (C2_BX1-C2_BX0)
#define C2_BY0       RAIL_Y
#define C2_BH        28
#define C2_BY1       (C2_BY0+C2_BH)
#define C2_ROD_Y0    (C2_BY0+8)
#define C2_ROD_H     12
#define C2_TIP_RET   C2_BX1
#define C2_TIP_EXT   200
#define C3_BW         20
#define C3_BH         32
#define C3_ROD_W       8
#define C3_BODY_Y0    SC_Y0
#define C3_BODY_Y1    (SC_Y0 + C3_BH)
#define C3_ROD_MAX_Y  (SC_Y1 - 14)

TFT_eSPI            tft  = TFT_eSPI();
TFT_eSprite         spr  = TFT_eSprite(&tft);
SPIClass            tsSPI(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
Preferences         prefs;

// ═════════════════════════════════════════════════════════════════════════════
//  ESP-NOW  —  WORKER MAC  =  {0x1C, 0xDB, 0xD4, 0x38, 0xBD, 0xFC}
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

// Compile-time size guard — both boards must agree
static_assert(sizeof(DataPacket) == (1 + 16 + 4 + 4 + 7*4 + 128 + 4),
              "DataPacket size mismatch");

// Worker board MAC (STA MAC printed on serial at boot)
uint8_t   peerMAC[6]  = {0x1C, 0xDB, 0xD4, 0x38, 0xBD, 0xFC};
bool      peerPaired  = false;
bool      espnowReady = false;
uint32_t  msgCounter  = 0;

DataPacket sendPkt;
DataPacket recvPkt;
volatile bool newDataRx  = false;
bool          peerAlive  = false;
unsigned long lastPeerMs = 0;

bool    coil[4]   = {false,false,false,false};
int16_t holdReg0  = 0;
const char* coilLabel[4] = {"RELAY 1","RELAY 2","RELAY 3","RELAY 4"};
const char* coilGPIO[4]  = {"GPIO 2","GPIO 3","GPIO 4","GPIO 5"};

float   imuAccX=0, imuAccY=0, imuAccZ=1.0f;
float   imuGyrX=0, imuGyrY=0, imuGyrZ=0;
float   imuChipTempC = 25.0f;
bool    imuValid = false;
unsigned long lastVibPoll = 0;

bool calRequestPending = false;
unsigned long calRequestMs = 0;

#define VIBGRAPH_N   80
float vibBufX[VIBGRAPH_N] = {0};
float vibBufY[VIBGRAPH_N] = {0};
float vibBufZ[VIBGRAPH_N] = {0};
int   vibBufIdx = 0;

float dcX = 0.0f, dcY = 0.0f, dcZ = 1.0f;
#define DC_ALPHA  0.02f

float vibAccRMS    = 0.0f;
float vibPeakDyn   = 0.0f;
float vibMinDyn    = 9999.0f;
float vibMaxDyn    = 0.0f;
float vibCrestFact = 0.0f;
float vibVelocRMS  = 0.0f;
float vibFreqEst   = 50.0f;
float vibScale     = 1.0f;
float vibPlotScale = 0.5f;
float vibCurMag    = 0.0f;

enum VibZone { ZONE_A=0, ZONE_B, ZONE_C, ZONE_D };
VibZone vibZone = ZONE_A;
const char* zoneLabel[4] = {"GOOD","SATISF","ALERT","CRIT"};
const uint16_t zoneColor[4] = {ZONE_A_COL, ZONE_B_COL, ZONE_C_COL, ZONE_D_COL};

unsigned long lastGraphMs = 0;

#define GX_PUSH  24
#define GY_PUSH  112
#define SPR_W  (SW-GX_PUSH-4)
#define SPR_H  (SH-GY_PUSH-6)
bool spriteAllocated = false;

bool condUpdatePending = false;

enum Screen { SCR_HOME, SCR_CONTROL, SCR_MANUAL, SCR_AUTO,
              SCR_INFO, SCR_CONN, SCR_CAL, SCR_VIBRATION };
enum InputField { FLD_NONE, FLD_MAC };

Screen    curScreen  = SCR_HOME;
unsigned long lastTouchMs = 0;

bool     qrFullscreen = false;
int16_t  infoScrollY  = 0;
#define  INFO_MAX_SCROLL  430
#define  INFO_TAP_SLOP    6
#define  QR_CONTENT_Y     476
#define  QR_IMG_X         (M+8)
#define  QR_IMG_W         64
#define  QR_BTN_X         (M+8+64+8)
#define  QR_BTN_W         90
#define  QR_BTN_H         24
int      touchStartY  = -1;
int      touchStartX  = -1;
int      touchPrevY   = -1;
bool     touchDragging= false;

#define CAL_N 9
struct CalPt { uint16_t rx, ry; int16_t dx, dy; };
CalPt   calPts[CAL_N];
bool    calDone = false;
int     calStep = 0;
float   calAff[6] = {0,0,0,0,0,0};
const int16_t calTX[CAL_N] = {20,160,300,20,160,300,20,160,300};
const int16_t calTY[CAL_N] = {30,30,30,120,120,120,210,210,210};

struct CylState { float pos; int target; bool animating; float speed; };
CylState cyl2 = {0.0f,0,false,0.7f};
CylState cyl3 = {0.0f,0,false,0.7f};
unsigned long lastAnimMs = 0;

bool showVibPopup = false;
bool vibFromConn = false;
InputField activeField = FLD_NONE;
char       kbBuf[64]   = "";
int        kbLen       = 0;

static QRCode  _qrObj;
static uint8_t _qrBuf[110];
static bool    _qrReady = false;

uint8_t pulsePhase  = 0;
uint8_t breathPhase = 0;

// ── Forward declarations ──────────────────────────────────────────────────────
void drawHome(); void drawControl(); void drawManual(); void drawAuto();
void drawInfo(); void drawConn();    void drawCal();    void drawSplash();
void drawVibration(); void drawQRFullscreen(); void drawCalPoint();
void drawTopBar(const char* title, bool showBack=true);
void handleTouch(int,int);
void handleHomeT(int,int); void handleCtrlT(int,int); void handleManT(int,int);
void handleAutoT(int,int); void handleInfoT(int,int); void handleConnT(int,int);
void handleCalT(int,int);  void handleKbT(int,int);   void handleVibT(int,int);
void openKb(InputField,const char*); void drawKb(InputField);
bool inR(int,int,int,int,int,int);
void enterDeepSleep();
void loadPrefs(); void savePrefs(); void saveCal(); void loadCal();
void computeAffineCalibration(); void mapTouch(int,int,int&,int&);
void espnowSendCmd(int32_t); void espnowInit(); void espnowAddPeer();
void espnowRequestVibration();
void espnowSendAck(uint32_t,int32_t,const char*);
void goHome();
void drawRelayCard(int); void drawHRBar(); void drawCalBtn(int,int,int,int);
void drawAutoScene(); void updateCylAnims(float dt);
void drawVibPopup(); void drawVibIcon();
void initQR(); void drawQRPixelArt(int ox,int oy,int scale);
void computeVibMetrics();
void drawVibGraphSprite();
void drawVibHeaderCompact();
void drawVibAxisLabels();
float computePlotScaleFromBuffer();
void drawConditionPanel(int x, int y, int w, int h);

uint16_t lerpColor(uint16_t a, uint16_t b, uint8_t t);
void gradientV(int,int,int,int,uint16_t,uint16_t);
void gradientH(int,int,int,int,uint16_t,uint16_t);
void neonRect(int,int,int,int,uint16_t,uint8_t r=6);
void neonBtn(int,int,int,int,uint16_t,uint8_t r=8);
void neonBtnPress(int,int,int,int,uint16_t,uint8_t r=8);
void cardPanel(int,int,int,int,uint16_t accent=0,uint8_t r=10);
void glowDot(int,int,int,uint16_t);
void drawStatusLED(int,int,bool);
void screenWipe();
void fillGradientBG();
void metalBar2(int,int,int,int,uint16_t,bool);
void metalCylinder2(int,int,int,int,bool,uint16_t);
void rodChrome2(int,int,int,bool,int);
void pistonHead2(int,int,int,int,bool);
void screwHead2(int,int,int);
void drawGearIcon(int,int,int,uint16_t);
void drawInfoIcon(int,int,int,uint16_t);
void drawWifiIcon(int,int,int,uint16_t);
void drawTargetIcon(int,int,int,uint16_t);
void drawPlayIcon(int,int,int,uint16_t);
void drawHandIcon(int,int,int,uint16_t);

inline int getCyl2TipX()   { return (int)(C2_TIP_RET + cyl2.pos*(C2_TIP_EXT - C2_TIP_RET)); }
inline int getCyl3BodyX0() { return getCyl2TipX() - C3_BW/2; }
inline int getCyl3TipY()   { return C3_BODY_Y1 + (int)(cyl3.pos * (C3_ROD_MAX_Y - C3_BODY_Y1)); }

// ═════════════════════════════════════════════════════════════════════════════
//  COLOR / DRAWING PRIMITIVES
// ═════════════════════════════════════════════════════════════════════════════

uint16_t lerpColor(uint16_t c1, uint16_t c2, uint8_t t) {
  uint8_t r1=(c1>>11)&0x1F, g1=(c1>>5)&0x3F, b1=c1&0x1F;
  uint8_t r2=(c2>>11)&0x1F, g2=(c2>>5)&0x3F, b2=c2&0x1F;
  uint8_t r=r1+((int)(r2-r1)*t>>8);
  uint8_t g=g1+((int)(g2-g1)*t>>8);
  uint8_t b=b1+((int)(b2-b1)*t>>8);
  return (r<<11)|(g<<5)|b;
}

void gradientV(int x,int y,int w,int h,uint16_t top,uint16_t bot) {
  for(int i=0;i<h;i++) tft.drawFastHLine(x,y+i,w,lerpColor(top,bot,(i*255)/max(1,h-1)));
}

void gradientH(int x,int y,int w,int h,uint16_t left,uint16_t right) {
  for(int i=0;i<w;i++) tft.drawFastVLine(x+i,y,h,lerpColor(left,right,(i*255)/max(1,w-1)));
}

void fillGradientBG() {
  gradientV(0,0,SW,SH/3,0x1053,BG_DEEP);
  gradientV(0,SH/3,SW,SH/3,BG_DEEP,0x000D);
  gradientV(0,2*SH/3,SW,SH/3+1,0x000D,0x0012);
}

void cardPanel(int x,int y,int w,int h,uint16_t accent,uint8_t r) {
  tft.fillRoundRect(x+2,y+2,w,h,r,BG_DEEP);
  gradientV(x,y,w,h,BG_CARD_HI,BG_CARD);
  tft.drawFastHLine(x+r,y,w-r*2,lerpColor(BG_CARD_HI,METAL_MID,80));
  tft.drawRoundRect(x,y,w,h,r,lerpColor(BG_CARD_HI,METAL_LOW,128));
  if(accent){
    tft.fillRoundRect(x+2,y+1,w-4,3,1,accent);
    tft.drawFastHLine(x+4,y+4,w-8,lerpColor(accent,BG_CARD_HI,200));
  }
}

void neonRect(int x,int y,int w,int h,uint16_t col,uint8_t r) {
  tft.drawRoundRect(x-2,y-2,w+4,h+4,r+2,lerpColor(BG_DEEP,col,30));
  tft.drawRoundRect(x-1,y-1,w+2,h+2,r+1,lerpColor(BG_DEEP,col,60));
  tft.drawRoundRect(x,y,w,h,r,col);
}

void neonBtn(int x,int y,int w,int h,uint16_t col,uint8_t r) {
  tft.fillRoundRect(x+1,y+2,w,h,r,BG_DEEP);
  gradientV(x,y,w,h,lerpColor(BG_RAISED,col,40),BG_CARD);
  tft.drawRoundRect(x,y,w,h,r,col);
  tft.drawFastHLine(x+r,y,w-r*2,lerpColor(col,METAL_WHITE,80));
  tft.drawFastHLine(x+r+2,y+1,w-r*2-4,lerpColor(col,BG_CARD,100));
}

void neonBtnPress(int x,int y,int w,int h,uint16_t col,uint8_t r) {
  gradientV(x,y,w,h,lerpColor(BG_DEEP,col,60),lerpColor(BG_DEEP,col,30));
  tft.drawRoundRect(x,y,w,h,r,col);
  tft.drawRoundRect(x+1,y+1,w-2,h-2,r-1,lerpColor(col,BG_DEEP,128));
}

void glowDot(int cx,int cy,int r,uint16_t col) {
  for(int i=r+4;i>r;i--)
    tft.drawCircle(cx,cy,i,lerpColor(BG_DEEP,col,(r+4-i)*40));
  tft.fillCircle(cx,cy,r,col);
  if(r>=3) tft.fillCircle(cx-r/3,cy-r/3,max(1,r/3),METAL_WHITE);
}

void drawStatusLED(int x,int y,bool alive) {
  uint16_t col = alive?NGREEN:CORAL;
  uint8_t bright=180+(int)(75.0f*sinf(pulsePhase*0.0245f));
  uint16_t animCol=lerpColor(col,METAL_WHITE,alive?bright/4:0);
  for(int rr=10;rr>5;rr--)
    tft.drawCircle(x,y,rr,lerpColor(BG_DEEP,col,(10-rr)*20));
  tft.fillCircle(x,y,5,animCol);
  tft.drawCircle(x,y,5,lerpColor(col,METAL_WHITE,60));
  tft.fillCircle(x,y-1,2,lerpColor(animCol,METAL_WHITE,120));
}

void screwHead2(int cx,int cy,int r) {
  tft.fillCircle(cx,cy,r+1,METAL_SH);
  tft.fillCircle(cx,cy,r,METAL_MID);
  tft.drawCircle(cx,cy,r,METAL_HI);
  if(r>=2){
    tft.drawLine(cx-r+1,cy,cx+r-1,cy,METAL_SH);
    tft.drawLine(cx,cy-r+1,cx,cy+r-1,METAL_SH);
  }
  tft.drawPixel(cx-1,cy-1,METAL_SPEC);
}

void screenWipe() {
  for(int x=0;x<SW;x+=8){
    tft.fillRect(x,0,8,SH,BG_DEEP);
    if(x<SW/2) delay(1);
  }
}

// ── Icons ─────────────────────────────────────────────────────────────────────
void drawGearIcon(int cx,int cy,int sz,uint16_t col) {
  tft.fillCircle(cx,cy,sz-2,col);
  tft.fillCircle(cx,cy,sz-5,BG_CARD);
  tft.drawCircle(cx,cy,sz-7,col);
  for(int a=0;a<360;a+=45){
    float r2=a*0.01745f;
    int x1=cx+(int)((sz-4)*cosf(r2)), y1=cy+(int)((sz-4)*sinf(r2));
    int x2=cx+(int)((sz+1)*cosf(r2)), y2=cy+(int)((sz+1)*sinf(r2));
    tft.fillRect(min(x1,x2)-1,min(y1,y2)-1,abs(x2-x1)+3,abs(y2-y1)+3,col);
  }
  tft.fillCircle(cx,cy,sz/3,col);
  tft.fillCircle(cx,cy,sz/5,BG_CARD);
}

void drawInfoIcon(int cx,int cy,int sz,uint16_t col) {
  tft.drawCircle(cx,cy,sz,col);
  tft.drawCircle(cx,cy,sz-1,lerpColor(col,BG_DEEP,100));
  tft.fillCircle(cx,cy-sz/2,sz/5+1,col);
  tft.fillRect(cx-sz/5,cy-sz/4,sz/5*2+1,sz*3/4,col);
}

void drawWifiIcon(int cx,int cy,int sz,uint16_t col) {
  tft.fillCircle(cx,cy+sz/2,sz/5+1,col);
  for(int rr=sz/3;rr<=sz;rr+=sz/3)
    tft.drawArc(cx,cy+sz/2,rr,rr-2,220,320,col,BG_CARD);
}

void drawTargetIcon(int cx,int cy,int sz,uint16_t col) {
  tft.drawCircle(cx,cy,sz,col);
  tft.drawCircle(cx,cy,sz*2/3,col);
  tft.drawCircle(cx,cy,sz/3,col);
  tft.fillCircle(cx,cy,sz/6+1,col);
  tft.drawFastHLine(cx-sz-3,cy,sz*2+7,lerpColor(col,BG_DEEP,160));
  tft.drawFastVLine(cx,cy-sz-3,sz*2+7,lerpColor(col,BG_DEEP,160));
}

void drawPlayIcon(int cx,int cy,int sz,uint16_t col) {
  tft.fillTriangle(cx-sz/2,cy-sz*3/4,cx-sz/2,cy+sz*3/4,cx+sz,cy,col);
}

void drawHandIcon(int cx,int cy,int sz,uint16_t col) {
  tft.fillRoundRect(cx-sz/2,cy-sz/4,sz,sz*3/4,3,col);
  for(int i=0;i<4;i++){
    int fx=cx-sz/2+2+i*(sz/4);
    tft.fillRoundRect(fx,cy-sz*3/4+i*2,sz/5,sz/2,2,col);
  }
  tft.fillRoundRect(cx-sz*3/4,cy,sz/5,sz/3,2,col);
}

void metalBar2(int x,int y,int w,int h,uint16_t mid,bool horiz) {
  if(horiz){
    gradientV(x,y,w,h,lerpColor(mid,METAL_SPEC,60),lerpColor(mid,BG_DEEP,80));
    tft.drawFastHLine(x,y,w,lerpColor(mid,METAL_WHITE,100));
    tft.drawFastHLine(x,y+h-1,w,METAL_DARK);
  } else {
    gradientH(x,y,w,h,lerpColor(mid,METAL_SPEC,60),lerpColor(mid,BG_DEEP,80));
    tft.drawFastVLine(x,y,h,lerpColor(mid,METAL_WHITE,100));
    tft.drawFastVLine(x+w-1,y,h,METAL_DARK);
  }
}

void metalCylinder2(int x,int y,int w,int h,bool horiz,uint16_t accent) {
  if(horiz){
    gradientV(x,y,w,h,METAL_HI,METAL_SH);
    tft.fillRect(x,y+2,w,max(2,h/7),METAL_SPEC);
    tft.fillRect(x,y+2+max(2,h/7),w,max(2,h/7),METAL_HI);
    tft.fillRect(x,y+h-max(2,h/6),w,max(2,h/6),METAL_DARK);
    tft.drawFastHLine(x,y+h/2,w,METAL_LOW);
    gradientV(x,y,5,h,METAL_MID,METAL_DARK);
    gradientV(x+w-5,y,5,h,METAL_MID,METAL_DARK);
    screwHead2(x+8,y+h/2,3); screwHead2(x+w-9,y+h/2,3);
    tft.drawFastHLine(x,y,w,METAL_SPEC);
    tft.drawFastHLine(x,y+h-1,w,BG_DEEP);
    if(accent) tft.drawFastHLine(x+2,y+1,w-4,accent);
  } else {
    gradientH(x,y,w,h,METAL_HI,METAL_SH);
    tft.fillRect(x+2,y,max(2,w/7),h,METAL_SPEC);
    tft.fillRect(x+w-max(2,w/6),y,max(2,w/6),h,METAL_DARK);
    tft.drawFastVLine(x+w/2,y,h,METAL_LOW);
    gradientH(x,y,w,5,METAL_MID,METAL_DARK);
    gradientH(x,y+h-5,w,5,METAL_MID,METAL_DARK);
    screwHead2(x+w/2,y+7,3); screwHead2(x+w/2,y+h-8,3);
    tft.drawFastVLine(x,y,h,METAL_SPEC);
    tft.drawFastVLine(x+w-1,y,h,BG_DEEP);
    if(accent) tft.drawFastVLine(x+1,y+2,h-4,accent);
  }
}

void rodChrome2(int x,int y,int len,bool horiz,int thick) {
  if(len<=0) return;
  if(horiz){
    gradientV(x,y,len,thick,METAL_WHITE,METAL_LOW);
    tft.drawFastHLine(x,y,len,METAL_SPEC);
    tft.fillRect(x,y+1,len,max(1,thick/5),lerpColor(METAL_WHITE,METAL_SPEC,128));
  } else {
    gradientH(x,y,thick,len,METAL_WHITE,METAL_LOW);
    tft.drawFastVLine(x,y,len,METAL_SPEC);
    tft.fillRect(x+1,y,max(1,thick/5),len,lerpColor(METAL_WHITE,METAL_SPEC,128));
  }
}

void pistonHead2(int x,int y,int w,int h,bool horiz) {
  if(horiz){
    gradientV(x,y,w,h,METAL_HI,METAL_SH);
    tft.drawFastHLine(x,y,w,METAL_SPEC);
    tft.drawFastVLine(x,y,h,METAL_SPEC);
    tft.drawFastVLine(x+w-1,y,h,METAL_DARK);
    screwHead2(x+w/2,y+h/2,3);
  } else {
    gradientH(x,y,w,h,METAL_HI,METAL_SH);
    tft.drawFastVLine(x,y,h,METAL_SPEC);
    tft.drawFastHLine(x,y,w,METAL_SPEC);
    tft.drawFastHLine(x,y+h-1,w,METAL_DARK);
    screwHead2(x+w/2,y+h/2,3);
  }
}

void initQR() {
  if(_qrReady) return;
  qrcode_initText(&_qrObj,_qrBuf,3,ECC_LOW,"https://github.com/madebygyanesh/ARS_IIT");
  _qrReady=true;
}

void drawQRPixelArt(int ox,int oy,int scale) {
  initQR();
  int qSide=_qrObj.size, qz=3;
  tft.fillRect(ox-qz,oy-qz,qSide*scale+qz*2,qSide*scale+qz*2,TXT_PRI);
  for(uint8_t row=0;row<qSide;row++)
    for(uint8_t col=0;col<qSide;col++){
      uint16_t c=qrcode_getModule(&_qrObj,col,row)?BG_DEEP:TXT_PRI;
      tft.fillRect(ox+col*scale,oy+row*scale,scale,scale,c);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  TOP BAR
// ═════════════════════════════════════════════════════════════════════════════
void drawTopBar(const char* title, bool showBack) {
  gradientV(0,0,SW,22,lerpColor(BG_NAVY,CYAN_DARK,40),BG_DEEP);
  tft.drawFastHLine(0,22,SW,CYAN_DIM);
  tft.drawFastHLine(0,23,SW,lerpColor(CYAN_DIM,BG_DEEP,160));
  gradientV(0,24,SW,24,BG_DARK,BG_DEEP);
  tft.drawFastHLine(0,47,SW,lerpColor(CYAN,BG_DEEP,200));

  drawStatusLED(12,11,peerAlive);
  tft.setTextColor(peerAlive?NGREEN:CORAL, lerpColor(BG_NAVY,CYAN_DARK,40));
  tft.drawString(peerAlive?"LIVE":"OFFLN", 22, 5, F2);

  if(imuValid && peerAlive){
    neonBtn(90,3,52,16,PURPLE_V,4);
    tft.setTextColor(PURPLE_HI,lerpColor(BG_RAISED,PURPLE_V,40));
    tft.drawCentreString("IMU",116,7,F2);
  }
  if(calRequestPending){
    neonBtnPress(148,3,62,16,AMBER,4);
    tft.setTextColor(BG_DEEP,lerpColor(BG_DEEP,AMBER,60));
    tft.drawCentreString("CAL..",179,7,F2);
  }

  if(title && strlen(title)>0){
    tft.setTextColor(TXT_SEC,BG_DARK);
    tft.drawString(title,8,30,F2);
    int tw2=strlen(title)*7;
    tft.drawFastHLine(8,42,tw2,lerpColor(CYAN,BG_DARK,180));
  }

  if(showBack){
    neonBtn(SW-72,26,66,20,CYAN,6);
    tft.setTextColor(CYAN_HI,lerpColor(BG_RAISED,CYAN,40));
    tft.drawCentreString("< BACK",SW-39,31,F2);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  ESP-NOW CALLBACKS  —  IDF v4 and v5 compatible
// ═════════════════════════════════════════════════════════════════════════════

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
void IRAM_ATTR onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void IRAM_ATTR onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if(len != sizeof(DataPacket)) return;           // size guard
  memcpy(&recvPkt, data, sizeof(DataPacket));
  newDataRx  = true;
  peerAlive  = true;
  lastPeerMs = millis();

  // Send ACK only for non-ACK packets to prevent ACK ping-pong
  if(recvPkt.msgType != MSG_TYPE_ACK){
    espnowSendAck(recvPkt.messageId, recvPkt.commandId, "RX");
  }

  if(recvPkt.msgType == MSG_TYPE_MPU_DATA){
    // val1=AccX  val2=AccY  val3=AccZ  val4=GyrX  val5=GyrY  val6=GyrZ  val7=chipTemp
    imuAccX = recvPkt.val1;
    imuAccY = recvPkt.val2;
    imuAccZ = recvPkt.val3;
    imuGyrX = recvPkt.val4;
    imuGyrY = recvPkt.val5;
    imuGyrZ = recvPkt.val6;
    imuChipTempC = recvPkt.val7;
    imuValid = true;
    vibBufX[vibBufIdx] = imuAccX;
    vibBufY[vibBufIdx] = imuAccY;
    vibBufZ[vibBufIdx] = imuAccZ;
    vibBufIdx = (vibBufIdx+1) % VIBGRAPH_N;
    computeVibMetrics();
    condUpdatePending = true;
  } else if(recvPkt.msgType == MSG_TYPE_RELAY_STATUS){
    const char* t = recvPkt.text;
    coil[0] = (strstr(t,"R1:ON") != nullptr);
    coil[1] = (strstr(t,"R2:ON") != nullptr);
    coil[2] = (strstr(t,"R3:ON") != nullptr);
    coil[3] = (strstr(t,"R4:ON") != nullptr);
  } else if(recvPkt.msgType == MSG_TYPE_ACK){
    if(recvPkt.commandId == CMD_MPU_CALIBRATE) calRequestPending = false;
  }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t st){
  peerPaired = (st == ESP_NOW_SEND_SUCCESS);
}
#else
void onDataSent(const uint8_t *mac, esp_now_send_status_t st){
  peerPaired = (st == ESP_NOW_SEND_SUCCESS);
}
#endif

// ── espnowInit: channel MUST be set before esp_now_init ──────────────────────
void espnowInit(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  // Lock to channel 1 before init — must match worker
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if(esp_now_init() != ESP_OK){
    Serial.println("[ESP-NOW] init failed");
    return;
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  espnowReady = true;
  Serial.println("[ESP-NOW] init OK, ch=1");
}

void espnowAddPeer(){
  if(!espnowReady) return;
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if(esp_now_is_peer_exist(peerMAC)){
    esp_now_del_peer(peerMAC);        // remove stale entry first
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = 1;
  peer.encrypt = false;
  esp_err_t err = esp_now_add_peer(&peer);
  if(err == ESP_OK)
    Serial.printf("[ESP-NOW] peer added %02X:%02X:%02X:%02X:%02X:%02X\n",
      peerMAC[0],peerMAC[1],peerMAC[2],peerMAC[3],peerMAC[4],peerMAC[5]);
  else
    Serial.printf("[ESP-NOW] peer add failed: %d\n", err);
}

void espnowSendCmd(int32_t cmdId){
  if(!espnowReady) return;
  memset(&sendPkt, 0, sizeof(sendPkt));
  sendPkt.msgType   = MSG_TYPE_COMMAND;
  strncpy(sendPkt.senderName, "ESP32-CTRL", 15);
  sendPkt.messageId = ++msgCounter;
  sendPkt.commandId = cmdId;
  sendPkt.timestamp = millis();
  snprintf(sendPkt.text, sizeof(sendPkt.text), "CMD:%d", (int)cmdId);
  esp_err_t err = esp_now_send(peerMAC, (uint8_t*)&sendPkt, sizeof(DataPacket));
  if(err != ESP_OK)
    Serial.printf("[ESP-NOW] send error %d (cmd=%d)\n", err, (int)cmdId);
}

void espnowSendAck(uint32_t rxMsgId, int32_t rxCmdId, const char* tag){
  if(!espnowReady) return;
  memset(&sendPkt, 0, sizeof(sendPkt));
  sendPkt.msgType   = MSG_TYPE_ACK;
  strncpy(sendPkt.senderName, "ESP32-DISP", 15);
  sendPkt.messageId = ++msgCounter;
  sendPkt.commandId = rxCmdId;
  sendPkt.val1      = (float)rxMsgId;
  sendPkt.timestamp = millis();
  snprintf(sendPkt.text, sizeof(sendPkt.text), "ACK:%lu:%s",
           (unsigned long)rxMsgId, tag ? tag : "OK");
  esp_now_send(peerMAC, (uint8_t*)&sendPkt, sizeof(DataPacket));
}

void espnowRequestVibration(){ espnowSendCmd(CMD_MPU_ONCE); }

// ═════════════════════════════════════════════════════════════════════════════
//  VIBRATION METRICS
// ═════════════════════════════════════════════════════════════════════════════
void computeVibMetrics() {
  dcX = dcX*(1.0f-DC_ALPHA) + imuAccX*DC_ALPHA;
  dcY = dcY*(1.0f-DC_ALPHA) + imuAccY*DC_ALPHA;
  dcZ = dcZ*(1.0f-DC_ALPHA) + imuAccZ*DC_ALPHA;

  float sumSq=0.0f, peakDyn=0.0f;
  int zeroCrossings=0;
  float prevDynZ=0.0f;

  for(int i=0;i<VIBGRAPH_N;i++){
    float ax=vibBufX[i]-dcX, ay=vibBufY[i]-dcY, az=vibBufZ[i]-dcZ;
    float vm=sqrtf(ax*ax+ay*ay+az*az);
    sumSq+=vm*vm;
    if(vm>peakDyn) peakDyn=vm;
    if(i>0&&((az>0)!=(prevDynZ>0))) zeroCrossings++;
    prevDynZ=az;
  }

  vibAccRMS  = sqrtf(sumSq/VIBGRAPH_N);
  vibPeakDyn = peakDyn;
  if(peakDyn>vibMaxDyn) vibMaxDyn=peakDyn;
  if(vibAccRMS>0.001f&&peakDyn<vibMinDyn) vibMinDyn=peakDyn;

  float cdx=imuAccX-dcX, cdy=imuAccY-dcY, cdz=imuAccZ-dcZ;
  vibCurMag = sqrtf(cdx*cdx+cdy*cdy+cdz*cdz);
  vibCrestFact = (vibAccRMS>0.001f)?(vibPeakDyn/vibAccRMS):0.0f;

  float fEst = (float)zeroCrossings/(2.0f*16.0f);
  fEst = constrain(fEst, 5.0f, 200.0f);
  vibFreqEst = vibFreqEst*0.8f + fEst*0.2f;
  if(vibFreqEst<5.0f) vibFreqEst=50.0f;

  vibVelocRMS = (vibAccRMS*9810.0f)/(2.0f*3.14159f*vibFreqEst);

  if(vibVelocRMS<1.4f)       vibZone=ZONE_A;
  else if(vibVelocRMS<2.8f)  vibZone=ZONE_B;
  else if(vibVelocRMS<4.5f)  vibZone=ZONE_C;
  else                        vibZone=ZONE_D;

  float targetScale = max(peakDyn*1.25f, vibAccRMS*3.5f);
  if(targetScale<0.12f) targetScale=0.12f;
  if(targetScale>8.0f)  targetScale=8.0f;
  if(targetScale>vibScale) vibScale=vibScale*0.45f+targetScale*0.55f;
  else                      vibScale=vibScale*0.80f+targetScale*0.20f;
  if(vibScale<0.12f) vibScale=0.12f;
}

// ═════════════════════════════════════════════════════════════════════════════
//  CONDITION PANEL
// ═════════════════════════════════════════════════════════════════════════════
void drawConditionPanel(int x,int y,int w,int h) {
  uint16_t zc=zoneColor[(int)vibZone];
  const char* zl=zoneLabel[(int)vibZone];

  tft.fillRoundRect(x,y,w,h,5,lerpColor(BG_DEEP,zc,15));
  tft.drawRoundRect(x,y,w,h,5,zc);
  tft.drawFastHLine(x+1,y+1,w-2,lerpColor(zc,METAL_WHITE,40));

  int bw=w/5;
  tft.fillRoundRect(x+2,y+2,bw,h-4,4,lerpColor(BG_DEEP,zc,60));
  tft.drawRoundRect(x+2,y+2,bw,h-4,4,zc);
  tft.setTextColor(zc,lerpColor(BG_DEEP,zc,60));
  tft.drawCentreString(zl,x+2+bw/2,y+h/2-4,F2);

  int mx=x+bw+5;
  char buf[28];
  tft.setTextColor(CYAN,BG_DEEP);
  tft.drawString("V:",mx,y+2,F2);
  snprintf(buf,28,"%.1fmm/s",vibVelocRMS);
  tft.setTextColor(TXT_PRI,BG_DEEP);
  tft.drawString(buf,mx+16,y+2,F2);

  tft.setTextColor(AMBER,BG_DEEP);
  tft.drawString("R:",mx,y+12,F2);
  snprintf(buf,28,"%.2fg",vibAccRMS);
  tft.setTextColor(TXT_SEC,BG_DEEP);
  tft.drawString(buf,mx+16,y+12,F2);

  int rx2=mx+86;
  tft.setTextColor(CORAL,BG_DEEP);
  tft.drawString("CF",rx2,y+2,F2);
  snprintf(buf,12,"%.1f",vibCrestFact);
  tft.setTextColor(vibCrestFact>6?CORAL:(vibCrestFact>3?AMBER:TXT_PRI),BG_DEEP);
  tft.drawString(buf,rx2+18,y+2,F2);

  snprintf(buf,14,"T%.0fC",imuChipTempC);
  tft.setTextColor(lerpColor(NGREEN,CORAL,(uint8_t)constrain((imuChipTempC-40)*6,0,255)),BG_DEEP);
  tft.drawString(buf,rx2,y+12,F2);
}

// ═════════════════════════════════════════════════════════════════════════════
//  VIB GRAPH SPRITE
// ═════════════════════════════════════════════════════════════════════════════
void drawVibGraphSprite() {
  if(!spriteAllocated){
    spr.setColorDepth(16);
    spr.setTextWrap(false);
    if(!spr.createSprite(SPR_W,SPR_H)) return;
    spriteAllocated=true;
  }

  spr.fillSprite(BG_DEEP);
  uint16_t gridDim = lerpColor(BG_DEEP, NGREEN_DIM, 42);
  uint16_t gridMid = lerpColor(BG_DEEP, NGREEN_DIM, 88);
  int midY = SPR_H/2;

  for(int gy=0;gy<SPR_H;gy+=18) spr.drawFastHLine(0,gy,SPR_W,gridDim);
  for(int gx=0;gx<SPR_W;gx+=24) spr.drawFastVLine(gx,0,SPR_H,gridDim);
  spr.drawFastHLine(0,midY,SPR_W,gridMid);

  struct Ch { uint16_t col; float* buf; int axis; } ch[3] = {
    {CYAN,   vibBufX, 0},
    {NGREEN, vibBufY, 1},
    {CORAL,  vibBufZ, 2},
  };

  float sc = computePlotScaleFromBuffer();

  for(int c=0;c<3;c++){
    for(int i=1;i<VIBGRAPH_N;i++){
      int i1=(vibBufIdx+i-1)%VIBGRAPH_N;
      int i2=(vibBufIdx+i)%VIBGRAPH_N;
      float v1,v2;
      if(ch[c].axis==0){v1=ch[c].buf[i1]-dcX;v2=ch[c].buf[i2]-dcX;}
      else if(ch[c].axis==1){v1=ch[c].buf[i1]-dcY;v2=ch[c].buf[i2]-dcY;}
      else{v1=ch[c].buf[i1]-dcZ;v2=ch[c].buf[i2]-dcZ;}

      int px1=(i-1)*(SPR_W-1)/(VIBGRAPH_N-1);
      int px2=i*(SPR_W-1)/(VIBGRAPH_N-1);
      int py1=midY-(int)constrain(v1*(SPR_H/2-4)/sc,-(float)(SPR_H/2-4),(float)(SPR_H/2-4));
      int py2=midY-(int)constrain(v2*(SPR_H/2-4)/sc,-(float)(SPR_H/2-4),(float)(SPR_H/2-4));
      py1=constrain(py1,1,SPR_H-2);
      py2=constrain(py2,1,SPR_H-2);

      spr.drawLine(px1,py1,px2,py2,lerpColor(ch[c].col,METAL_WHITE,40));
      spr.drawLine(px1,py1+1,px2,py2+1,ch[c].col);
    }
    int last=(vibBufIdx+VIBGRAPH_N-1)%VIBGRAPH_N;
    float hv=(ch[c].axis==0)?(ch[c].buf[last]-dcX):
             ((ch[c].axis==1)?(ch[c].buf[last]-dcY):(ch[c].buf[last]-dcZ));
    int hpy=midY-(int)constrain(hv*(SPR_H/2-4)/sc,-(float)(SPR_H/2-4),(float)(SPR_H/2-4));
    hpy=constrain(hpy,1,SPR_H-2);
    spr.drawPixel(SPR_W-2,hpy,lerpColor(ch[c].col,METAL_WHITE,120));
  }

  if(!peerAlive){
    spr.setTextColor(CORAL_DIM,BG_DEEP);
    spr.drawCentreString("NO SIGNAL",SPR_W/2,SPR_H/2-4,F2);
  }
  spr.pushSprite(GX_PUSH,GY_PUSH);
}

void drawVibHeaderCompact() {
  char b[48];
  tft.fillRect(M+2,58,SW-M*2-4,42,BG_CARD);
  tft.setTextColor(CYAN,BG_CARD);
  snprintf(b,48,"ACC %+.2f %+.2f %+.2fg",imuAccX,imuAccY,imuAccZ);
  tft.drawString(b,M+6,60,F2);
  tft.setTextColor(NGREEN,BG_CARD);
  snprintf(b,48,"GYR %+.1f %+.1f %+.1fd/s",imuGyrX,imuGyrY,imuGyrZ);
  tft.drawString(b,M+6,72,F2);
  tft.setTextColor(zoneColor[(int)vibZone],BG_CARD);
  snprintf(b,48,"ZONE:%s  V:%.2fmm/s",zoneLabel[(int)vibZone],vibVelocRMS);
  tft.drawString(b,M+6,84,F2);
  tft.setTextColor(AMBER,BG_CARD);
  snprintf(b,48,"T:%.1fC  CF:%.1f  f:%.0fHz",imuChipTempC,vibCrestFact,vibFreqEst);
  tft.drawString(b,M+6,96,F2);
}

void drawVibAxisLabels() {
  float sc=max(0.12f,vibPlotScale);
  float lv[3]={sc,0.0f,-sc};
  char buf[12];
  tft.fillRect(0,GY_PUSH-1,GX_PUSH-2,SPR_H+2,BG_DEEP);
  tft.drawFastVLine(GX_PUSH-1,GY_PUSH,SPR_H,METAL_SH);
  for(int i=0;i<3;i++){
    int y=GY_PUSH+(int)((float)i*(SPR_H-1)/2.0f);
    snprintf(buf,12,(i==1)?"0":"%.2f",lv[i]);
    tft.setTextColor(TXT_VDIM,BG_DEEP);
    tft.drawRightString(buf,GX_PUSH-3,y-3,F1);
  }
}

float computePlotScaleFromBuffer() {
  float maxDyn=0.05f;
  for(int i=0;i<VIBGRAPH_N;i++){
    float ax=fabsf(vibBufX[i]-dcX);
    float ay=fabsf(vibBufY[i]-dcY);
    float az=fabsf(vibBufZ[i]-dcZ);
    if(ax>maxDyn) maxDyn=ax;
    if(ay>maxDyn) maxDyn=ay;
    if(az>maxDyn) maxDyn=az;
  }
  float target=constrain(maxDyn*1.15f,0.12f,8.0f);
  vibPlotScale=vibPlotScale*0.70f+target*0.30f;
  vibPlotScale=constrain(vibPlotScale,0.12f,8.0f);
  return vibPlotScale;
}

// ═════════════════════════════════════════════════════════════════════════════
//  VIB MONITOR SCREEN
// ═════════════════════════════════════════════════════════════════════════════
void drawVibration() {
  curScreen=SCR_VIBRATION;
  fillGradientBG();
  drawTopBar("VIBRATION ANALYSIS");
  cardPanel(M,52,SW-M*2,52,PURPLE_V,8);
  tft.setTextColor(PURPLE_HI,BG_CARD_HI);
  tft.drawString("REAL-TIME IMU DATA",M+6,54,F2);
  tft.drawFastHLine(M+6,66,SW-M*2-12,PURPLE_DIM);
  drawVibHeaderCompact();
  tft.fillRect(0,104,SW,6,BG_DEEP);
  tft.fillRoundRect(M,104,42,6,2,CYAN);
  tft.setTextColor(CYAN,BG_DEEP); tft.drawString("X",M+46,104,F1);
  tft.fillRoundRect(M+62,104,42,6,2,NGREEN);
  tft.setTextColor(NGREEN,BG_DEEP); tft.drawString("Y",M+108,104,F1);
  tft.fillRoundRect(M+124,104,42,6,2,CORAL);
  tft.setTextColor(CORAL,BG_DEEP); tft.drawString("Z",M+170,104,F1);
  tft.drawRoundRect(GX_PUSH-1,GY_PUSH-1,SPR_W+2,SPR_H+2,2,METAL_SH);
  drawVibAxisLabels();
  drawVibGraphSprite();
}

// ═════════════════════════════════════════════════════════════════════════════
//  VIB POPUP
// ═════════════════════════════════════════════════════════════════════════════
void drawVibPopup() {
  if(!showVibPopup) return;
  tft.fillScreen(BG_DEEP);
  gradientV(0,0,SW,20,lerpColor(BG_NAVY,PURPLE_DIM,40),BG_DEEP);
  tft.drawFastHLine(0,20,SW,PURPLE_V);
  drawStatusLED(10,10,peerAlive);
  tft.setTextColor(peerAlive?NGREEN:CORAL,lerpColor(BG_NAVY,PURPLE_DIM,40));
  tft.drawString(peerAlive?"LIVE":"OFFLN",20,4,F2);
  tft.setTextColor(PURPLE_HI,lerpColor(BG_NAVY,PURPLE_DIM,40));
  tft.drawCentreString("VIB MONITOR",SW/2,4,F2);
  neonBtn(SW-58,2,54,16,CORAL,4);
  tft.setTextColor(TXT_PRI,lerpColor(BG_RAISED,CORAL,40));
  tft.drawCentreString("CLOSE",SW-31,6,F2);

  char buf[80];
  gradientV(0,21,SW,14,lerpColor(BG_DEEP,BG_CARD,80),BG_DEEP);
  tft.setTextColor(CYAN_HI,BG_DEEP);
  tft.drawString("ACC",4,22,F2);
  snprintf(buf,60,"%+.1f %+.1f %+.1fg",imuAccX,imuAccY,imuAccZ);
  tft.setTextColor(peerAlive?TXT_PRI:TXT_DIM,BG_DEEP);
  tft.drawString(buf,34,22,F2);
  tft.setTextColor(PURPLE_V,BG_DEEP);
  tft.drawString("GYR",4,34,F2);
  snprintf(buf,60,"%+.0f %+.0f %+.0fd/s",imuGyrX,imuGyrY,imuGyrZ);
  tft.setTextColor(peerAlive?TXT_SEC:TXT_DIM,BG_DEEP);
  tft.drawString(buf,34,34,F2);
  tft.drawFastHLine(0,46,SW,METAL_SH);

  gradientV(0,46,SW,12,BG_NAVY,BG_DEEP);
  snprintf(buf,20,"T:%.0fC",imuChipTempC);
  uint16_t tCol=(imuChipTempC>70)?CORAL:(imuChipTempC>55)?AMBER:NGREEN;
  tft.setTextColor(tCol,BG_NAVY);
  tft.drawString(buf,4,47,F2);
  snprintf(buf,24,"V:%.1fmm/s",vibVelocRMS);
  tft.setTextColor(zoneColor[(int)vibZone],BG_NAVY);
  tft.drawString(buf,60,47,F2);
  snprintf(buf,16,"CF:%.1f",vibCrestFact);
  tft.setTextColor(vibCrestFact>6?CORAL:vibCrestFact>3?AMBER:NGREEN,BG_NAVY);
  tft.drawString(buf,180,47,F2);
  snprintf(buf,16,"%.0fHz",vibFreqEst);
  tft.setTextColor(PURPLE_HI,BG_NAVY);
  tft.drawRightString(buf,SW-4,47,F2);
  tft.drawFastHLine(0,59,SW,METAL_SH);

  uint16_t zc=zoneColor[(int)vibZone];
  tft.fillRoundRect(M,60,56,32,6,lerpColor(BG_DEEP,zc,50));
  tft.drawRoundRect(M,60,56,32,6,zc);
  tft.setTextColor(zc,lerpColor(BG_DEEP,zc,50));
  tft.drawCentreString("ZONE",M+28,62,F2);
  const char* zoneLet2[4]={"A","B","C","D"};
  tft.setTextColor(METAL_WHITE,lerpColor(BG_DEEP,zc,50));
  tft.drawCentreString(zoneLet2[(int)vibZone],M+28,72,F4);

  struct { const char* l; char v[20]; uint16_t c; } mx[6]={
    {"AccRMS","",CYAN},  {"V_RMS","",zc},     {"Min_g","",NGREEN},
    {"Peak",  "",AMBER}, {"Max_g","",CORAL},   {"CF",   "",vibCrestFact>6?CORAL:AMBER},
  };
  snprintf(mx[0].v,20,"%.2fg",vibAccRMS);
  snprintf(mx[1].v,20,"%.1fmm/s",vibVelocRMS);
  snprintf(mx[2].v,20,vibMinDyn<9000?"%.2fg":"---",vibMinDyn<9000?vibMinDyn:0.0f);
  snprintf(mx[3].v,20,"%.2fg",vibPeakDyn);
  snprintf(mx[4].v,20,"%.2fg",vibMaxDyn);
  snprintf(mx[5].v,20,"%.1f",vibCrestFact);

  int cellW=84, startX=70;
  for(int i=0;i<6;i++){
    int ci=i%3, ri=i/3;
    int bx=startX+ci*cellW, by=60+ri*18;
    tft.fillRect(bx,by,cellW-2,17,BG_DEEP);
    tft.setTextColor(mx[i].c,BG_DEEP);
    tft.drawString(mx[i].l,bx+2,by+1,F2);
    tft.setTextColor(TXT_PRI,BG_DEEP);
    tft.drawString(mx[i].v,bx+2,by+10,F2);
  }
  tft.drawFastHLine(0,97,SW,METAL_SH);
  drawVibGraphSprite();
}

// ═════════════════════════════════════════════════════════════════════════════
//  VIB ICON + CAL BUTTON
// ═════════════════════════════════════════════════════════════════════════════
void drawVibIcon() {
  int bx=4,by=SC_Y1+SC_FLOOR_H+2,bw=58,bh=28;
  int cx=bx+bw/2,cy=by+bh/2-4;
  tft.fillRect(bx-2,by-2,bw+4,bh+4,BG_DEEP);
  if(showVibPopup) neonBtnPress(bx,by,bw,bh,PURPLE_V,5);
  else{neonBtn(bx,by,bw,bh,PURPLE_V,5);tft.drawRoundRect(bx-1,by-1,bw+2,bh+2,6,lerpColor(BG_DEEP,PURPLE_V,40));}
  uint16_t wc=showVibPopup?TXT_PRI:PURPLE_HI;
  for(int i=0;i<bw-12;i++){
    float a1=i*(4.0f*3.14159f/(bw-13)), a2=(i+1)*(4.0f*3.14159f/(bw-13));
    int y1=cy-(int)(4.0f*sinf(a1)), y2=cy-(int)(4.0f*sinf(a2));
    tft.drawLine(bx+6+i,y1,bx+7+i,y2,wc);
    tft.drawLine(bx+6+i,y1+1,bx+7+i,y2+1,wc);
  }
  uint16_t lbg=showVibPopup?lerpColor(BG_DEEP,PURPLE_V,60):lerpColor(BG_RAISED,PURPLE_V,40);
  tft.setTextColor(showVibPopup?TXT_PRI:PURPLE_HI,lbg);
  tft.drawCentreString("VIB",cx,by+bh-11,F2);
}

void drawCalBtn(int x,int y,int w,int h) {
  if(calRequestPending){
    neonBtnPress(x,y,w,h,AMBER,5);
    tft.setTextColor(BG_DEEP,lerpColor(BG_DEEP,AMBER,60));
    tft.drawCentreString("CAL..",x+w/2,y+h/2-4,F2);
  } else {
    neonBtn(x,y,w,h,AMBER,5);
    tft.setTextColor(AMBER_HI,lerpColor(BG_RAISED,AMBER,40));
    int icx=x+8,icy=y+h/2;
    tft.fillTriangle(icx-3,icy-5,icx+2,icy,icx-1,icy,AMBER_HI);
    tft.fillTriangle(icx+1,icy,icx-2,icy+5,icx+3,icy,AMBER_HI);
    tft.drawString("MPU CAL",x+18,y+h/2-4,F2);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[DISPLAY] Booting ARS IIT v11.2");
  esp_sleep_enable_ext0_wakeup(TOUCH_WAKEUP_PIN,0);

  tsSPI.begin(XPT2046_CLK,XPT2046_MISO,XPT2046_MOSI,XPT2046_CS);
  ts.begin(tsSPI);
  ts.setRotation(1);

  tft.init();
  tft.setRotation(1);
  tft.setTextWrap(false);
  tft.fillScreen(BG_DEEP);

  loadPrefs(); loadCal();
  drawSplash();

  cyl2={0.0f,0,false,0.7f};
  cyl3={0.0f,0,false,0.7f};
  lastAnimMs=millis();

  for(int i=0;i<VIBGRAPH_N;i++){vibBufX[i]=0;vibBufY[i]=0;vibBufZ[i]=1.0f;}
  dcX=0.0f; dcY=0.0f; dcZ=1.0f;

  if(!calDone){curScreen=SCR_CAL; drawCal();}
  else{curScreen=SCR_HOME; drawHome();}
  lastTouchMs=millis();
  lastGraphMs=millis();
}

// ═════════════════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now=millis();
  if(now-lastTouchMs>SLEEP_TIMEOUT_MS) enterDeepSleep();

  pulsePhase++;
  breathPhase+=2;

  if(calRequestPending && now-calRequestMs>10000) calRequestPending=false;

  if(newDataRx){
    newDataRx=false;
    if(curScreen==SCR_MANUAL){
      for(int i=0;i<4;i++) drawRelayCard(i);
      drawHRBar();
    }
  }

  if(peerAlive && now-lastPeerMs>3000) peerAlive=false;

  if((curScreen==SCR_VIBRATION) && now-lastGraphMs>=GRAPH_FPS_MS){
    lastGraphMs=now;
    if(condUpdatePending){
      condUpdatePending=false;
      drawVibHeaderCompact();
      drawVibAxisLabels();
    }
    drawVibGraphSprite();
  }

  if(curScreen==SCR_VIBRATION && now-lastVibPoll>VIBRATION_POLL_MS){
    lastVibPoll=now;
    espnowRequestVibration();
  }

  if(curScreen==SCR_AUTO && (cyl2.animating||cyl3.animating)){
    float dt=(now-lastAnimMs)*0.001f;
    lastAnimMs=now;
    if(dt>0.001f && dt<0.15f){
      updateCylAnims(dt);
      tft.fillRect(0,SC_Y0-2,SW,SC_Y1-SC_Y0+SC_FLOOR_H+30,BG_DEEP);
      drawAutoScene();
    }
  } else {
    lastAnimMs=now;
  }

  if(ts.tirqTouched() && ts.touched()){
    lastTouchMs=millis();
    TS_Point p=ts.getPoint();
    int tx,ty; mapTouch(p.x,p.y,tx,ty);

    if(activeField!=FLD_NONE){ handleKbT(tx,ty); delay(80); }
    else if(curScreen==SCR_INFO && qrFullscreen){
      if(inR(tx,ty,M,M,74,28)){
        touchStartY=-1; touchStartX=-1; touchPrevY=-1; touchDragging=false;
        qrFullscreen=false; delay(80); drawInfo();
      }
      delay(30);
    }
    else if(curScreen==SCR_INFO){
      bool handled=false;
      if(ty<=52 && tx>=SW-90){
        if(inR(tx,ty,SW-82,14,82,44)){
          handled=true; touchStartY=-1; touchStartX=-1;
          touchPrevY=-1; touchDragging=false; qrFullscreen=false;
          delay(80); goHome();
        }
      }
      if(!handled){
        int psY=QR_CONTENT_Y-infoScrollY;
        int qrImgY=psY+6, qrBtnY=psY+8;
        if(!handled && qrImgY>=44 && qrImgY<SH-8)
          if(inR(tx,ty,QR_IMG_X-8,qrImgY-8,QR_IMG_W+16,QR_IMG_W+16)){
            handled=true; touchStartY=-1; touchStartX=-1;
            touchPrevY=-1; touchDragging=false;
            qrFullscreen=true; delay(80); drawQRFullscreen();
          }
        if(!handled && qrBtnY>=44 && qrBtnY<SH-8)
          if(inR(tx,ty,QR_BTN_X-8,qrBtnY-8,QR_BTN_W+16,QR_BTN_H+16)){
            handled=true; touchStartY=-1; touchStartX=-1;
            touchPrevY=-1; touchDragging=false;
            qrFullscreen=true; delay(80); drawQRFullscreen();
          }
      }
      if(!handled && ty>47){
        if(touchStartY<0){touchStartY=ty;touchStartX=tx;touchPrevY=ty;touchDragging=false;}
        else{
          int totalMoved=abs(touchStartY-ty), stepDelta=touchPrevY-ty;
          if(!touchDragging && totalMoved>INFO_TAP_SLOP) touchDragging=true;
          if(touchDragging && stepDelta!=0){
            infoScrollY=constrain(infoScrollY+stepDelta,0,INFO_MAX_SCROLL);
            drawInfo();
          }
          touchPrevY=ty;
        }
      }
      if(!handled) delay(30);
    }
    else{
      if(!touchDragging) handleTouch(tx,ty);
      touchPrevY=ty; delay(80);
    }
  } else {
    if(curScreen==SCR_INFO && !qrFullscreen && touchStartY>=0 && touchStartX>=0 && !touchDragging){
      int ax=touchStartX, ay=touchStartY;
      if(ay<=52 && ax>=SW-90 && inR(ax,ay,SW-82,14,82,44)){qrFullscreen=false; goHome();}
      else{
        int psY=QR_CONTENT_Y-infoScrollY;
        int qrImgY=psY+6, qrBtnY=psY+8;
        if(qrImgY>=44&&qrImgY<SH-8&&inR(ax,ay,QR_IMG_X-8,qrImgY-8,QR_IMG_W+16,QR_IMG_W+16)){
          qrFullscreen=true; drawQRFullscreen();
        } else if(!qrFullscreen&&qrBtnY>=44&&qrBtnY<SH-8&&inR(ax,ay,QR_BTN_X-8,qrBtnY-8,QR_BTN_W+16,QR_BTN_H+16)){
          qrFullscreen=true; drawQRFullscreen();
        }
      }
    }
    touchStartY=-1; touchStartX=-1; touchPrevY=-1; touchDragging=false;
  }
}

void enterDeepSleep(){
  for(int i=0;i<8;i++){tft.fillRect(0,0,SW,SH,lerpColor(BG_DEEP,0,i*32));delay(30);}
  tft.fillScreen(0); delay(50);
  esp_sleep_enable_ext0_wakeup(TOUCH_WAKEUP_PIN,0);
  esp_deep_sleep_start();
}

// ═════════════════════════════════════════════════════════════════════════════
//  SPLASH SCREEN
// ═════════════════════════════════════════════════════════════════════════════
void drawSplash(){
  tft.fillScreen(BG_DEEP);
  for(int x=0;x<SW;x+=2){tft.drawFastVLine(x,0,SH,lerpColor(BG_DEEP,CYAN_DARK,30));delay(1);}

  cardPanel(M+6,6,SW-M*2-12,72,CYAN,12);
  screwHead2(M+16,16,3); screwHead2(SW-M-16,16,3);
  screwHead2(M+16,68,3); screwHead2(SW-M-16,68,3);

  tft.setTextColor(CYAN_DARK,BG_CARD_HI); tft.drawCentreString("ARS",110,14,F4);
  tft.setTextColor(CYAN,BG_CARD_HI);      tft.drawCentreString("ARS",109,13,F4);
  tft.drawFastVLine(138,13,30,CYAN_DIM);
  tft.setTextColor(lerpColor(AMBER,AMBER_HI,180),BG_CARD_HI); tft.drawCentreString("IIT",210,14,F4);
  tft.setTextColor(AMBER,BG_CARD_HI);     tft.drawCentreString("IIT",209,13,F4);

  tft.setTextColor(TXT_SEC,BG_CARD_HI);   tft.drawCentreString("Industrial Robotics & Application",160,44,F2);
  tft.setTextColor(TXT_VDIM,BG_CARD);     tft.drawCentreString("MSBTE Institute Code: 0569",160,68,F1);

  neonBtn(96,82,128,20,CYAN,6);
  tft.setTextColor(CYAN_HI,lerpColor(BG_RAISED,CYAN,40));
  tft.drawCentreString("PROJECT",160,87,F2);

  tft.setTextColor(TXT_DIM,BG_DEEP);
  tft.drawCentreString("Abdul Razzak Kalsekar Polytechnic",160,106,F2);
  tft.setTextColor(PURPLE_HI,BG_DEEP);
  tft.drawCentreString("ESP-NOW | MPU-6500 |",160,120,F2);

  // Init ESP-NOW during splash — channel set inside espnowInit
  espnowInit();
  loadPrefs();
  espnowAddPeer();
  espnowSendCmd(CMD_MPU_START);

  int barX=M+4, barW=SW-M*2-8;
  cardPanel(M+4,136,barW,42,PURPLE_V,4);
  tft.setTextColor(PURPLE_HI,BG_CARD_HI);
  tft.drawCentreString("Searching for Peer...",160,142,F2);
  int cpY=158;
  tft.fillRoundRect(barX+8,cpY,barW-16,10,3,BG_DEEP);
  tft.drawRoundRect(barX+8,cpY,barW-16,10,3,CYAN_DIM);

  unsigned long wStart=millis(); bool found=false;
  while(millis()-wStart < ESPNOW_TIMEOUT_MS){
    unsigned long el=millis()-wStart;
    int prog=(int)((long)(barW-20)*el/ESPNOW_TIMEOUT_MS);
    tft.fillRoundRect(barX+10,cpY+2,prog,6,1,CYAN_MID);
    if((el%400)<30) espnowSendCmd(CMD_PING);
    if(peerAlive && !found){
      found=true;
      tft.fillRect(M+4,136,barW,42,BG_DEEP);
      cardPanel(M+4,136,barW,42,NGREEN,4);
      tft.setTextColor(NGREEN_HI,BG_CARD_HI);
      tft.drawCentreString("PEER CONNECTED",160,152,F2);
      tft.fillRoundRect(barX+8,cpY,barW-16,10,3,BG_DEEP);
      tft.fillRoundRect(barX+10,cpY+2,barW-20,6,1,NGREEN);
      delay(500); break;
    }
    delay(25);
  }
  if(!found){
    tft.fillRect(M+4,136,barW,42,BG_DEEP);
    cardPanel(M+4,136,barW,42,CORAL,4);
    tft.setTextColor(CORAL_HI,BG_CARD_HI);
    tft.drawCentreString("Offline Mode Active",160,152,F2);
    delay(500);
  }

  delay(200);
  for(int i=0;i<3;i++) tft.drawRoundRect(58-i,184-i,204+i*2,24+i*2,10+i,lerpColor(BG_DEEP,NGREEN,20+i*15));
  neonBtn(60,186,200,22,NGREEN,8);
  tft.setTextColor(NGREEN_HI,lerpColor(BG_RAISED,NGREEN,40));
  tft.drawCentreString("SYSTEM READY",160,192,F2);
  delay(600);
}

// ═════════════════════════════════════════════════════════════════════════════
//  HOME SCREEN
// ═════════════════════════════════════════════════════════════════════════════
void drawHome(){
  curScreen=SCR_HOME;
  screenWipe(); fillGradientBG();

  gradientV(0,0,SW,24,lerpColor(BG_NAVY,CYAN_DARK,30),BG_DEEP);
  tft.drawFastHLine(0,24,SW,CYAN_DIM);
  drawStatusLED(14,12,peerAlive);
  tft.setTextColor(peerAlive?NGREEN:CORAL,lerpColor(BG_NAVY,CYAN_DARK,30));
  tft.drawString(peerAlive?"LIVE":"OFFLINE",26,5,F2);
  tft.setTextColor(CYAN,lerpColor(BG_NAVY,CYAN_DARK,30));
  tft.drawRightString("ARS | IIT ",SW-8,5,F2);

  int tw=148,th=100,gap=8;
  int col0=M+2, col1=M+tw+gap+2;
  int row0=30,  row1=row0+th+gap;

  struct{int x,y;uint16_t acc;const char*title,*sub;int icon;}tiles[4]={
    {col0,row0,CYAN,  "CONTROL",   "Manual & Auto",  0},
    {col1,row0,CORAL, "INFO",      "Help",            1},
    {col0,row1,NGREEN,"VIBRATION", "Graph + MPU Cal", 2},
    {col1,row1,AMBER, "CALIBRATE", "Touch Mapping",   3},
  };

  for(int i=0;i<4;i++){
    int tx2=tiles[i].x, ty2=tiles[i].y;
    cardPanel(tx2,ty2,tw,th,tiles[i].acc,10);
    int icx=tx2+tw/2, icy=ty2+40;
    for(int r=24;r>18;r--) tft.drawCircle(icx,icy,r,lerpColor(BG_CARD,tiles[i].acc,(24-r)*15));
    tft.fillCircle(icx,icy,18,BG_DEEP);
    tft.drawCircle(icx,icy,18,lerpColor(tiles[i].acc,TXT_PRI,60));
    tft.drawCircle(icx,icy,17,tiles[i].acc);
    switch(tiles[i].icon){
      case 0: drawGearIcon(icx,icy,11,tiles[i].acc);   break;
      case 1: drawInfoIcon(icx,icy,11,tiles[i].acc);   break;
      case 2: drawWifiIcon(icx,icy,11,tiles[i].acc);   break;
      case 3: drawTargetIcon(icx,icy,11,tiles[i].acc); break;
    }
    tft.drawFastHLine(tx2+10,ty2+66,tw-20,lerpColor(METAL_SH,tiles[i].acc,60));
    tft.setTextColor(TXT_PRI,BG_CARD); tft.drawCentreString(tiles[i].title,icx,ty2+72,F2);
    tft.setTextColor(TXT_DIM,BG_CARD); tft.drawCentreString(tiles[i].sub,  icx,ty2+86,F2);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  CONTROL SCREEN
// ═════════════════════════════════════════════════════════════════════════════
void drawControl(){
  curScreen=SCR_CONTROL; screenWipe(); fillGradientBG(); drawTopBar("CONTROL MODE");
  tft.setTextColor(TXT_DIM,BG_DEEP); tft.drawCentreString("Select operation mode",160,54,F2);

  struct{int x;uint16_t acc;const char*title,*l1,*l2;int icon;}modes[2]={
    {M,   CYAN, "MANUAL",   "Toggle relays","ESP-NOW cmds",0},
    {164, CORAL,"AUTOMATIC","Cylinder anim","+ IMU Vibr",  1},
  };
  for(int i=0;i<2;i++){
    int bx=modes[i].x, by=66, bw=150, bh=164;
    cardPanel(bx,by,bw,bh,modes[i].acc,12);
    int icx=bx+bw/2, icy=by+56;
    for(int r=26;r>20;r--) tft.drawCircle(icx,icy,r,lerpColor(BG_CARD,modes[i].acc,(26-r)*12));
    tft.fillCircle(icx,icy,20,BG_DEEP); tft.drawCircle(icx,icy,20,modes[i].acc);
    if(i==0) drawHandIcon(icx,icy,10,modes[i].acc); else drawPlayIcon(icx,icy,8,modes[i].acc);
    tft.drawFastHLine(bx+12,by+86,bw-24,lerpColor(METAL_SH,modes[i].acc,60));
    tft.setTextColor(modes[i].acc,BG_CARD); tft.drawCentreString(modes[i].title,icx,by+92,F2);
    tft.setTextColor(TXT_SEC,BG_CARD);
    tft.drawCentreString(modes[i].l1,icx,by+110,F2);
    tft.drawCentreString(modes[i].l2,icx,by+124,F2);
    neonBtn(bx+16,by+140,bw-32,22,modes[i].acc,6);
    tft.setTextColor(lerpColor(modes[i].acc,TXT_PRI,80),lerpColor(BG_RAISED,modes[i].acc,40));
    tft.drawCentreString("OPEN >",icx,by+146,F2);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  MANUAL SCREEN
// ═════════════════════════════════════════════════════════════════════════════
void drawManual(){
  curScreen=SCR_MANUAL; screenWipe(); fillGradientBG(); drawTopBar("MANUAL CONTROL");
  gradientV(0,48,SW,14,BG_DARK,BG_DEEP);
  tft.setTextColor(TXT_DIM,BG_DARK);
  tft.drawCentreString("Tap relay to toggle via ESP-NOW",160,51,F2);
  for(int i=0;i<4;i++) drawRelayCard(i);
  drawHRBar();
}

void drawRelayCard(int i){
  int col=i%2, row=i/2, bx=M+col*156, by=64+row*80, bw=150, bh=74;
  bool on=coil[i];
  if(on){
    tft.fillRoundRect(bx-1,by-1,bw+2,bh+2,11,lerpColor(BG_DEEP,NGREEN_DIM,40));
    gradientV(bx,by,bw,bh,lerpColor(BG_CARD_HI,NGREEN_DIM,40),BG_CARD);
    tft.drawRoundRect(bx,by,bw,bh,10,NGREEN);
    tft.fillRoundRect(bx+2,by+1,bw-4,3,1,NGREEN);
  } else { cardPanel(bx,by,bw,bh,0,10); }

  int ledX=bx+bw-18, ledY=by+16;
  glowDot(ledX,ledY,6,on?NGREEN:CORAL_DIM);
  uint16_t bgCard=on?lerpColor(BG_CARD_HI,NGREEN_DIM,40):BG_CARD_HI;
  tft.setTextColor(on?NGREEN_HI:TXT_PRI,bgCard);
  tft.drawString(coilLabel[i],bx+10,by+8,F2);
  tft.setTextColor(TXT_DIM,bgCard);
  tft.drawString(coilGPIO[i],bx+10,by+22,F2);
  tft.drawFastHLine(bx+8,by+38,bw-16,on?NGREEN_DIM:METAL_SH);
  if(on){
    neonBtnPress(bx+10,by+44,52,22,NGREEN,5);
    tft.setTextColor(BG_DEEP,lerpColor(BG_DEEP,NGREEN,60));
    tft.drawCentreString("ON",bx+36,by+51,F2);
  } else {
    neonBtn(bx+10,by+44,52,22,METAL_SH,5);
    tft.setTextColor(TXT_DIM,lerpColor(BG_RAISED,METAL_SH,40));
    tft.drawCentreString("OFF",bx+36,by+51,F2);
  }
}

void drawHRBar(){
  gradientV(0,SH-18,SW,18,BG_DARK,BG_DEEP);
  tft.drawFastHLine(0,SH-18,SW,METAL_SH);
  tft.setTextColor(TXT_DIM,BG_DARK); tft.drawString("HR[0]:",M,SH-14,F2);
  char buf[28]; snprintf(buf,28,"%d (0x%04X)",holdReg0,(uint16_t)holdReg0);
  tft.setTextColor(CYAN,BG_DARK); tft.drawString(buf,48,SH-14,F2);
  uint16_t badge=peerAlive?NGREEN:CORAL;
  neonBtn(SW-74,SH-16,68,14,badge,4);
  tft.setTextColor(lerpColor(badge,TXT_PRI,80),lerpColor(BG_RAISED,badge,40));
  tft.drawCentreString(peerAlive?"PEER OK":"NO PEER",SW-40,SH-14,F2);
}

// ═════════════════════════════════════════════════════════════════════════════
//  AUTO SCENE
// ═════════════════════════════════════════════════════════════════════════════
void updateCylAnims(float dt){
  if(cyl2.animating){
    float d=(float)cyl2.target-cyl2.pos;
    if(fabsf(d)<0.02f){cyl2.pos=(float)cyl2.target;cyl2.animating=false;}
    else{cyl2.pos+=(d>0?cyl2.speed:-cyl2.speed)*dt;cyl2.pos=constrain(cyl2.pos,0.0f,1.0f);}
  }
  if(cyl3.animating){
    float d=(float)cyl3.target-cyl3.pos;
    if(fabsf(d)<0.02f){cyl3.pos=(float)cyl3.target;cyl3.animating=false;}
    else{cyl3.pos+=(d>0?cyl3.speed:-cyl3.speed)*dt;cyl3.pos=constrain(cyl3.pos,0.0f,1.0f);}
  }
}

void drawAutoScene(){
  gradientV(0,SC_Y1,SW,SC_FLOOR_H,METAL_LOW,METAL_DARK);
  tft.drawFastHLine(0,SC_Y1,SW,METAL_SPEC);
  for(int xi=0;xi<SW;xi+=14) tft.drawLine(xi,SC_Y1+2,xi-10,SC_Y1+SC_FLOOR_H-1,METAL_SH);

  metalBar2(STAND_X1,RAIL_Y,SW-M-STAND_X1,RAIL_H,METAL_MID,true);
  for(int xi=STAND_X1+14;xi<SW-M-8;xi+=22) screwHead2(xi,RAIL_Y+RAIL_H/2,2);

  metalBar2(STAND_X0-4,SC_Y1-4,STAND_W+8,4,METAL_LOW,true);
  gradientH(STAND_X0,RAIL_Y,STAND_W,SC_Y1-RAIL_Y,METAL_MID,METAL_SH);
  tft.fillRect(STAND_X0+1,RAIL_Y+4,3,SC_Y1-RAIL_Y-8,METAL_HI);
  tft.fillRect(STAND_X1-3,RAIL_Y+4,2,SC_Y1-RAIL_Y-8,METAL_DARK);
  gradientV(STAND_X0-3,RAIL_Y,STAND_W+6,8,METAL_HI,METAL_MID);
  tft.fillRoundRect(STAND_X0-1,RAIL_Y+50,18,16,3,BG_CARD);
  tft.drawRoundRect(STAND_X0-1,RAIL_Y+50,18,16,3,CYAN_DIM);
  tft.setTextColor(CYAN,BG_CARD); tft.drawCentreString("1",STAND_X0+8,RAIL_Y+54,F2);
  screwHead2(STAND_X0+8,SC_Y1-12,2); screwHead2(STAND_X0+8,SC_Y1-28,2);

  int c2tipX=getCyl2TipX(), rodLen=c2tipX-C2_BX1;
  int c3bx0=getCyl3BodyX0(), c3tipY=getCyl3TipY(), c3rodLen=c3tipY-C3_BODY_Y1;

  metalCylinder2(C2_BX0,C2_BY0,C2_BW,C2_BH,true,CYAN);
  tft.fillRoundRect(C2_BX0+4,C2_BY0+6,20,14,3,lerpColor(BG_DEEP,CYAN,40));
  tft.setTextColor(CYAN_HI,lerpColor(BG_DEEP,CYAN,40));
  tft.drawCentreString("2",C2_BX0+14,C2_BY0+9,F2);
  if(rodLen>0){
    rodChrome2(C2_BX1,C2_ROD_Y0,rodLen,true,C2_ROD_H);
    if(cyl2.animating){
      uint16_t ac=(cyl2.target==1)?NGREEN:CORAL;
      int midX=C2_BX1+rodLen/2, dir=(cyl2.target==1)?1:-1;
      for(int ai=-1;ai<=1;ai++){
        int ax2=midX+ai*8*dir;
        if(ax2>C2_BX1+2&&ax2<c2tipX-2) tft.drawFastVLine(ax2,C2_BY0-12,6,ac);
      }
      tft.fillTriangle(midX+10*dir,C2_BY0-14,midX+10*dir,C2_BY0-8,midX+16*dir,C2_BY0-11,ac);
    }
  }
  if(rodLen>=0) pistonHead2(c2tipX-8,C2_BY0,10,C2_BH,true);
  {
    char s[14]; int pct=(int)(cyl2.pos*100);
    if(!cyl2.animating) snprintf(s,14,cyl2.target?"EXT%d%%":"RET%d%%",pct);
    else snprintf(s,14,"MOV%d%%",pct);
    tft.fillRoundRect(C2_BX0,C2_BY1+3,62,14,3,BG_DEEP);
    tft.drawRoundRect(C2_BX0,C2_BY1+3,62,14,3,CYAN_DIM);
    tft.setTextColor(CYAN,BG_DEEP); tft.drawString(s,C2_BX0+4,C2_BY1+4,F2);
  }

  gradientV(c2tipX-5,C3_BODY_Y1-8,10,8,METAL_HI,METAL_SH);
  metalBar2(c2tipX-7,C3_BODY_Y1-10,14,5,METAL_HI,true);
  screwHead2(c2tipX,C3_BODY_Y1-6,2);

  metalCylinder2(c3bx0,C3_BODY_Y0,C3_BW,C3_BH,false,CORAL);
  tft.fillRoundRect(c3bx0+2,C3_BODY_Y0+3,16,13,3,lerpColor(BG_DEEP,CORAL,40));
  tft.setTextColor(CORAL_HI,lerpColor(BG_DEEP,CORAL,40));
  tft.drawCentreString("3",c3bx0+10,C3_BODY_Y0+6,F2);
  if(c3rodLen>0){
    int rodX=c3bx0+(C3_BW-C3_ROD_W)/2;
    int safeLen=min(c3rodLen,SC_Y1-C3_BODY_Y1-12);
    if(safeLen>0) rodChrome2(rodX,C3_BODY_Y1,safeLen,false,C3_ROD_W);
    if(cyl3.animating){
      uint16_t ac=(cyl3.target==1)?NGREEN:CORAL;
      int midY=C3_BODY_Y1+safeLen/2, dir=(cyl3.target==1)?1:-1;
      for(int ai=-1;ai<=1;ai++){
        int ay=midY+ai*6*dir;
        if(ay>C3_BODY_Y1+2&&ay<c3tipY-2) tft.drawFastHLine(c3bx0-14,ay,6,ac);
      }
      tft.fillTriangle(c3bx0-15,midY+8*dir,c3bx0-9,midY+8*dir,c3bx0-12,midY+14*dir,ac);
    }
  }
  int pistonY=min(c3tipY-8,SC_Y1-18);
  pistonHead2(c3bx0,pistonY,C3_BW,10,false);
  metalBar2(c3bx0-5,pistonY+10,C3_BW+10,6,METAL_HI,true);
  screwHead2(c3bx0,pistonY+13,2); screwHead2(c3bx0+C3_BW,pistonY+13,2);
  {
    char s[14]; int pct=(int)(cyl3.pos*100);
    if(!cyl3.animating) snprintf(s,14,cyl3.target?"EXT%d%%":"RET%d%%",pct);
    else snprintf(s,14,"MOV%d%%",pct);
    int lx=SW-76, ly=SC_Y1+SC_FLOOR_H+4;
    tft.fillRoundRect(lx,ly,70,14,3,BG_DEEP);
    tft.drawRoundRect(lx,ly,70,14,3,CORAL_DIM);
    tft.setTextColor(CORAL,BG_DEEP); tft.drawString(s,lx+4,ly+1,F2);
  }

  uint16_t zc=zoneColor[(int)vibZone];
  int zx=6, zy=SC_Y1+SC_FLOOR_H+2, zw=SW-zx-6, zh=28;
  tft.fillRoundRect(zx,zy,zw,zh,5,lerpColor(BG_DEEP,zc,20));
  tft.drawRoundRect(zx,zy,zw,zh,5,zc);
  tft.setTextColor(zc,lerpColor(BG_DEEP,zc,20));
  char cbuf[22];
  snprintf(cbuf,22,"Z:%s V:%.1f",zoneLabel[(int)vibZone],vibVelocRMS);
  tft.drawString(cbuf,zx+4,zy+3,F2);
  snprintf(cbuf,22,"T:%.0fC CF:%.1f",imuChipTempC,vibCrestFact);
  tft.setTextColor(TXT_DIM,lerpColor(BG_DEEP,zc,20));
  tft.drawString(cbuf,zx+4,zy+15,F2);
}

void drawAuto(){
  curScreen=SCR_AUTO;
  screenWipe(); fillGradientBG(); drawTopBar("AUTO CONTROL"); drawAutoScene();
}

// ═════════════════════════════════════════════════════════════════════════════
//  INFO SCREEN
// ═════════════════════════════════════════════════════════════════════════════
void drawInfo(){
  curScreen=SCR_INFO;
  if(qrFullscreen){drawQRFullscreen();return;}
  fillGradientBG(); drawTopBar("ABOUT PROJECT");

  int S=infoScrollY;
  auto sy=[&](int y)->int{return y-S;};
  auto vis=[](int y)->bool{return(y>=48&&y<SH-8);};

  {int sy2=sy(50);if(vis(sy2)){
    gradientV(0,sy2,SW,26,lerpColor(BG_DARK,CYAN_DARK,30),BG_DEEP);
    tft.drawFastHLine(0,sy2,SW,CYAN_DIM);
    tft.setTextColor(CYAN_HI,lerpColor(BG_DARK,CYAN_DARK,30));
    tft.drawString("ARS",12,sy2+5,F4);
    tft.setTextColor(CYAN_DIM,lerpColor(BG_DARK,CYAN_DARK,30));
    tft.drawFastVLine(62,sy2+4,20,CYAN_DIM);
    tft.setTextColor(AMBER,lerpColor(BG_DARK,CYAN_DARK,30));
    tft.drawString("IIT",74,sy2+5,F4);
    tft.setTextColor(TXT_SEC,lerpColor(BG_DARK,CYAN_DARK,30));
    tft.drawString("Industrial Internet of Things",118,sy2+9,F2);
  }}
  {int sy2=sy(84);if(vis(sy2)){
    tft.fillRect(0,sy2,SW,16,BG_DARK);
    tft.setTextColor(TXT_PRI,BG_DARK);
    tft.drawCentreString("Abdul Razzak Kalsekar Polytechnic, Panvel",160,sy2+3,F2);
  }}
  {int sy2=sy(102);if(vis(sy2)){
    tft.fillRect(0,sy2,SW,16,BG_DEEP);
    tft.setTextColor(TXT_DIM,BG_DEEP);
    tft.drawCentreString("Navi Mumbai | 2023-26 | 0569",160,sy2+3,F2);
  }}
  {int sy2=sy(128);if(vis(sy2)){
    gradientV(0,sy2,SW,20,AMBER_DIM,BG_DEEP);
    tft.drawFastHLine(0,sy2,SW,AMBER);
    tft.setTextColor(AMBER_HI,AMBER_DIM);
    tft.drawCentreString("LEARNING OBJECTIVES",160,sy2+5,F2);
  }}
  struct{const char*t;uint16_t c;}llos[3]={
    {"Vibration & Temp Monitor",CYAN},
    {"ESP-NOW Data Protocol",NGREEN},
    {"Industrial IoT Control",CORAL},
  };
  for(int i=0;i<3;i++){
    int vy=152+i*22, iy2=sy(vy);
    if(iy2+18<=48||iy2>=SH-8) continue;
    uint16_t rb=(i%2==0)?BG_DARK:BG_NAVY;
    tft.fillRect(0,iy2,SW,18,rb);
    tft.fillRect(0,iy2,4,18,llos[i].c);
    tft.setTextColor(llos[i].c,rb);
    tft.drawString(llos[i].t,10,iy2+4,F2);
  }
  {int sy2=sy(222);if(vis(sy2)) tft.drawFastHLine(M,sy2,SW-M*2,METAL_SH);}
  {int sy2=sy(232);if(vis(sy2)){
    gradientV(0,sy2,SW,20,CORAL_DIM,BG_DEEP);
    tft.drawFastHLine(0,sy2,SW,CORAL);
    tft.setTextColor(CORAL_HI,CORAL_DIM);
    tft.drawCentreString("TEAM MEMBERS",160,sy2+5,F2);
  }}
  struct{const char*name;const char*role;uint16_t bc;}mbrs[4]={
    {"Gyanesh Maurya","LEADER",  CYAN},
    {"Huzaifa Adi",   "3D DES",  PURPLE_V},
    {"Mohammad Ibaad","HW BUILD",CORAL},
    {"Mohammad Ayan", "ROBOTIC", NGREEN},
  };
  for(int i=0;i<4;i++){
    int vy=256+i*26, iy2=sy(vy);
    if(iy2+22<=48||iy2>=SH-8) continue;
    uint16_t rb=(i%2==0)?BG_DARK:BG_NAVY;
    tft.fillRect(0,iy2,SW,22,rb);
    tft.drawFastHLine(0,iy2+21,SW,METAL_DARK);
    tft.fillRect(0,iy2,4,22,mbrs[i].bc);
    tft.setTextColor(TXT_PRI,rb);
    tft.drawString(mbrs[i].name,10,iy2+6,F2);
    int bw2=min((int)(strlen(mbrs[i].role)*8+14),68);
    int badgeX=SW-bw2-8;
    tft.fillRoundRect(badgeX-1,iy2+2,bw2+2,17,4,lerpColor(BG_DEEP,mbrs[i].bc,30));
    tft.fillRoundRect(badgeX,iy2+3,bw2,15,3,mbrs[i].bc);
    tft.drawRoundRect(badgeX,iy2+3,bw2,15,3,lerpColor(mbrs[i].bc,METAL_WHITE,60));
    tft.setTextColor(BG_DEEP,mbrs[i].bc);
    tft.drawCentreString(mbrs[i].role,badgeX+bw2/2,iy2+5,F2);
  }
  {int sy2=sy(364);if(vis(sy2)) tft.drawFastHLine(M,sy2,SW-M*2,METAL_SH);}
  {int sy2=sy(374);if(vis(sy2)){
    tft.fillRect(0,sy2,SW,16,BG_DEEP);
    tft.setTextColor(TXT_DIM,BG_DEEP);
    tft.drawCentreString("Under guidance of:",160,sy2+3,F2);
  }}
  {int sy2=sy(392);if(vis(sy2)){
    tft.fillRect(0,sy2,SW,16,BG_DARK);
    tft.setTextColor(AMBER_HI,BG_DARK);
    tft.drawCentreString("Shaikh Naema  &  Imran Rajwani",160,sy2+3,F2);
  }}
  {int sy2=sy(416);if(vis(sy2)) tft.drawFastHLine(M,sy2,SW-M*2,METAL_SH);}
  {int sy2=sy(426);if(vis(sy2)){
    gradientV(0,sy2,SW,20,NGREEN_DIM,BG_DEEP);
    tft.drawFastHLine(0,sy2,SW,NGREEN);
    tft.setTextColor(NGREEN_HI,NGREEN_DIM);
    tft.drawCentreString("use github link",160,sy2+5,F2);
  }}
  {int sy2=sy(448);if(vis(sy2)){
    tft.fillRect(0,sy2,SW,16,BG_DEEP);
    tft.setTextColor(TXT_DIM,BG_DEEP);
    tft.drawCentreString("for more info",160,sy2+3,F2);
  }}
  {
    int psY=sy(QR_CONTENT_Y); int pH=80, pW=SW-M*2;
    if(psY<SH-8&&psY+pH>48){
      int clipY=max(48,psY), clipBot=min(SH-8,psY+pH), visH=clipBot-clipY;
      tft.fillRoundRect(M,clipY,pW,visH,6,BG_CARD);
      tft.drawRoundRect(M,clipY,pW,visH,6,CYAN_DIM);
      if(psY>=48&&psY<SH-20){
        drawQRPixelArt(QR_IMG_X,psY+6,2);
        neonBtn(QR_BTN_X,psY+10,QR_BTN_W,QR_BTN_H,CYAN,5);
        tft.setTextColor(CYAN_HI,lerpColor(BG_RAISED,CYAN,40));
        tft.drawCentreString("ZOOM QR",QR_BTN_X+QR_BTN_W/2,psY+16,F2);
        int txtY=psY+42;
        if(txtY<SH-8&&txtY>48){
          tft.setTextColor(TXT_DIM,BG_CARD);
          tft.drawString("github.com/madebygyanesh",QR_BTN_X,txtY,F2);
          tft.drawString("/ARS_IIT",QR_BTN_X,txtY+12,F2);
        }
      }
    }
  }
  int trackH=SH-56, trackY=48;
  tft.fillRect(SW-4,trackY,2,trackH,BG_CARD);
  int thumbH=max(16,trackH*192/INFO_MAX_SCROLL);
  int thumbY=trackY+(int)((long)infoScrollY*(trackH-thumbH)/INFO_MAX_SCROLL);
  tft.fillRect(SW-4,thumbY,2,thumbH,CYAN);
  tft.drawPixel(SW-3,thumbY,CYAN_HI);
  tft.fillRect(0,SH-8,SW,8,BG_DEEP);
  tft.setTextColor(TXT_VDIM,BG_DEEP);
  tft.drawCentreString(infoScrollY<INFO_MAX_SCROLL-20?"swipe up for more":"-- end --",158,SH-7,F2);
}

void drawQRFullscreen(){
  tft.fillScreen(TXT_PRI);
  int scale=7, qSide=29, px2=qSide*scale, ox=(SW-px2)/2, oy=14;
  drawQRPixelArt(ox,oy,scale);
  neonBtn(M,M,74,28,CORAL,7);
  tft.setTextColor(TXT_PRI,lerpColor(BG_RAISED,CORAL,40));
  tft.drawCentreString("X CLOSE",M+37,M+10,F2);
  tft.setTextColor(BG_DEEP,TXT_PRI);
  tft.drawCentreString("github.com/madebygyanesh/ARS_IIT",160,SH-12,F2);
}

// ═════════════════════════════════════════════════════════════════════════════
//  VIBRATION CENTER
// ═════════════════════════════════════════════════════════════════════════════
void drawConn(){
  curScreen=SCR_CONN;
  screenWipe(); fillGradientBG(); drawTopBar("VIBRATION CENTER");

  gradientV(0,48,SW,24,lerpColor(BG_DARK,PURPLE_DIM,40),BG_DEEP);
  tft.drawFastHLine(0,72,SW,METAL_SH);
  tft.setTextColor(TXT_SEC,BG_DARK);
  tft.drawCentreString("Graph monitor and MPU calibration",160,56,F2);

  cardPanel(M,80,SW-M*2,44,CYAN,8);
  tft.setTextColor(CYAN_HI,BG_CARD_HI);
  tft.drawString("LIVE VIBRATION GRAPH",M+10,90,F2);
  tft.setTextColor(TXT_DIM,BG_CARD);
  tft.drawString("Auto-scaling enabled",M+10,104,F2);
  neonBtn(SW-118,88,104,28,CYAN,6);
  tft.setTextColor(CYAN_HI,lerpColor(BG_RAISED,CYAN,40));
  tft.drawCentreString("OPEN",SW-66,97,F2);

  cardPanel(M,132,SW-M*2,44,AMBER,8);
  tft.setTextColor(AMBER_HI,BG_CARD_HI);
  tft.drawString("MPU CALIBRATION",M+10,142,F2);
  tft.setTextColor(TXT_DIM,BG_CARD);
  tft.drawString("Send remote calibration CMD",M+10,156,F2);
  if(calRequestPending) neonBtnPress(SW-118,140,104,28,AMBER,6);
  else neonBtn(SW-118,140,104,28,AMBER,6);
  tft.setTextColor(calRequestPending?BG_DEEP:AMBER_HI,
                   calRequestPending?lerpColor(BG_DEEP,AMBER,60):lerpColor(BG_RAISED,AMBER,40));
  tft.drawCentreString(calRequestPending?"WAIT":"CALIBRATE",SW-66,149,F2);

  uint16_t zc=zoneColor[(int)vibZone];
  cardPanel(M,184,SW-M*2,44,zc,8);
  tft.setTextColor(peerAlive?NGREEN_HI:CORAL_HI,BG_CARD_HI);
  tft.drawString(peerAlive?"ESP-NOW LINK: LIVE":"ESP-NOW LINK: OFFLINE",M+10,194,F2);
  char sbuf[40];
  snprintf(sbuf,40,"ZONE %s  |  V: %.2f mm/s",zoneLabel[(int)vibZone],vibVelocRMS);
  tft.setTextColor(TXT_PRI,BG_CARD);
  tft.drawString(sbuf,M+10,208,F2);
}

// ═════════════════════════════════════════════════════════════════════════════
//  CALIBRATION SCREEN
// ═════════════════════════════════════════════════════════════════════════════
void drawCal(){
  curScreen=SCR_CAL; calStep=0; fillGradientBG();
  gradientV(0,0,SW,42,lerpColor(BG_NAVY,CYAN_DIM,20),BG_DEEP);
  screwHead2(12,10,3); screwHead2(SW-12,10,3);
  screwHead2(12,32,3); screwHead2(SW-12,32,3);
  tft.setTextColor(CYAN_HI,lerpColor(BG_NAVY,CYAN_DIM,20));
  tft.drawCentreString("TOUCH CALIBRATION",160,6,F4);
  tft.setTextColor(TXT_DIM,lerpColor(BG_NAVY,CYAN_DIM,20));
  tft.drawCentreString("Tap each crosshair precisely  (9 pts)",160,28,F2);
  tft.drawFastHLine(0,42,SW,CYAN);
  drawCalPoint();
}

void drawCalPoint(){
  tft.fillRect(0,44,SW,SH-44,BG_DEEP);
  if(calStep>=CAL_N){
    saveCal(); calDone=true;
    cardPanel(M+14,64,SW-M*2-28,110,NGREEN,14);
    neonRect(M+14,64,SW-M*2-28,110,NGREEN,14);
    tft.setTextColor(NGREEN_HI,BG_CARD_HI);
    tft.drawCentreString("CALIBRATION COMPLETE",160,80,F2);
    tft.drawLine(140,110,155,125,NGREEN); tft.drawLine(155,125,180,95,NGREEN);
    tft.drawLine(141,110,156,125,NGREEN); tft.drawLine(156,125,181,95,NGREEN);
    tft.setTextColor(TXT_SEC,BG_CARD);
    tft.drawCentreString("9-point affine fitted",160,134,F2);
    tft.drawCentreString("Touch accuracy optimised",160,148,F2);
    neonBtn(80,182,160,34,CYAN,8);
    tft.setTextColor(CYAN_HI,lerpColor(BG_RAISED,CYAN,40));
    tft.drawCentreString("GO TO HOME",160,194,F2);
    return;
  }
  char pg[24]; snprintf(pg,24,"Point %d of %d",calStep+1,CAL_N);
  tft.setTextColor(TXT_DIM,BG_DEEP); tft.drawCentreString(pg,160,50,F2);
  int barX=M+2, barW=SW-M*2-4;
  tft.fillRoundRect(barX,64,barW,6,2,BG_CARD);
  tft.fillRoundRect(barX,64,barW*calStep/CAL_N,6,2,CYAN);
  int cx=calTX[calStep], cy=calTY[calStep];
  for(int r=28;r>20;r--) tft.drawCircle(cx,cy,r,lerpColor(BG_DEEP,CORAL,(28-r)*8));
  tft.drawCircle(cx,cy,22,lerpColor(CORAL,TXT_PRI,60));
  tft.drawCircle(cx,cy,18,CYAN);
  tft.drawFastHLine(cx-28,cy,56,CORAL); tft.drawFastVLine(cx,cy-28,56,CORAL);
  tft.drawFastHLine(cx-28,cy-1,56,lerpColor(CORAL,BG_DEEP,160));
  tft.drawFastVLine(cx-1,cy-28,56,lerpColor(CORAL,BG_DEEP,160));
  tft.fillCircle(cx,cy,10,BG_DEEP);
  tft.drawCircle(cx,cy,8,CORAL);
  tft.fillCircle(cx,cy,3,METAL_WHITE);
  tft.setTextColor(TXT_DIM,BG_DEEP); tft.drawCentreString("TAP THE CROSSHAIR PRECISELY",160,210,F2);
  tft.setTextColor(TXT_VDIM,BG_DEEP); tft.drawCentreString("Hold still then lift finger",160,224,F2);
}

// ═════════════════════════════════════════════════════════════════════════════
//  KEYBOARD
// ═════════════════════════════════════════════════════════════════════════════
#define KB_KEY_W  29
#define KB_KEY_H  28
#define KB_GAP     2
#define KB_ROW0_Y 60
static const char kbNum[10]={'1','2','3','4','5','6','7','8','9','0'};
static const char kbR1[10] ={'Q','W','E','R','T','Y','U','I','O','P'};
static const char kbR2[10] ={'A','S','D','F','G','H','J','K','L','.'};
static const char kbR3[8]  ={'Z','X','C','V','B','N','M','@'};
static const char kbHexD[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
static const char kbHexS[4]={':','-','_','.'};

int kbKeyX2(int col,int rowLen){
  int tw=rowLen*(KB_KEY_W+KB_GAP)-KB_GAP;
  return (SW-tw)/2+col*(KB_KEY_W+KB_GAP);
}

void drawKbKey(int x,int y,char ch,bool isSpec){
  if(isSpec){
    neonBtnPress(x,y,KB_KEY_W,KB_KEY_H,CORAL,4);
    tft.setTextColor(CORAL_HI,lerpColor(BG_DEEP,CORAL,60));
  } else {
    neonBtn(x,y,KB_KEY_W,KB_KEY_H,CYAN_DIM,4);
    tft.setTextColor(TXT_PRI,lerpColor(BG_RAISED,CYAN_DIM,40));
  }
  char s[2]={ch,0};
  tft.drawCentreString(s,x+KB_KEY_W/2,y+(KB_KEY_H-8)/2+1,F2);
}

void drawKb(InputField field){
  fillGradientBG();
  gradientV(0,0,SW,20,lerpColor(BG_NAVY,CYAN_DIM,20),BG_DEEP);
  tft.drawFastHLine(0,20,SW,CYAN_DIM);
  const char* fn=(field==FLD_MAC)?"PEER MAC":"INPUT";
  tft.setTextColor(TXT_SEC,lerpColor(BG_NAVY,CYAN_DIM,20));
  tft.drawString(fn,M+2,5,F2);
  cardPanel(M,22,SW-M*2,32,CYAN,6);
  neonRect(M,22,SW-M*2,32,CYAN,6);
  tft.setTextColor(TXT_PRI,BG_CARD);
  tft.drawString(kbLen?kbBuf:"type here...",M+6,30,F2);
  if((millis()/500)%2==0){
    int cx2=min(M+6+kbLen*11,SW-M-10);
    tft.drawFastVLine(cx2,26,22,CYAN);
  }
  bool isMac=(field==FLD_MAC);
  if(isMac){
    for(int i=0;i<8;i++) drawKbKey(kbKeyX2(i,8),KB_ROW0_Y,kbHexD[i],false);
    for(int i=0;i<8;i++) drawKbKey(kbKeyX2(i,8),KB_ROW0_Y+KB_KEY_H+KB_GAP,kbHexD[i+8],false);
    int sy2=KB_ROW0_Y+2*(KB_KEY_H+KB_GAP);
    for(int i=0;i<4;i++) drawKbKey(kbKeyX2(i,8),sy2,kbHexS[i],kbHexS[i]==':');
  } else {
    for(int i=0;i<10;i++) drawKbKey(kbKeyX2(i,10),KB_ROW0_Y,kbNum[i],false);
    for(int i=0;i<10;i++) drawKbKey(kbKeyX2(i,10),KB_ROW0_Y+KB_KEY_H+KB_GAP,kbR1[i],false);
    for(int i=0;i<10;i++) drawKbKey(kbKeyX2(i,10),KB_ROW0_Y+2*(KB_KEY_H+KB_GAP),kbR2[i],false);
    for(int i=0;i<8;i++)  drawKbKey(kbKeyX2(i,8),KB_ROW0_Y+3*(KB_KEY_H+KB_GAP),kbR3[i],kbR3[i]=='@');
  }
  int aby=SH-30, spW=120, delW=SW/2-spW/2-6;
  int ox3=SW/2+spW/2+4, ow3=SW-ox3-M;
  neonBtn(M,aby,delW,24,CORAL,6);
  tft.setTextColor(CORAL_HI,lerpColor(BG_RAISED,CORAL,40));
  tft.drawCentreString("DEL",M+delW/2,aby+8,F2);
  neonBtn(SW/2-spW/2,aby,spW,24,METAL_SH,6);
  tft.setTextColor(TXT_DIM,lerpColor(BG_RAISED,METAL_SH,40));
  tft.drawCentreString("SPACE",SW/2,aby+8,F2);
  neonBtn(ox3,aby,ow3,24,NGREEN,6);
  tft.setTextColor(NGREEN_HI,lerpColor(BG_RAISED,NGREEN,40));
  tft.drawCentreString("OK",ox3+ow3/2,aby+8,F2);
}

void openKb(InputField field,const char* init){
  activeField=field;
  kbLen=strlen(init);
  strncpy(kbBuf,init,63);
  kbBuf[63]=0;
  drawKb(field);
}

void handleKbT(int tx,int ty){
  bool isMac=(activeField==FLD_MAC);
  int aby=SH-30, spW=120, delW=SW/2-spW/2-6;
  if(inR(tx,ty,M,aby,delW,24)){if(kbLen>0)kbBuf[--kbLen]=0;drawKb(activeField);return;}
  if(inR(tx,ty,SW/2-spW/2,aby,spW,24)){if(kbLen<63){kbBuf[kbLen++]=' ';kbBuf[kbLen]=0;}drawKb(activeField);return;}
  int ox3=SW/2+spW/2+4, ow3=SW-ox3-M;
  if(inR(tx,ty,ox3,aby,ow3,24)){
    if(activeField==FLD_MAC){
      unsigned int m[6]={};
      if(sscanf(kbBuf,"%02X:%02X:%02X:%02X:%02X:%02X",&m[0],&m[1],&m[2],&m[3],&m[4],&m[5])==6)
        for(int i=0;i<6;i++) peerMAC[i]=(uint8_t)m[i];
      espnowAddPeer();
    }
    activeField=FLD_NONE; drawConn(); return;
  }
  if(isMac){
    for(int i=0;i<8;i++){int kx=kbKeyX2(i,8);if(inR(tx,ty,kx,KB_ROW0_Y,KB_KEY_W,KB_KEY_H)){if(kbLen<63){kbBuf[kbLen++]=kbHexD[i];kbBuf[kbLen]=0;}drawKb(activeField);return;}}
    for(int i=0;i<8;i++){int kx=kbKeyX2(i,8),ky=KB_ROW0_Y+KB_KEY_H+KB_GAP;if(inR(tx,ty,kx,ky,KB_KEY_W,KB_KEY_H)){if(kbLen<63){kbBuf[kbLen++]=kbHexD[i+8];kbBuf[kbLen]=0;}drawKb(activeField);return;}}
    int sy2=KB_ROW0_Y+2*(KB_KEY_H+KB_GAP);
    for(int i=0;i<4;i++){int kx=kbKeyX2(i,8);if(inR(tx,ty,kx,sy2,KB_KEY_W,KB_KEY_H)){if(kbLen<63){kbBuf[kbLen++]=kbHexS[i];kbBuf[kbLen]=0;}drawKb(activeField);return;}}
  } else {
    for(int i=0;i<10;i++){int kx=kbKeyX2(i,10);if(inR(tx,ty,kx,KB_ROW0_Y,KB_KEY_W,KB_KEY_H)){if(kbLen<63){kbBuf[kbLen++]=kbNum[i];kbBuf[kbLen]=0;}drawKb(activeField);return;}}
    for(int i=0;i<10;i++){int kx=kbKeyX2(i,10),ky=KB_ROW0_Y+KB_KEY_H+KB_GAP;if(inR(tx,ty,kx,ky,KB_KEY_W,KB_KEY_H)){if(kbLen<63){kbBuf[kbLen++]=kbR1[i];kbBuf[kbLen]=0;}drawKb(activeField);return;}}
    for(int i=0;i<10;i++){int kx=kbKeyX2(i,10),ky=KB_ROW0_Y+2*(KB_KEY_H+KB_GAP);if(inR(tx,ty,kx,ky,KB_KEY_W,KB_KEY_H)){if(kbLen<63){kbBuf[kbLen++]=kbR2[i];kbBuf[kbLen]=0;}drawKb(activeField);return;}}
    for(int i=0;i<8;i++){int kx=kbKeyX2(i,8),ky=KB_ROW0_Y+3*(KB_KEY_H+KB_GAP);if(inR(tx,ty,kx,ky,KB_KEY_W,KB_KEY_H)){if(kbLen<63){kbBuf[kbLen++]=kbR3[i];kbBuf[kbLen]=0;}drawKb(activeField);return;}}
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  TOUCH DISPATCH
// ═════════════════════════════════════════════════════════════════════════════
bool inR(int tx,int ty,int x,int y,int w,int h){
  return (tx>=x && tx<x+w && ty>=y && ty<y+h);
}

void goHome(){
  activeField=FLD_NONE;
  if(spriteAllocated){spr.deleteSprite();spriteAllocated=false;}
  drawHome();
}

void handleTouch(int tx,int ty){
  switch(curScreen){
    case SCR_HOME:      handleHomeT(tx,ty); break;
    case SCR_CONTROL:   handleCtrlT(tx,ty); break;
    case SCR_MANUAL:    handleManT(tx,ty);  break;
    case SCR_AUTO:      handleAutoT(tx,ty); break;
    case SCR_INFO:      handleInfoT(tx,ty); break;
    case SCR_CONN:      handleConnT(tx,ty); break;
    case SCR_CAL:       handleCalT(tx,ty);  break;
    case SCR_VIBRATION: handleVibT(tx,ty);  break;
  }
}

void handleHomeT(int tx,int ty){
  int tw=148,th=100,gap=8,col0=M+2,col1=M+tw+gap+2,row0=30,row1=row0+th+gap;
  if(inR(tx,ty,col0,row0,tw,th)) drawControl();
  if(inR(tx,ty,col1,row0,tw,th)){infoScrollY=0;drawInfo();}
  if(inR(tx,ty,col0,row1,tw,th)) drawConn();
  if(inR(tx,ty,col1,row1,tw,th)) drawCal();
}

void handleCtrlT(int tx,int ty){
  if(inR(tx,ty,SW-72,26,66,20)){goHome();return;}
  if(inR(tx,ty,M,66,150,164)) drawManual();
  if(inR(tx,ty,164,66,150,164)) drawAuto();
}

void handleManT(int tx,int ty){
  if(inR(tx,ty,SW-72,26,66,20)){drawControl();return;}
  for(int i=0;i<4;i++){
    int col=i%2, row=i/2, bx=M+col*156, by=64+row*80;
    if(inR(tx,ty,bx,by,150,74)){
      coil[i]=!coil[i];
      espnowSendCmd(coil[i]?(CMD_RELAY1_ON+i*2):(CMD_RELAY1_OFF+i*2));
      espnowSendCmd(CMD_RELAY_STATUS);
      drawRelayCard(i); drawHRBar(); return;
    }
  }
}

void handleAutoT(int tx,int ty){
  if(inR(tx,ty,SW-72,26,66,20)){drawControl();return;}
  int c3bx0=getCyl3BodyX0();
  if(inR(tx,ty,c3bx0-10,SC_Y0,C3_BW+20,SC_Y1-SC_Y0)){
    cyl3.target=(cyl3.target==0)?1:0; cyl3.animating=true;
    espnowSendCmd(cyl3.target?CMD_RELAY3_ON:CMD_RELAY3_OFF); return;
  }
  if(inR(tx,ty,C2_BX0,SC_Y0,C2_TIP_EXT-C2_BX0+10,C2_BH+26)){
    cyl2.target=(cyl2.target==0)?1:0; cyl2.animating=true;
    espnowSendCmd(cyl2.target?CMD_RELAY1_ON:CMD_RELAY1_OFF); return;
  }
  if(inR(tx,ty,STAND_X0-4,SC_Y0,STAND_W+8,SC_Y1-SC_Y0)){
    cardPanel(40,90,240,44,CORAL,10); neonRect(40,90,240,44,CORAL,10);
    tft.setTextColor(CORAL_HI,BG_CARD_HI); tft.drawCentreString("STAND (FIXED)",160,98,F2);
    tft.setTextColor(TXT_DIM,BG_CARD); tft.drawCentreString("This element does not move",160,116,F2);
    delay(900); drawAuto(); return;
  }
}

void handleInfoT(int tx,int ty){
  if(qrFullscreen){
    if(inR(tx,ty,M,M,74,28)){qrFullscreen=false;drawInfo();}
    return;
  }
}

void handleConnT(int tx,int ty){
  if(inR(tx,ty,SW-72,26,66,20)){goHome();return;}
  if(inR(tx,ty,SW-118,88,104,28)){vibFromConn=true;drawVibration();return;}
  if(inR(tx,ty,SW-118,140,104,28)){
    if(!calRequestPending){
      calRequestPending=true;
      calRequestMs=millis();
      espnowSendCmd(CMD_MPU_CALIBRATE);
      drawConn();
    }
    return;
  }
}

void handleCalT(int tx,int ty){
  if(calStep>=CAL_N){if(inR(tx,ty,80,182,160,34)) goHome();return;}
  long sumX=0,sumY=0; int got=0;
  for(int s=0;s<8;s++){delay(8);if(ts.touched()){TS_Point p=ts.getPoint();sumX+=p.x;sumY+=p.y;got++;}}
  if(got<3) return;
  calPts[calStep].rx=(uint16_t)(sumX/got);
  calPts[calStep].ry=(uint16_t)(sumY/got);
  calPts[calStep].dx=calTX[calStep];
  calPts[calStep].dy=calTY[calStep];
  glowDot(calTX[calStep],calTY[calStep],14,NGREEN);
  tft.fillCircle(calTX[calStep],calTY[calStep],4,TXT_PRI);
  calStep++; delay(280);
  if(calStep>=CAL_N) computeAffineCalibration();
  drawCalPoint();
}

void handleVibT(int tx,int ty){
  if(inR(tx,ty,SW-72,26,66,20)){
    if(spriteAllocated){spr.deleteSprite();spriteAllocated=false;}
    if(vibFromConn){vibFromConn=false;drawConn();}
    else drawAuto();
    return;
  }
  espnowRequestVibration();
  drawVibration();
}

// ═════════════════════════════════════════════════════════════════════════════
//  9-POINT AFFINE CALIBRATION
// ═════════════════════════════════════════════════════════════════════════════
void computeAffineCalibration(){
  double XtX[3][3]={},XtYx[3]={},XtYy[3]={};
  for(int i=0;i<CAL_N;i++){
    double row[3]={(double)calPts[i].rx,(double)calPts[i].ry,1.0};
    for(int a=0;a<3;a++){
      for(int b=0;b<3;b++) XtX[a][b]+=row[a]*row[b];
      XtYx[a]+=row[a]*calPts[i].dx;
      XtYy[a]+=row[a]*calPts[i].dy;
    }
  }
  double MT[3][5];
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++) MT[i][j]=XtX[i][j];
    MT[i][3]=XtYx[i]; MT[i][4]=XtYy[i];
  }
  for(int col=0;col<3;col++){
    int piv=col;
    for(int r=col+1;r<3;r++) if(fabs(MT[r][col])>fabs(MT[piv][col])) piv=r;
    if(piv!=col) for(int c=0;c<5;c++){double t=MT[col][c];MT[col][c]=MT[piv][c];MT[piv][c]=t;}
    for(int r=col+1;r<3;r++){
      double f=MT[r][col]/MT[col][col];
      for(int c=col;c<5;c++) MT[r][c]-=f*MT[col][c];
    }
  }
  for(int r=2;r>=0;r--){
    MT[r][3]/=MT[r][r]; MT[r][4]/=MT[r][r];
    for(int k=0;k<r;k++){MT[k][3]-=MT[k][r]*MT[r][3];MT[k][4]-=MT[k][r]*MT[r][4];}
  }
  calAff[0]=(float)MT[0][3]; calAff[1]=(float)MT[1][3]; calAff[2]=(float)MT[2][3];
  calAff[3]=(float)MT[0][4]; calAff[4]=(float)MT[1][4]; calAff[5]=(float)MT[2][4];
}

void mapTouch(int rawX,int rawY,int &dx,int &dy){
  if(!calDone){
    dx=constrain(map(rawX,3900,200,0,SW-1),0,SW-1);
    dy=constrain(map(rawY,3800,240,0,SH-1),0,SH-1);
    return;
  }
  dx=constrain((int)(calAff[0]*rawX+calAff[1]*rawY+calAff[2]),0,SW-1);
  dy=constrain((int)(calAff[3]*rawX+calAff[4]*rawY+calAff[5]),0,SH-1);
}

// ═════════════════════════════════════════════════════════════════════════════
//  PREFERENCES
// ═════════════════════════════════════════════════════════════════════════════
void loadPrefs(){
  prefs.begin("ars",true);
  uint8_t pm[6]={0};
  size_t n=prefs.getBytes("pmac",pm,6);
  prefs.end();
  if(n==6){
    bool allZero=true;
    for(int i=0;i<6;i++) if(pm[i]!=0x00){allZero=false;break;}
    if(!allZero) memcpy(peerMAC,pm,6);
  }
}
void savePrefs(){prefs.begin("ars",false);prefs.putBytes("pmac",peerMAC,6);prefs.end();}
void saveCal(){
  prefs.begin("arsc",false);
  prefs.putBool("ok",true);
  for(int i=0;i<6;i++){char k[5];snprintf(k,5,"ca%d",i);prefs.putFloat(k,calAff[i]);}
  prefs.end();
}
void loadCal(){
  prefs.begin("arsc",true);
  calDone=prefs.getBool("ok",false);
  if(calDone){for(int i=0;i<6;i++){char k[5];snprintf(k,5,"ca%d",i);calAff[i]=prefs.getFloat(k,0.0f);}}
  prefs.end();
}
// ═══════════════════════════════ END OF FILE ═══════════════════════════════
