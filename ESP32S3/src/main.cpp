#include <Arduino.h>
#include "esp_camera.h"
#include "SD_MMC.h"
#include "FS.h"
#include "avi_utils.h" 

// ==========================================
// [설정] 파일명 접두사 (슬래시 제외)
// ==========================================
#define FILE_BASE_NAME "Paglider_Video_"
// ==========================================

// ---------------------------
// 1. 통신 핀 설정
// ---------------------------
#define TEENSY_SERIAL Serial1
#define RXD1 44 
#define TXD1 43 

// ---------------------------
// 2. 카메라 핀 설정
// ---------------------------
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// ---------------------------
// 3. SD카드 및 녹화 설정
// ---------------------------
#define SD_MMC_CLK  7
#define SD_MMC_CMD  9
#define SD_MMC_D0   8

#define FRAME_SIZE FRAMESIZE_VGA 
#define JPEG_QUALITY 10          // 호환성을 위해 10~12 추천
#define TARGET_FPS 15            
#define MAX_FRAMES 4000          

// [중요] 호환성을 위한 512바이트 헤더 사용
#define AVI_HEADER_SIZE 512 
#define CHUNK_HEADER_SIZE 8

// =============================================================
// [핵심] 표준 JPEG 허프만 테이블 (DHT) 데이터 - 호환성 해결의 열쇠
// =============================================================
const uint8_t JPGE_DHT[] = {
  0xFF, 0xC4, 0x01, 0xA2, 0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 
  0x0B, 0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x10, 0x00, 
  0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7D, 0x01, 
  0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 
  0x71, 0x14, 0x32, 0x81, 0x91, 0xA1, 0x08, 0x23, 0x42, 0xB1, 0xC1, 0x15, 0x52, 0xD1, 0xF0, 0x24, 
  0x33, 0x62, 0x72, 0x82, 0x09, 0x0A, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x25, 0x26, 0x27, 0x28, 0x29, 
  0x2A, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 
  0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 
  0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 
  0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 
  0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 
  0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2, 0xE3, 
  0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 
  0xFA, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 
  0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 
  0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 
  0x52, 0xF0, 0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16, 0x24, 0x34, 0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19, 
  0x1A, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 
  0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 
  0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 
  0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 
  0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 
  0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 
  0xD9, 0xDA, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 
  0xF7, 0xF8, 0xF9, 0xFA
};

// 전역 변수
File aviFile;
bool isRecording = false;
uint16_t frameCount = 0;
String currentFileName = "";
unsigned long recordStartTime = 0;
uint32_t totalDataSize = 0;

struct FrameInfo {
  uint32_t offset;
  uint32_t size;
};
FrameInfo* frameIndex = NULL;

// 함수 프로토타입
bool initCamera();
bool initSDCard();
void startRecording();
void stopRecording();
void captureFrame();
int getNextFileNumber();

void setup() {
  Serial.begin(9600);
  TEENSY_SERIAL.begin(9600, SERIAL_8N1, RXD1, TXD1);

  delay(3000); // 리셋 후 대기
  
  Serial.println("\n\n=== ESP32S3 AVI Recorder (Universal Compatibility) ===");
  Serial.printf("✓ Target: %s*.avi\n", FILE_BASE_NAME);

  if (!psramFound()) {
    Serial.println("❌ ERROR: PSRAM required!");
    while(1) delay(1000);
  }

  if (!initCamera()) {
    Serial.println("❌ Camera Init Failed!");
    while(1) delay(1000);
  }
  
  delay(1000);

  if (!initSDCard()) {
    Serial.println("❌ SD Card Init Failed!");
    while(1) delay(1000);
  }

  Serial.println("✓ Ready. Waiting for Teensy...");
}

void loop() {
  // Teensy 명령어 처리
  if (TEENSY_SERIAL.available()) {
    char cmd = TEENSY_SERIAL.read();
    while(TEENSY_SERIAL.available()) TEENSY_SERIAL.read();

    Serial.print("Rx: "); Serial.println(cmd);

    if (cmd == '1') {
      if (!isRecording) {
        startRecording();
        if (isRecording) TEENSY_SERIAL.write('1'); 
      } else {
         TEENSY_SERIAL.write('1'); 
      }
    } 
    else if (cmd == '0') {
      if (isRecording) {
        stopRecording();
        TEENSY_SERIAL.write('0'); 
      } else {
        TEENSY_SERIAL.write('0');
      }
    }
  }

  // 녹화 루프
  if (isRecording) {
    captureFrame();
    // FPS 미세 조정 (프레임 저장 시간이 있어서 delay를 적게 줌)
    delay(1000 / TARGET_FPS / 2); 
  } else {
    delay(50);
  }
}

// ==========================================
//           기능 구현부
// ==========================================

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAME_SIZE;
  config.jpeg_quality = JPEG_QUALITY;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&config) != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  s->set_brightness(s, 0); 
  s->set_contrast(s, 0);   
  s->set_saturation(s, 0); 
  s->set_whitebal(s, 1);   
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);    
  return true;
}

bool initSDCard() {
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if (!SD_MMC.begin("/sdcard", true)) return false; 
  if (SD_MMC.cardType() == CARD_NONE) return false;
  return true;
}

int getNextFileNumber() {
  int maxNumber = -1;
  File root = SD_MMC.open("/");
  if (!root) return 0;

  File file = root.openNextFile();
  String baseName = String(FILE_BASE_NAME);

  while (file) {
    if (!file.isDirectory()) {
      String fileName = file.name();
      if (fileName.startsWith("/")) fileName = fileName.substring(1);

      if (fileName.startsWith(baseName) && fileName.endsWith(".avi")) {
        String numStr = fileName.substring(baseName.length());
        if (numStr.length() > 4) numStr = numStr.substring(0, numStr.length() - 4);
        int num = numStr.toInt();
        if (num > maxNumber) maxNumber = num;
      }
    }
    file = root.openNextFile();
  }
  root.close();
  return maxNumber + 1;
}

void startRecording() {
  int fileNumber = getNextFileNumber();
  currentFileName = "/" + String(FILE_BASE_NAME) + String(fileNumber) + ".avi";

  if (frameIndex != NULL) free(frameIndex);
  frameIndex = (FrameInfo*)malloc(MAX_FRAMES * sizeof(FrameInfo));
  if (!frameIndex) {
    Serial.println("❌ Memory Alloc Failed");
    TEENSY_SERIAL.write('0'); 
    return;
  }

  aviFile = SD_MMC.open(currentFileName, FILE_WRITE);
  if (!aviFile) {
    Serial.println("❌ File Create Failed");
    free(frameIndex); frameIndex = NULL;
    TEENSY_SERIAL.write('0');
    return;
  }

  uint8_t emptyHeader[AVI_HEADER_SIZE] = {0};
  aviFile.write(emptyHeader, AVI_HEADER_SIZE);

  frameCount = 0;
  totalDataSize = 0;
  recordStartTime = millis();
  isRecording = true;

  Serial.printf("🔴 Rec Start: %s\n", currentFileName.c_str());
}

void stopRecording() {
  if (!isRecording) return;
  isRecording = false;
  Serial.println("⏹ Rec Stop");

  // idx1 쓰기
  uint8_t idx1Header[8] = {'i', 'd', 'x', '1', 0, 0, 0, 0};
  uint32_t idxSize = frameCount * 16;
  memcpy(idx1Header + 4, &idxSize, 4);
  aviFile.write(idx1Header, 8);

  for (int i = 0; i < frameCount; i++) {
    uint8_t idxEntry[16];
    memcpy(idxEntry, "00dc", 4);
    memcpy(idxEntry + 4, "\x10\x00\x00\x00", 4);
    memcpy(idxEntry + 8, &frameIndex[i].offset, 4);
    memcpy(idxEntry + 12, &frameIndex[i].size, 4);
    aviFile.write(idxEntry, 16);
  }

  // 헤더 업데이트
  uint8_t header[AVI_HEADER_SIZE];
  buildAviHeader(header, TARGET_FPS, getFrameSizeIndex(FRAME_SIZE), frameCount, totalDataSize);
  aviFile.seek(0);
  aviFile.write(header, AVI_HEADER_SIZE);
  aviFile.close();

  if (frameIndex != NULL) { free(frameIndex); frameIndex = NULL; }
  Serial.printf("✓ Saved: %s\n", currentFileName.c_str());
}

// -----------------------------------------------------------
// [핵심] 캡처 시 Huffman Table(DHT) 강제 삽입
// 윈도우/맥 기본 플레이어 재생을 위한 "수술" 코드
// -----------------------------------------------------------
void captureFrame() {
  if (frameCount >= MAX_FRAMES) {
    stopRecording();
    TEENSY_SERIAL.write('0');
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  // JPEG 데이터 크기 보정 (기존 크기 + DHT 크기)
  size_t dhtSize = sizeof(JPGE_DHT);
  size_t finalSize = fb->len + dhtSize;

  // "00dc" + Size + Data(SOI + DHT + 나머지)
  uint8_t marker[4] = {0x30, 0x30, 0x64, 0x63}; 
  uint32_t jpegSize = finalSize;

  frameIndex[frameCount].offset = totalDataSize + 4;
  frameIndex[frameCount].size = jpegSize;

  // 1. AVI Chunk Header 쓰기
  aviFile.write(marker, 4);
  aviFile.write((uint8_t*)&jpegSize, 4);

  // 2. JPEG 데이터 "수술"해서 쓰기
  // 원본: [SOI (2bytes)] [나머지 데이터...]
  // 변경: [SOI (2bytes)] [DHT (400+ bytes)] [나머지 데이터...]
  
  // (1) SOI (Start of Image) - 첫 2바이트
  aviFile.write(fb->buf, 2); 
  
  // (2) DHT (Huffman Table) 삽입
  aviFile.write(JPGE_DHT, dhtSize);

  // (3) 나머지 JPEG 데이터 (SOI 제외한 뒷부분 전부)
  aviFile.write(fb->buf + 2, fb->len - 2);

  // 전체 크기 누적
  totalDataSize += CHUNK_HEADER_SIZE + jpegSize;

  esp_camera_fb_return(fb);
  frameCount++;
}