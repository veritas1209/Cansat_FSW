/*
 * AVI 파일 포맷 유틸리티 (512바이트 정렬 버전)
 * 호환성 개선: JUNK 청크를 추가하여 movi 리스트를 512바이트 경계에 맞춤
 */

#ifndef AVI_UTILS_H
#define AVI_UTILS_H

#include <Arduino.h>

// 프레임 사이즈별 해상도 정보
struct FrameSizeData {
  uint16_t width;
  uint16_t height;
};

const FrameSizeData FRAME_SIZES[] = {
  {96, 96},       // 0
  {160, 120},     // 1
  {176, 144},     // 2
  {240, 176},     // 3
  {240, 240},     // 4
  {320, 240},     // 5
  {400, 296},     // 6
  {480, 320},     // 7
  {640, 480},     // 8
  {800, 600},     // 9
  {1024, 768},    // 10
  {1280, 720},    // 11
  {1280, 1024},   // 12
  {1600, 1200}    // 13
};

// Little-endian 쓰기 유틸리티
inline void write4Bytes(uint8_t* buf, uint32_t value) {
  buf[0] = value & 0xFF;
  buf[1] = (value >> 8) & 0xFF;
  buf[2] = (value >> 16) & 0xFF;
  buf[3] = (value >> 24) & 0xFF;
}

inline void write2Bytes(uint8_t* buf, uint16_t value) {
  buf[0] = value & 0xFF;
  buf[1] = (value >> 8) & 0xFF;
}

// AVI 헤더 생성 (JUNK 청크 포함, 총 512바이트)
void buildAviHeader(uint8_t* header, uint8_t fps, uint8_t frameSizeIdx,
                    uint16_t frameCount, uint32_t totalDataSize) {

  // 512바이트 전체 0으로 초기화
  memset(header, 0, 512);

  uint16_t width = FRAME_SIZES[frameSizeIdx].width;
  uint16_t height = FRAME_SIZES[frameSizeIdx].height;
  uint32_t usPerFrame = 1000000 / fps;

  // 크기 계산
  // 헤더 전체 크기(512) - 8(RIFF+Size) = 504
  // 파일 전체 크기 = 504 + moviSize + idx1Size(나중에 추가됨)
  // 여기서는 단순히 (헤더 + 데이터) 크기만 계산해서 넣음
  uint32_t moviSize = totalDataSize + 4; 
  uint32_t fileSize = 504 + moviSize; 

  int pos = 0;

  // 1. RIFF Header
  memcpy(header + pos, "RIFF", 4); pos += 4;
  write4Bytes(header + pos, fileSize); pos += 4;
  memcpy(header + pos, "AVI ", 4); pos += 4;

  // 2. LIST hdrl
  memcpy(header + pos, "LIST", 4); pos += 4;
  write4Bytes(header + pos, 274); pos += 4; // hdrl 데이터 크기 (변경 없음)
  memcpy(header + pos, "hdrl", 4); pos += 4;

  // avih (56 bytes)
  memcpy(header + pos, "avih", 4); pos += 4;
  write4Bytes(header + pos, 56); pos += 4;
  write4Bytes(header + pos, usPerFrame); pos += 4;
  write4Bytes(header + pos, 0); pos += 4; // MaxBytesPerSec
  write4Bytes(header + pos, 0); pos += 4; // Padding
  write4Bytes(header + pos, 0x10); pos += 4; // Flags: AVIF_HASINDEX
  write4Bytes(header + pos, frameCount); pos += 4;
  write4Bytes(header + pos, 0); pos += 4; // InitialFrames
  write4Bytes(header + pos, 1); pos += 4; // Streams
  write4Bytes(header + pos, 0); pos += 4; // SuggestedBuffer
  write4Bytes(header + pos, width); pos += 4;
  write4Bytes(header + pos, height); pos += 4;
  pos += 16; // Reserved

  // LIST strl (132 bytes)
  memcpy(header + pos, "LIST", 4); pos += 4;
  write4Bytes(header + pos, 132); pos += 4;
  memcpy(header + pos, "strl", 4); pos += 4;

  // strh (56 bytes)
  memcpy(header + pos, "strh", 4); pos += 4;
  write4Bytes(header + pos, 56); pos += 4;
  memcpy(header + pos, "vids", 4); pos += 4;
  memcpy(header + pos, "MJPG", 4); pos += 4;
  write4Bytes(header + pos, 0); pos += 4; // Flags
  write4Bytes(header + pos, 0); pos += 4; // Priority
  write4Bytes(header + pos, 0); pos += 4; // InitialFrames
  write4Bytes(header + pos, 1); pos += 4; // Scale
  write4Bytes(header + pos, fps); pos += 4; // Rate
  write4Bytes(header + pos, 0); pos += 4; // Start
  write4Bytes(header + pos, frameCount); pos += 4;
  write4Bytes(header + pos, 0); pos += 4; // Buffer
  write4Bytes(header + pos, 0xFFFFFFFF); pos += 4; // Quality
  write4Bytes(header + pos, 0); pos += 4; // SampleSize
  write2Bytes(header + pos, 0); pos += 2; // Frame layout
  write2Bytes(header + pos, 0); pos += 2;
  write2Bytes(header + pos, width); pos += 2;
  write2Bytes(header + pos, height); pos += 2;

  // strf (48 bytes: header 8 + content 40)
  memcpy(header + pos, "strf", 4); pos += 4;
  write4Bytes(header + pos, 40); pos += 4;
  write4Bytes(header + pos, 40); pos += 4; // biSize
  write4Bytes(header + pos, width); pos += 4;
  write4Bytes(header + pos, height); pos += 4;
  write2Bytes(header + pos, 1); pos += 2; // Planes
  write2Bytes(header + pos, 24); pos += 2; // BitCount
  memcpy(header + pos, "MJPG", 4); pos += 4; // Compression
  write4Bytes(header + pos, width * height * 3); pos += 4; // SizeImage
  write4Bytes(header + pos, 0); pos += 4; // XPels
  write4Bytes(header + pos, 0); pos += 4; // YPels
  write4Bytes(header + pos, 0); pos += 4; // ClrUsed
  write4Bytes(header + pos, 0); pos += 4; // ClrImportant

  // [중요] JUNK Chunk for Alignment (Padding)
  // 현재까지 위치 계산:
  // RIFF(12) + LIST hdrl(12) + avih(64) + LIST strl(12) + strh(64) + strf(48) = 212 바이트
  // LIST movi 시작 전까지 512 바이트가 되어야 함.
  // 512 - 212 = 300 바이트가 남음.
  // JUNK 헤더(8바이트) + 데이터(292바이트) 추가
  
  uint32_t currentPos = pos;
  uint32_t paddingSize = 512 - 12 - currentPos; // -12는 movi 리스트 헤더 크기
  uint32_t junkDataSize = paddingSize - 8;      // JUNK 태그와 사이즈 제외

  memcpy(header + pos, "JUNK", 4); pos += 4;
  write4Bytes(header + pos, junkDataSize); pos += 4;
  
  // 나머지 0으로 채우기 (이미 memset으로 0임)
  pos += junkDataSize;

  // LIST movi (시작 부분만 작성)
  memcpy(header + pos, "LIST", 4); pos += 4;
  write4Bytes(header + pos, moviSize); pos += 4;
  memcpy(header + pos, "movi", 4); pos += 4;

  // 여기까지 정확히 512 바이트가 됨
}

uint8_t getFrameSizeIndex(framesize_t size) {
  return (uint8_t)size;
}

#endif // AVI_UTILS_H