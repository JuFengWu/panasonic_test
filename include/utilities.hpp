#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

extern "C" {
#include "ethercat.h" // SOEM
}

// ------------------- SDO write wrappers -------------------

inline bool sdo_write_u8(uint16 slave, uint16 index, uint8 sub, uint8 val) {
  int wkc = ec_SDOwrite(slave, index, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);
  return (wkc > 0);
}

inline bool sdo_write_u16(uint16 slave, uint16 index, uint8 sub, uint16 val) {
  int wkc = ec_SDOwrite(slave, index, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);
  return (wkc > 0);
}

inline bool sdo_write_i16(uint16 slave, uint16 index, uint8 sub, int16 val) {
  int wkc = ec_SDOwrite(slave, index, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);
  return (wkc > 0);
}

inline bool sdo_write_i32(uint16 slave, uint16 index, uint8 sub, int32 val) {
  int wkc = ec_SDOwrite(slave, index, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);
  return (wkc > 0);
}

inline bool sdo_write_u32(uint16 slave, uint16 index, uint8 sub, uint32 val) {
  int wkc = ec_SDOwrite(slave, index, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);

  if (wkc <= 0) {
    printf("SDOwrite failed: %04X:%02X wkc=%d\n", index, sub, wkc);
    if (EcatError) {
      printf("SOEM EcatError: %s\n", ec_elist2string());
    }
    return false;
  }
  return true;
}

// ------------------- SDO read wrappers -------------------

inline bool sdo_read_u8(uint16 slave, uint16 index, uint8 sub, uint8 *out) {
  uint8 val = 0;
  int size = sizeof(val);
  int wkc = ec_SDOread(slave, index, sub, FALSE, &size, &val, EC_TIMEOUTRXM);
  if (wkc <= 0) {
    return false;
  }
  *out = val;
  return true;
}

inline bool sdo_read_u16(uint16 slave, uint16 index, uint8 sub, uint16 *out) {
  uint16 val = 0;
  int size = sizeof(val);
  int wkc = ec_SDOread(slave, index, sub, FALSE, &size, &val, EC_TIMEOUTRXM);
  if (wkc <= 0) {
    return false;
  }
  *out = val;
  return true;
}

inline bool sdo_read_u32(uint16 slave, uint16 index, uint8 sub, uint32 *out) {
  uint32 val = 0;
  int size = sizeof(val);
  int wkc = ec_SDOread(slave, index, sub, FALSE, &size, &val, EC_TIMEOUTRXM);
  if (wkc <= 0) {
    return false;
  }
  *out = val;
  return true;
}

static inline void set_i16(uint8 *p, int off, int16 v) {
  p[off] = (uint8)(v & 0xFF);
  p[off + 1] = (uint8)((v >> 8) & 0xFF);
}

static inline int16 get_i16(uint8 *p, int off) {
  return (int16)(p[off] | (p[off + 1] << 8));
}

static inline uint16 get_u16(uint8 *p, int off) {
  return (uint16)(p[off] | (p[off + 1] << 8));
}

static inline void set_u16(uint8 *p, int off, uint16 v) {
  p[off] = (uint8)(v & 0xFF);
  p[off + 1] = (uint8)((v >> 8) & 0xFF);
}

static inline int32 get_i32(uint8 *p, int off) {
  return (int32)(p[off] | (p[off + 1] << 8) | (p[off + 2] << 16) |
                 (p[off + 3] << 24));
}

static inline void set_i32(uint8 *p, int off, int32 v) {
  p[off] = (uint8)(v & 0xFF);
  p[off + 1] = (uint8)((v >> 8) & 0xFF);
  p[off + 2] = (uint8)((v >> 16) & 0xFF);
  p[off + 3] = (uint8)((v >> 24) & 0xFF);
}

