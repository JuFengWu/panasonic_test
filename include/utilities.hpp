#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

extern "C" {
#include "ethercat.h"   // SOEM
}

// ------------------- SDO write wrappers -------------------

inline bool sdo_write_u8(uint16 slave, uint16 index, uint8 sub, uint8 val)
{
    int wkc = ec_SDOwrite(slave, index, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);
    return (wkc > 0);
}

inline bool sdo_write_u16(uint16 slave, uint16 index, uint8 sub, uint16 val)
{
    int wkc = ec_SDOwrite(slave, index, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);
    return (wkc > 0);
}

inline bool sdo_write_i16(uint16 slave, uint16 index, uint8 sub, int16 val)
{
    int wkc = ec_SDOwrite(slave, index, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);
    return (wkc > 0);
}

inline bool sdo_write_i32(uint16 slave, uint16 index, uint8 sub, int32 val)
{
    int wkc = ec_SDOwrite(slave, index, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);
    return (wkc > 0);
}

inline bool sdo_write_u32(uint16 slave, uint16 index, uint8 sub, uint32 val)
{
    int wkc = ec_SDOwrite(slave, index, sub, FALSE, sizeof(val), &val, EC_TIMEOUTRXM);

    if (wkc <= 0)
    {
        printf("SDOwrite failed: %04X:%02X wkc=%d\n", index, sub, wkc);
        if (EcatError)
        {
            printf("SOEM EcatError: %s\n", ec_elist2string());
        }
        return false;
    }
    return true;
}

// ------------------- SDO read wrappers -------------------

inline bool sdo_read_u8(uint16 slave, uint16 index, uint8 sub, uint8 *out)
{
    uint8 val = 0;
    int size = sizeof(val);
    int wkc = ec_SDOread(slave, index, sub, FALSE, &size, &val, EC_TIMEOUTRXM);
    if (wkc <= 0) return false;
    *out = val;
    return true;
}

inline bool sdo_read_u16(uint16 slave, uint16 index, uint8 sub, uint16 *out)
{
    uint16 val = 0;
    int size = sizeof(val);
    int wkc = ec_SDOread(slave, index, sub, FALSE, &size, &val, EC_TIMEOUTRXM);
    if (wkc <= 0) return false;
    *out = val;
    return true;
}

inline bool sdo_read_u32(uint16 slave, uint16 index, uint8 sub, uint32 *out)
{
    uint32 val = 0;
    int size = sizeof(val);
    int wkc = ec_SDOread(slave, index, sub, FALSE, &size, &val, EC_TIMEOUTRXM);
    if (wkc <= 0) return false;
    *out = val;
    return true;
}
