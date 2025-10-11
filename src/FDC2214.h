// This is a header file for FDC2214 library
// By Harijs Zablockis, Intelitech, March 2018 
// This file is heavily based on NelsonsLog_FDC2214.h by Chris Nelson 
// Masks and channels added
//
// There is no warranty to the code. There is no support, don't ask for it.
// Use or skip on your own responsibility.
// NOT ALL FEATURES ARE TESTED! 
//
// The code might get revisited at some point or it might not.
// The code does more than I need at the moment.
//
// Feel free to do whatever you want with it. No licensing BS. No limitations.
//

#pragma once


#ifndef _FDC2214_H_
#define _FDC2214_H_

#include <stdint.h>
#include <Wire.h>

enum FDC2214_DEVICE : uint16_t {
  FDC2214_DEVICE_INVALID = 0x0000,
  FDC2214_DEVICE_FDC211x = 0x3054, // FDC2112 or FDC2114
  FDC2214_DEVICE_FDC221x = 0x3055, // FDC2212 or FDC2214
};

// Address is 0x2A (default) or 0x2B (if ADDR is high)
enum FDC2214_I2C_ADDR : uint8_t {
  FDC2214_I2C_ADDR_0 = 0x2A,
  FDC2214_I2C_ADDR_1 = 0x2B,
};

enum FDC2214_DEGLITCH : uint8_t {
  FDC2214_DEGLITCH_1000Khz = 1,
  FDC2214_DEGLITCH_3300Khz = 4,
  FDC2214_DEGLITCH_10Mhz   = 5,
  FDC2214_DEGLITCH_33Mhz   = 6,
};

enum FDC2214_GAIN : uint16_t {
  FDC2214_GAIN_1  = 0,
  FDC2214_GAIN_4  = 1,
  FDC2214_GAIN_8  = 2,
  FDC2214_GAIN_16 = 3,
};

class FDC2214 {
public:
    // For using a different supported I2C interface pass Wire2, Wire3... as second parameter.
    FDC2214(FDC2214_I2C_ADDR i2caddr, typeof(Wire)& wire = Wire);

    FDC2214_DEVICE begin(uint8_t channelMask, bool enableSleepMode = false,
        FDC2214_DEGLITCH deglitchValue = FDC2214_DEGLITCH_33Mhz,
        bool useInternalOscillator = true, FDC2214_GAIN gain = FDC2214_GAIN_1);

    // Deprecated. autoscanSeq has become obsolete. It is automatically calculated from chanMask.
    bool begin(uint8_t channelMask, uint8_t deglitchValue, uint8_t autoscanSeq, bool useInternalOscillator);

    const FDC2214_DEVICE getDevice() const;

    const size_t getChannelCount() const;

    // return true on success: Otherwise false.
    bool setFrequencyDivider(uint8_t channel, uint16_t value);

    // return true on success: Otherwise false.
    bool setOffset(uint8_t channel, uint16_t value);

    // return true, if sleep mode is enabled. Otherwise false.
    bool isSleepModeEnabled()const;

    // return sleep mode before that call.
    bool enableSleepMode();

    // return sleep mode before that call.
    bool disableSleepMode();

// To be used with FDC2112 and FDC2114
    unsigned long getReading16(uint8_t channel, int timeout = 100) const;
// To be used with FDC2212 and FDC2214
    unsigned long getReading28(uint8_t channel, int timeout = 100) const;

private:
    void loadSettings(uint8_t chanMask, bool enableSleepMode, uint8_t deglitchValue, bool useInternalOscillator, FDC2214_GAIN gain);
    void loadChannelSettings(const uint16_t regOffset);
    const FDC2214_DEVICE readDeviceId() const;

    void write8FDC(uint16_t address, uint8_t data);
    uint8_t read8FDC(uint16_t address) const;

    void write16FDC(uint16_t address, uint16_t data);
    uint16_t read16FDC(uint16_t address) const;

    FDC2214_I2C_ADDR _i2caddr;
    typeof(Wire)& _wire;
    mutable FDC2214_DEVICE _device;
};
#endif //include guard

//Added by Sloeber 
#pragma once
