// This is a source file for FDC2214 library
// By Harijs Zablockis, Intelitech, March 2018 
// This file is based on NelsonsLog_FDC2214.c by Chris Nelson 
//
// The 16 bit output overflow bug is fixed.
// Configuration is made more dynamic and register comments are made readable 
// Added other channels
// Added masking out error bits from data 
// Some junk code and header form original file left for historical value
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

#include "FDC2214.h"

//bitmasks
static constexpr uint16_t FDC2214_CH0_UNREADCONV      = 0x0008;         //denotes unread CH0 reading in STATUS register
static constexpr uint16_t FDC2214_CH1_UNREADCONV      = 0x0004;         //denotes unread CH1 reading in STATUS register
static constexpr uint16_t FDC2214_CH2_UNREADCONV      = 0x0002;         //denotes unread CH2 reading in STATUS register
static constexpr uint16_t FDC2214_CH3_UNREADCONV      = 0x0001;         //denotes unread CH3 reading in STATUS register

static constexpr unsigned FDC2214_OSCILLATOR_SHIFT    = 9;
static constexpr uint16_t FDC2214_OSCILLATOR_MASK     = 0x1 << FDC2214_OSCILLATOR_SHIFT;
static constexpr unsigned FDC2214_SLEEP_SHIFT         = 13;
static constexpr uint16_t FDC2214_SLEEP_MASK          = 0x1 << FDC2214_SLEEP_SHIFT;
static constexpr uint16_t FDC2214_CLOCK_DIVIDER_MASK  = 0x3FF;
static constexpr unsigned FDC2214_SENSOR_FREQ_SHIFT   = 12;
static constexpr uint16_t FDC2214_SENSOR_FREQ_MASK    = 0x3 << FDC2214_SENSOR_FREQ_SHIFT;
static constexpr unsigned FDC2214_GAIN_SHIFT          = 9;
static constexpr uint16_t FDC2214_GAIN_MASK           = 0x3 << FDC2214_GAIN_SHIFT;

//registers
static constexpr uint16_t FDC2214_DEVICE_ID           = 0x7F;
static constexpr uint16_t FDC2214_MUX_CONFIG          = 0x1B;
static constexpr uint16_t FDC2214_CONFIG              = 0x1A;
static constexpr uint16_t FDC2214_RCOUNT_CH0          = 0x08;
static constexpr uint16_t FDC2214_OFFSET_CH0          = 0x0C;
static constexpr uint16_t FDC2214_SETTLECOUNT_CH0     = 0x10;
static constexpr uint16_t FDC2214_CLOCK_DIVIDERS_CH0  = 0x14;
static constexpr uint16_t FDC2214_STATUS              = 0x18;
static constexpr uint16_t FDC2214_DATA_CH0_MSB        = 0x00;
static constexpr uint16_t FDC2214_DATA_CH0_LSB        = 0x01;
static constexpr uint16_t FDC2214_DRIVE_CH0           = 0x1E;
static constexpr uint16_t FDC2214_RESET_DEV           = 0x1C;

// mask for 28bit data to filter out flag bits
static constexpr uint16_t FDC2214_DATA_CHx_MASK_DATA  = 0x0FFF;

static constexpr uint16_t FREQ_DIVIDER_DEFAULT        = 1;
static constexpr uint16_t SENSOR_FREQ_DEFAULT         = 2;

// index into this array is channel mask minus one
static const uint8_t AUTOSCAN_EN[] = {
  0, // msk=0x0001 // SingleChannel Ch0
  0, // msk=0x0010 // SingleChannel Ch1
  1, // msk=0x0011
  0, // msk=0x0100 // SingleChannel Ch2
  1, // msk=0x0101
  1, // msk=0x0110
  1, // msk=0x0111
  0, // msk=0x1000 // SingleChannel Ch3
  1, // msk=0x1001
  1, // msk=0x1010
  1, // msk=0x1011
  1, // msk=0x1100
  1, // msk=0x1101
  1, // msk=0x1110
  1, // msk=0x1111
};

// index into this array is channel mask minus one
static const uint8_t AUTOSCAN [] = {
  0x00,  // msk=0x0001 SingleChannel Ch0
  0x00,  // msk=0x0010 SingleChannel Ch1
  0x00,  // msk=0x0011 Ch0, Ch1
  0x01,  // msk=0x0100 SingleChannel Ch2
  0x01,  // msk=0x0101 Ch0, Ch2 -> Ch0, Ch1, Ch2
  0x01,  // msk=0x0110 Ch1, Ch2 -> Ch0, Ch1, Ch2
  0x01,  // msk=0x0111 Ch0, Ch1, Ch2
  0x10,  // msk=0x1000 SingleChannel Ch3
  0x10,  // msk=0x1001 Ch0, Ch3 -> Ch0, Ch1, Ch2, Ch3
  0x10,  // msk=0x1010 Ch1, Ch3 -> Ch0, Ch1, Ch2, Ch3
  0x10,  // msk=0x1011 Ch0, Ch1, Ch3 -> Ch0, Ch1, Ch2, Ch3
  0x10,  // msk=0x1100 Ch2, Ch3 -> Ch0, Ch1, Ch2, Ch3
  0x10,  // msk=0x1101 Ch0, Ch2, Ch3 -> Ch0, Ch1, Ch2, Ch3
  0x10,  // msk=0x1110 Ch1, Ch2, Ch3 -> Ch0, Ch1, Ch2, Ch3
  0x10,  // msk=0x1111 Ch0, Ch1, Ch2, Ch3 -> Ch0, Ch1, Ch2, Ch3
};

// index into this array is channel
static uint16_t UNREADCONV[] = {
    FDC2214_CH0_UNREADCONV,
    FDC2214_CH1_UNREADCONV,
    FDC2214_CH2_UNREADCONV,
    FDC2214_CH3_UNREADCONV,
};

/**************************************************************************/
/*!
    @file     NelsonsLog_FDC2214.cpp
    @author   Chris Nelson
	@license  BSD (see license.txt)

	This is a library for an FDC2214 capacitive sensor
	----> http://www.nelsonsloxg.com

	@section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/
static uint16_t channelMaskToChannel(uint8_t chanMask_) {
  uint16_t result = 0;
  while(chanMask_ & 1) {
    result++;
    chanMask_ >>= 1;
  }
  return result;
}

FDC2214::FDC2214(FDC2214_I2C_ADDR i2caddr, typeof(Wire)& wire) : _i2caddr(i2caddr), _wire(wire), _device(FDC2214_DEVICE_INVALID) {
}

const FDC2214_DEVICE FDC2214::readDeviceId() const {
  const uint32_t device = read16FDC(FDC2214_DEVICE_ID);
  switch(device) {
    case FDC2214_DEVICE_FDC211x:
    case FDC2214_DEVICE_FDC221x:
      return static_cast<FDC2214_DEVICE>(device);
  }
  return FDC2214_DEVICE_INVALID;
}

const FDC2214_DEVICE FDC2214::getDevice() const {
  if (_device == FDC2214_DEVICE_INVALID) {
    _device = readDeviceId();
  }
  return _device;
}

const size_t FDC2214::getChannelCount() const {
  switch (getDevice()) {
    case FDC2214_DEVICE_FDC211x:
    return 2;
  case FDC2214_DEVICE_FDC221x:
    return 4;
  }
  return 0;
}

bool FDC2214::begin(uint8_t channelMask, uint8_t deglitchValue, uint8_t, bool useInternalOscillator) {
  _wire.begin();
  const FDC2214_DEVICE device = getDevice();
  const bool bOk = (device != FDC2214_DEVICE_INVALID);
  if (bOk) {
    loadSettings(channelMask, false, deglitchValue, useInternalOscillator, FDC2214_GAIN_1);
  }
  return bOk;
}

FDC2214_DEVICE FDC2214::begin(uint8_t chanMask, bool enableSleepMode, FDC2214_DEGLITCH deglitchValue, bool useInternalOscillator, FDC2214_GAIN gain) {
  _wire.begin();
  const FDC2214_DEVICE device = getDevice();
  const bool bOk = (device != FDC2214_DEVICE_INVALID);
  if (bOk) {
    loadSettings(chanMask, enableSleepMode, deglitchValue, useInternalOscillator, gain);
    return device;
  }
  return FDC2214_DEVICE_INVALID;
}

void FDC2214::loadChannelSettings(const uint16_t regOffset) {
  //settle count maximized, slow application
  write16FDC(FDC2214_SETTLECOUNT_CH0 + regOffset, 0x64);
  //rcount maximized for highest accuracy
  write16FDC(FDC2214_RCOUNT_CH0 + regOffset, 0xFFFF);
  //no offset
  write16FDC(FDC2214_OFFSET_CH0 + regOffset, 0x0000);

  // Set clock dividers
  //  Reserved
  //  | Sensor Frequency Select. b01 = /1 = sensor freq 0.01 to 8.75MHz; b10 = /2 = sensor freq 0.01 to 10 or 5 to 10 MHz
  //  | | Reserved
  //  | | |         Reference divider. Must be > 1. fref = fclk / this register`
  //  | | |         |
  // 00FF00RRRRRRRRRR -> 0010000000000001 -> 0x2001
  uint16_t clockRegister = read16FDC(FDC2214_CLOCK_DIVIDERS_CH0 + regOffset);
  clockRegister &= FDC2214_SENSOR_FREQ_MASK;
  clockRegister |= SENSOR_FREQ_DEFAULT << FDC2214_SENSOR_FREQ_SHIFT;

  // Set divider to default, if not set yet.
  if((clockRegister & FDC2214_CLOCK_DIVIDER_MASK) == 0) {
    clockRegister |= FREQ_DIVIDER_DEFAULT;
  }

  write16FDC(FDC2214_CLOCK_DIVIDERS_CH0 + regOffset, clockRegister);

  //set drive register
  write16FDC(FDC2214_DRIVE_CH0 + regOffset, 0xF800);
}

//Internal routine to do actual chip init
void FDC2214::loadSettings(uint8_t chanMask, bool enableSleepMode, uint8_t deglitchValue, bool useInternalOscillator, FDC2214_GAIN gain) {

	//Configuration register
	//	Active channel Select: b00 = ch0; b01 = ch1; b10 = ch2; b11 = ch3;
	//  |Sleep Mode: 0 - device active; 1 - device in sleep;
	//  ||Reserved, reserved, set to 1
	//  |||Sensor Activation Mode: 0 - drive sensor with full current. 1 - drive sensor with current set by DRIVE_CURRENT_CHn
	//  ||||Reserved, set to 1
	//  |||||Reference clock: 0 - use internal; 1 - use external clock
	//  ||||||Reserved, set to 0
	//  |||||||Disable interrupt. 0 - interrupt output on INTB pin; 1 - no interrupt output
	//  ||||||||High current sensor mode: 0 - 1.5mA max. 1 - > 1.5mA, not available if Autoscan is enabled
	//  |||||||||Reserved, set to 000001
	//  ||||||||||
	// CCS1A1R0IH000001 -> 0001 1110 1000 0001 -> 0x1E81 	ExtOsc
	// CCS1A1R0IH000001 -> 0001 1100 1000 0001 -> 0x1C81	IntOsc

  const uint8_t chanMask_(chanMask & 0x0f);
  const uint16_t autoScanEn = AUTOSCAN_EN[chanMask_-1];

  {
    const uint16_t singleScanChan = autoScanEn ? 0 : channelMaskToChannel(chanMask_);
    const uint16_t config = 0x1C81 | singleScanChan |
      (static_cast<uint16_t>(not useInternalOscillator) << FDC2214_OSCILLATOR_SHIFT) |
      (static_cast<uint16_t>(enableSleepMode) << FDC2214_SLEEP_SHIFT);
    write16FDC(FDC2214_CONFIG, config);  //set config
  }

  {
    uint16_t resetDev = read16FDC(FDC2214_RESET_DEV);
    resetDev &= ~FDC2214_GAIN_MASK;
    resetDev |= static_cast<uint16_t>(gain) << FDC2214_GAIN_SHIFT;
    write16FDC(FDC2214_RESET_DEV, resetDev);
  }

  for(size_t c = 0; c < getChannelCount(); c++) {
    //If channel c selected, init it..
    if (chanMask & (0x01 << c)) {
      loadChannelSettings(c);
    }
  }

	// Autoscan: 0 = single channel, selected by CONFIG.ACTIVE_CHAN
	// | Autoscan sequence. b00 for chan 1-2, b01 for chan 1-2-3, b10 for chan 1-2-3-4
	// | |         Reserved - must be b0001000001
	// | |         |  Deglitch frequency. b001 for 1 MHz, b100 for 3.3 MHz, b101 for 10 Mhz, b111 for 33 MHz
	// | |         |  |
  // ARR0001000001DDD -> b0000 0010 0000 1000 -> h0208
	const uint16_t muxVal = 0x0208 | (autoScanEn << 15) | (static_cast<uint16_t>(AUTOSCAN[chanMask_-1]) << 13) | deglitchValue;
	//
  write16FDC(FDC2214_MUX_CONFIG, muxVal);  //set mux config for channels
}

///**************************************************************************/
///*!
//    @brief  Given a reading calculates the sensor frequency
//*/
///**************************************************************************/
//long long NelsonsLog_FDC2214::calculateFsensor(unsigned long reading){
////    Serial.println("reading: "+ String(reading));
//    //fsensor = (CH_FIN_SEL * fref * data) / 2 ^ 28
//    //should be mega hz so can truncate to long long
//    Serial.println("FDC reading: " + String(reading));
//    unsigned long long temp;
//    temp = 1 * 40000000 * reading;
//    temp = temp / (2^28);
////    Serial.println("frequency: " + String((long)temp));
//    return temp;
//}

///**************************************************************************/
///*!
//    @brief  Given sensor frequency calculates capacitance
//*/
///**************************************************************************/
//double NelsonsLog_FDC2214::calculateCapacitance(long long fsensor){
//    //differential configuration
//    //c sensor = 1                            - (Cboard + Cparacitic)
//    //             / (L * (2*pi * fsensor)^2)
//
//    double pi = 3.14159265359;
//    double L = 18; //uH
//    double Cboard = 33; //pf
//    double Cparacitic = 3; //pf
//
//    double temp = 2 * pi * fsensor;
//    temp = temp * temp;
//
//    temp = temp / 1000000; //uH
//    temp *= L;
//
////    Serial.println("capacitance: " + String(temp));
//    return temp;
//
//}


// Gets 28bit reading for FDC2212 and FDC2214
// Takes in channel number, gives out the formatted 28 bit reading.
unsigned long FDC2214::getReading28(uint8_t channel, int timeout) const {
  if(channel < getChannelCount()) {
    const uint8_t addressLSB = FDC2214_DATA_CH0_LSB + 2*channel + 1;
    const uint8_t addressMSB = FDC2214_DATA_CH0_MSB + 2*channel;
    const uint16_t bitUnreadConv = UNREADCONV[channel];

    uint16_t status = read16FDC(FDC2214_STATUS);
    unsigned long reading = 0;

    while (timeout && !(status & bitUnreadConv)) {
          status = read16FDC(FDC2214_STATUS);
          timeout--;
    }

    if (timeout == 100) {
      // #####################################################################################################
      // There was this weird double read, as "first readout could be stale" in Nelsons file.
      // I have not confirmed the existence of this silicon bug.
      // I suspect that it might be due to crappy breadboard or rats nest wiring or lack of signal integrity for other reason
      //
      // On the other hand, I have done far too little testing to be sure, so I am leaving that bit in for now.
      //
      // #####################################################################################################

      //could be stale grab another //could it really it? ?????
      //read the 28 bit result
      reading = static_cast<unsigned long>(read16FDC(addressMSB) & FDC2214_DATA_CHx_MASK_DATA) << 16;
      reading |= read16FDC(addressLSB);
      while (timeout && !(status & bitUnreadConv)) {
          status = read16FDC(FDC2214_STATUS);
          timeout--;
      }
    }
    if (timeout) {
      //read the 28 bit result
      reading = static_cast<unsigned long>(read16FDC(addressMSB) & FDC2214_DATA_CHx_MASK_DATA) << 16;
      reading |= read16FDC(addressLSB);
      return reading;
    } else {
      // Could not get data, chip readynes flag timeout
    }
  }
  return 0;
}

// Gets 16bit reading for FDC2112 and FDC2114
// Takes in channel number, gives out the formatted 28 bit reading.
unsigned long FDC2214::getReading16(uint8_t channel, int timeout) const {
  if(channel < getChannelCount()) {
    const uint8_t addressMSB = FDC2214_DATA_CH0_MSB + 2*channel;
    const uint16_t bitUnreadConv = UNREADCONV[channel];

    uint16_t status = read16FDC(FDC2214_STATUS);
    unsigned long reading = 0;

    while (timeout && !(status & bitUnreadConv)) {
      status = read16FDC(FDC2214_STATUS);
      timeout--;
    }
    if (timeout == 100) {
      // #####################################################################################################
      // There was this weird double read, as "first readout could be stale" in Nelsons file.
      // I have not confirmed the existence of this silicon bug.
      // I suspect that it might be due to crappy breadboard or rats nest wiring or lack of signal integrity for other reason
      //
      // On the other hand, I have done far too little testing to be sure, so I am leaving that bit in for now.
      //
      // #####################################################################################################

      //could be stale grab another //could it really it? ?????
      //read the 28 bit result
      reading = static_cast<unsigned long>((read16FDC(addressMSB) & FDC2214_DATA_CHx_MASK_DATA)) << 16;
      while (timeout && !(status & bitUnreadConv)) {
        status = read16FDC(FDC2214_STATUS);
        timeout--;
      }
    }
    if (timeout) {
      //read the 16 bit result
      reading = static_cast<unsigned long>(read16FDC(addressMSB) & FDC2214_DATA_CHx_MASK_DATA) << 16;
      return reading;
    } else {
      // Could not get data, chip readyness flag timeout
    }
  }
  return 0;
}

bool FDC2214::setOffset(uint8_t channel, uint16_t value) {
  if(channel < getChannelCount()) {
    write16FDC(FDC2214_OFFSET_CH0 + channel, value);
    return true;
  }
  return false;
}

bool FDC2214::setFrequencyDivider(uint8_t channel, uint16_t value) {
  if(channel < 4) {
    if(value > 0 && value <= FDC2214_CLOCK_DIVIDER_MASK) {
      const bool sleepModeAlreadyEnabled = enableSleepMode();
      uint16_t clockDivider = read16FDC(FDC2214_CLOCK_DIVIDERS_CH0 + channel);
      clockDivider &= ~FDC2214_CLOCK_DIVIDER_MASK;
      clockDivider |= value;
      write16FDC(FDC2214_CLOCK_DIVIDERS_CH0 + channel, clockDivider);
      if(not sleepModeAlreadyEnabled) {
        disableSleepMode();
      }
    }
  }
  return false;
}

bool FDC2214::isSleepModeEnabled()const {
  const uint16_t config = read16FDC(FDC2214_CONFIG);
  return (config & FDC2214_SLEEP_MASK) != 0;
}

bool FDC2214::enableSleepMode() {
  uint16_t config = read16FDC(FDC2214_CONFIG);
  if((config & FDC2214_SLEEP_MASK) != 0) {
    // Already enabled.
    return true;
  }
  config |= FDC2214_SLEEP_MASK;
  write16FDC(FDC2214_CONFIG, config);
  return false;
}

bool FDC2214::disableSleepMode() {
  uint16_t config = read16FDC(FDC2214_CONFIG);
  if((config & FDC2214_SLEEP_MASK) == 0) {
    // Already disabled.
    return false;
  }
  config &= ~FDC2214_SLEEP_MASK;
  write16FDC(FDC2214_CONFIG, config);
  return true;
}

///**************************************************************************/
///*!
//    @brief  Takes a reading and calculates capacitance from it
//*/
///**************************************************************************/
//double NelsonsLog_FDC2214::readCapacitance() {
//    int timeout = 100;
//    unsigned long reading = 0;
//    long long fsensor = 0;
//    int status = read16FDC(FDC2214_STATUS_REGADDR);
//    while (timeout && !(status & FDC2214_CH0_UNREADCONV)) {
////        Serial.println("status: " + String(status));
//        status = read16FDC(FDC2214_STATUS_REGADDR);
//        timeout--;
//    }
//    if (timeout) {
//        //read the 28 bit result
//        reading = read16FDC(FDC2214_DATA_CH0_REGADDR) << 16;
//        reading |= read16FDC(FDC2214_DATA_LSB_CH0_REGADDR);
//        fsensor = calculateFsensor(reading);
//        return calculateCapacitance(fsensor);
//    } else {
//        //error not reading
//        Serial.println("error reading fdc");
//        return 0;
//    }
//}

/**************************************************************************/
/*!
    @brief  I2C low level interfacing
*/
/**************************************************************************/


// Read 1 byte from the FDC at 'address'
uint8_t FDC2214::read8FDC(uint16_t address) const {
    _wire.beginTransmission(_i2caddr);
    _wire.write(address >> 8);
    _wire.write(address);
    _wire.endTransmission(false);
    _wire.requestFrom(_i2caddr, (uint8_t) 1);
    const uint8_t r = _wire.read();
    return r;
}

// Read 2 byte from the FDC at 'address'
uint16_t FDC2214::read16FDC(uint16_t address) const {

    _wire.beginTransmission(_i2caddr);
//    _wire.write(address >> 8);
    _wire.write(address);
    _wire.endTransmission(false); //restart

    _wire.requestFrom(_i2caddr, 2);
    while (!_wire.available());
    uint16_t data = _wire.read();
    data <<= 8;
    while (!_wire.available());
    data |= _wire.read();
    _wire.endTransmission(true); //end
    return data;
}

// write 1 byte to FDC
void FDC2214::write8FDC(uint16_t address, uint8_t data) {
    _wire.beginTransmission(_i2caddr);
    _wire.write(address >> 8);
    _wire.write(address);
    _wire.write(data);
    _wire.endTransmission();
}

// write 2 bytes to FDC  
void FDC2214::write16FDC(uint16_t address, uint16_t data) {
    _wire.beginTransmission(_i2caddr);
    _wire.write(address & 0xFF);
    _wire.write(data >> 8);
    _wire.write(data);
    _wire.endTransmission();
}
