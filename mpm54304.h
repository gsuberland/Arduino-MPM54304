/*
 * Arduino library for Monolithic Power Systems MPM54304  
 * https://www.monolithicpower.com/en/mpm54304.html
 * 
 * Copyright (c) Graham Sutherland 2021
 * Released under MIT license.
 * 
 */

#ifndef _LIB_MPM54304_
#define _LIB_MPM54304_

#include "Arduino.h"
#include <Wire.h>

#define MPM54304_DEFAULT_ADDRESS (0x68)

#ifndef assert
#define _MPM54304_ASSERT_
#define assert(EX) { if (!(EX)) { Serial.print("ASSERTION FAILURE: "); Serial.print(__FILE__); Serial.print(" line "); Serial.println(__LINE__); Serial.println(#EX); while(1) {  } } }
#endif

#define MPM54304_BUCK1 (0)
#define MPM54304_BUCK2 (1)
#define MPM54304_BUCK3 (2)
#define MPM54304_BUCK4 (3)

#define MPM54304_SYSTEM_CONFIG_SIZE (8)
#define MPM54304_SYSTEM_CONFIG_WRITE_SIZE (6)
#define MPM54304_SYSTEM_CONFIG_REG_BASE (0x0C)

#define MPM54304_BUCK_CONFIG_SIZE (3)
#define MPM54304_BUCK_CONFIG_REG_BASE (0x00)

#define MPM54304_VREF_STEP_INT (10)
#define MPM54304_VREF_STEP_FP (0.01)
#define MPM54304_VREF_MIN_INT (550)
#define MPM54304_VREF_MIN_FP (0.55)
#define MPM54304_VREF_MAX_INT (1820)
#define MPM54304_VREF_MAX_FP (1.82)
#define MPM54304_VOUT_MIN_INT MPM54304_VREF_MIN_INT
#define MPM54304_VOUT_MIN_FP MPM54304_VREF_MIN_FP
#define MPM54304_VOUT_MAX_INT (MPM54304_VREF_MAX_INT*3)
#define MPM54304_VOUT_MAX_FP (MPM54304_VREF_MAX_FP*3.0)

// is a given I2C address valid for the MPM54304?
#define IS_MPM54304_I2C_ADDRESS_VALID(addr) ((((addr) & 0x60) == 0x60) && (((addr) & 0x80) == 0))
// assert that a given I2C address valid for the MPM54304 is valid
#define ASSERT_MPM54304_I2C_ADDRESS_VALID(addr) assert(IS_MPM54304_I2C_ADDRESS_VALID((addr)))

// is a given register offset valid for the MPM54304?
#define IS_MP54304_REGISTER_VALID(reg) (((reg) <= 0x13) || (((reg) >= 0x40) && ((reg) <= 0x52)))
// assert that a given register address is in bounds (0x00-0x13, 0x40-0x52)
#define ASSERT_MP54304_REGISTER_VALID(reg) assert(IS_MP54304_REGISTER_VALID((reg)))

// is a given buck regulator index valid for the MPM54304?
#define IS_MPM54034_BUCK_INDEX_VALID(idx) ((idx) < 4)
// assert that a given buck regulator index valid for the MPM54304
#define ASSERT_MPM54034_BUCK_INDEX_VALID(idx) assert(IS_MPM54034_BUCK_INDEX_VALID((idx)))

// set the dirty bit
#define MPM54304_SET_DIRTY { this->_dirty = true; }

enum MPM54304I2CStatus : uint16_t
{
  MPM54034_I2C_STATUS_OK = 0,
  MPM54034_I2C_STATUS_DATA_TOO_LONG = 1,
  MPM54034_I2C_STATUS_NACK_ON_ADDR = 2,
  MPM54034_I2C_STATUS_NACK_ON_DATA = 3,
  MPM54034_I2C_STATUS_ERROR = 4,
  MPM54034_I2C_STATUS_WRITE_LENGTH_MISMATCH = 0x8000,
  MPM54034_I2C_STATUS_INSUFFICIENT_DATA = 0x8001,
};

// soft start slew rate in uV/us
enum MPM54304SlewRate : uint8_t
{
  MPM54304_SLEW_RATE_2670uV = 0b000,
  MPM54304_SLEW_RATE_1600uV = 0b001,
  MPM54304_SLEW_RATE_1000uV = 0b010,
  MPM54304_SLEW_RATE_670uV = 0b011,
  MPM54304_SLEW_RATE_400uV = 0b100,
  MPM54304_SLEW_RATE_250uV = 0b101,
  MPM54304_SLEW_RATE_167uV = 0b110,
  MPM54304_SLEW_RATE_100uV = 0b111,
};

enum MPM54304SwitchingMode : uint8_t
{
  MPM54304_MODE_AUTO_PFM_PWM = 0,
  MPM54304_MODE_FORCED_PWM = 1,
};

enum MPM54304CurrentLimit : uint8_t
{
  MPM54304_CURRENT_LIMIT_2A_VALLEY_1A_OUTPUT = 0b00,
  MPM54304_CURRENT_LIMIT_3A_VALLEY_2A_OUTPUT = 0b01,
  MPM54304_CURRENT_LIMIT_4A_VALLEY_3A_OUTPUT = 0b10, // note: technically 4.2A valley but hard to put that in an enum
  MPM54304_CURRENT_LIMIT_5A_VALLEY_5A_OUTPUT = 0b11,
};

enum MPM54304PhaseDelay : uint8_t
{
  MPM54304_PHASE_DELAY_0 = 0b00,
  MPM54304_PHASE_DELAY_90 = 0b01,
  MPM54304_PHASE_DELAY_180 = 0b10,
  MPM54304_PHASE_DELAY_270 = 0b11,
};

enum MPM54304FeedbackRatio : uint8_t
{
  MPM54304_FEEDBACK_DIRECT = 0,
  MPM54304_FEEDBACK_ONE_THIRD = 1,
};

enum MPM54304UnderVoltageThreshold : uint8_t
{
  MPM54304_UVLO_THRESHOLD_3500mV = 0b00,
  MPM54304_UVLO_THRESHOLD_4500mV = 0b01,
  MPM54304_UVLO_THRESHOLD_5800mV = 0b10,
  MPM54304_UVLO_THRESHOLD_8500mV = 0b11,
};

enum MPM54304OutputPortSyncMode : uint8_t
{
  MPM54304_OP_SYNCOUT_PULL_LOW = 0,
  MPM54304_OP_SYNCOUT_OPEN_DRAIN = 1,
};

enum MPM54304SwitchingFrequency : uint8_t
{
  MPM54304_SWITCHING_533KHz = 0b00,
  MPM54304_SWITCHING_800KHz = 0b01,
  MPM54304_SWITCHING_1060KHz = 0b10,
  MPM54304_SWITCHING_1600KHz = 0b11,
};

enum MPM54304OutputPinFunction : uint8_t
{
  MPM54304_OUTPUT_PIN_ADDRESS = 0b00,
  MPM54304_OUTPUT_PIN_POWER_GOOD = 0b01,
  MPM54304_OUTPUT_PIN_GPIO = 0b10,
  MPM54304_OUTPUT_PIN_SYNC = 0b11,
};

enum MPM54304PowerGoodDelay : uint8_t
{
  MPM54304_PG_DELAY_200us  = 0b000,
  MPM54304_PG_DELAY_5ms    = 0b001,
  MPM54304_PG_DELAY_25ms   = 0b010,
  MPM54304_PG_DELAY_75ms   = 0b011,
  MPM54304_PG_DELAY_200ms  = 0b100,
};

#pragma pack(push, 1)

struct __attribute__((__packed__)) MPM54304_BUCK_CONFIG
{
  union
  {
    // just for convenience (you could technically cast the struct to this instead)
    uint8_t RawData[MPM54304_BUCK_CONFIG_SIZE];
    
    struct
    {
      // +0x00
      union
      {
        uint8_t Byte0;
        struct
        {
          uint8_t _padding : 2;
          uint8_t SoftStartDelayMillis : 2;
          bool AdditionalPhaseDelay : 1;
          MPM54304SlewRate SoftStartTime : 3;
        };
      };
      // +0x01
      union
      {
        uint8_t Byte1;
        struct
        {
          bool VoltageOutLimitEnable : 1; // Vout_Limit_ENx
          MPM54304SwitchingMode SwitchingMode : 1; // Mode
          MPM54304CurrentLimit CurrentLimit : 2;
          bool OverVoltageProtectionEnable : 1; // VOUT_OVP_ENx
          MPM54304PhaseDelay PhaseDelay : 2;
          bool VoltageOutDischargeEnable : 1; // VOUT_DIS_ENx
        };
      };
      // +0x02
      union
      {
        uint8_t Byte2;
        struct
        {
          MPM54304FeedbackRatio FeedbackRatio : 1; // Vout_Selectx
          uint8_t VoltageReference : 7; // Vref = 550mV + (VoltageReference * 10mV)
        };
      };
    };
  };
};


struct __attribute__((packed)) MPM54304_SYSTEM_CONFIG
{
  union
  {
    uint8_t RawData[MPM54304_SYSTEM_CONFIG_SIZE];
    
    struct
    {
      union
      {
        uint8_t Reg0x0C;
        struct
        {
          bool EnableBuck1 : 1; // EN1
          bool EnableBuck2 : 1; // EN2
          bool EnableBuck3 : 1; // EN3
          bool EnableBuck4 : 1; // EN4
          bool _padding_0x4c_bit3 : 1;
          MPM54304UnderVoltageThreshold UnderVoltageThreshold : 2; // UVLO
          MPM54304OutputPortSyncMode OutputPortSyncMode : 1; // OP_BIT
        };
      };
    
      union
      {
        uint8_t Reg0x0D;
        struct
        {
          MPM54304SwitchingFrequency SwitchingFrequency : 2; // FREQ
          bool ShutdownDelayEnable : 1; // Shutdown_Delay_EN
          uint8_t I2CAddress : 5; // bits A5 to A1 of the I2C address can be set here. bits A7 and A6 are always 1.
        };
      };
    
      union
      {
        uint8_t Reg0x0E;
        struct
        {
          MPM54304OutputPinFunction OutputPinFunction : 2; // ADD_PG_OP_SYNCOUT
          bool MTPProgram : 1;
          MPM54304PowerGoodDelay PowerGoodDelay : 3; // PG_Delay
          bool ParallelBuck34 : 1; // Parallel_2
          bool ParallelBuck12 : 1; // Parallel_1
        };
      };
    
      union
      {
        uint8_t Reg0x0F;
        uint8_t MTPConfigureCode;
      };
    
      union
      {
        uint8_t Reg0x10;
        uint8_t MTPRevisionNumber;
      };
    
      // this currently does nothing since reading from this register is undefined, and we don't support MTP writes.
      union
      {
        uint8_t Reg0x11;
        uint8_t MTPProgramPassword;
      };
    
      union
      {
        uint8_t Reg0x12;
        struct
        {
          bool PowerGoodBuck1 : 1; // PG1
          bool PowerGoodBuck2 : 1; // PG2
          bool PowerGoodBuck3 : 1; // PG3
          bool PowerGoodBuck4 : 1; // PG4
          bool OverTemperatureWarning : 1; // OT Warning
          bool OverTemperatureProtection : 1; // OTP
          uint8_t _padding : 2;
        };
      };
    
      union
      {
        uint8_t Reg0x13;
        struct
        {
          uint8_t VendorID : 4;
          bool ChecksumFlag : 1;
          uint8_t CurrentMTPPageIndex : 3;
        };
      };
    };
  };
};

#pragma pack(pop)

class MPM54304
{
  
public:
  MPM54304(uint8_t addr = MPM54304_DEFAULT_ADDRESS);
  ~MPM54304();
  
  bool begin();
  bool update(bool forceWrite = false);
  
  MPM54304_BUCK_CONFIG* getBuckConfigUnsafe(uint8_t buck);
  MPM54304_SYSTEM_CONFIG* getSystemConfigUnsafe();

  // set clock speed. 100KHz is default, 400KHz is generally supported everywhere. 1MHz and 3.4MHz may be supported; check your processor docs.
  void setClock100KHz();
  void setClock400KHz();
  void setClock1000KHz();
  void setClock3400KHz();

  void printConfig();

  // system settings
  
  void enableBuck(uint8_t buck);
  void disableBuck(uint8_t buck);
  bool getBuckEnabled(uint8_t buck);
  void setBuckEnabled(uint8_t buck, bool enable);
  
  MPM54304UnderVoltageThreshold getUnderVoltageThreshold();
  void setUnderVoltageThreshold(MPM54304UnderVoltageThreshold threshold);
  
  MPM54304OutputPortSyncMode getOutputPortSyncMode();
  void setOutputPortSyncMode(MPM54304OutputPortSyncMode syncMode);
  
  MPM54304SwitchingFrequency getSwitchingFrequency();
  void setSwitchingFrequency(MPM54304SwitchingFrequency freq);
  
  bool getShutdownDelay();
  void setShutdownDelay(bool enable);
  
  uint8_t getCurrentI2CAddress();
  bool isI2CAddressChangePending();
  uint8_t getConfiguredI2CAddress();
  bool setI2CAddress(uint8_t addr);
  
  MPM54304OutputPinFunction getOutputPinFunction();
  void setOutputPinFunction(MPM54304OutputPinFunction function);
  
  MPM54304PowerGoodDelay getPowerGoodDelay();
  void setPowerGoodDelay(MPM54304PowerGoodDelay pgDelay);
  
  bool getParallelBuck12Enable();
  bool getParallelBuck34Enable();
  void enableParallelBuck12();
  void enableParallelBuck34();
  void disableParallelBuck12();
  void disableParallelBuck34();
  void setParallelBuck12Enable(bool enable);
  void setParallelBuck34Enable(bool enable);

  uint8_t getMTPConfigureCode();
  void setMTPConfigureCode(uint8_t code);

  uint8_t getMTPRevisionNumber();
  void setMTPRevisionNumber(uint8_t revision);

  bool getPowerGood(uint8_t buck);

  bool getOverTemperatureWarning();

  bool getOverTemperatureProtection();

  uint8_t getVendorID();

  bool getChecksumFlag();

  uint8_t getCurrentMTPPageIndex();

  // buck settings

  uint8_t getSoftStartDelayMillis(uint8_t buck);
  void setSoftStartDelayMillis(uint8_t buck, uint8_t startDelay);

  bool getAdditionalPhaseDelay(uint8_t buck);
  void setAdditionalPhaseDelay(uint8_t buck, bool phaseDelay);

  MPM54304SlewRate getSoftStartTime(uint8_t buck);
  void setSoftStartTime(uint8_t buck, MPM54304SlewRate slewRate);

  bool getVoltageOutLimitEnable(uint8_t buck);
  void setVoltageOutLimitEnable(uint8_t buck, bool limitEnable);

  MPM54304SwitchingMode getSwitchingMode(uint8_t buck);
  void setSwitchingMode(uint8_t buck, MPM54304SwitchingMode switchingMode);

  MPM54304CurrentLimit getCurrentLimit(uint8_t buck);
  void setCurrentLimit(uint8_t buck, MPM54304CurrentLimit currentLimit);

  bool getOverVoltageProtectionEnable(uint8_t buck);
  void setOverVoltageProtectionEnable(uint8_t buck, bool enable);

  MPM54304PhaseDelay getPhaseDelay(uint8_t buck);
  void setPhaseDelay(uint8_t buck, MPM54304PhaseDelay phaseDelay);

  bool getVoltageOutDischargeEnable(uint8_t buck);
  void setVoltageOutDischargeEnable(uint8_t buck, bool enable);

  MPM54304FeedbackRatio getFeedbackRatio(uint8_t buck);
  void setFeedbackRatio(uint8_t buck, MPM54304FeedbackRatio feedbackRatio);

  uint16_t getReferenceVoltage(uint8_t buck);
  // sets the reference to 550-1790mV, in steps of 10mV
  bool setReferenceVoltage(uint8_t buck, uint16_t millivolts);
  
  // sets the nearest available output voltage (rounding up) by a combination of reference voltage and feedback ratio. returns the resulting reference voltage in mV, or 0 if the value was out of range.
  uint16_t setOutputVoltage(uint8_t buck, float volts);
  // gets the output voltage that will result from the current settings.
  float getOutputVoltage(uint8_t buck);

  // reads the system configuration block
  bool readSystemConfig();
  // writes the system configuration block
  bool writeSystemConfig();

  // reads the configuration block for one buck converter
  bool readBuckConfig(uint8_t buck);
  // writes the configuration block for one buck converter
  bool writeBuckConfig(uint8_t buck);

  // writes all configuration registers (system + all four bucks) at once, instead of separately.
  bool writeAllConfig();

private:
  bool _dirty = false;
  uint8_t _address = MPM54304_DEFAULT_ADDRESS;
  bool _addressChangePending = false;
  
  MPM54304I2CStatus _lastI2CStatus;
  MPM54304_BUCK_CONFIG _buckConfig[4];
  MPM54304_SYSTEM_CONFIG _systemConfig;

  bool readRegister(uint8_t reg, uint8_t* value);
  bool writeRegister(uint8_t reg, uint8_t value);
  bool writeRegisters(uint8_t base, uint8_t* values, size_t count);
  
  uint8_t getRegisterBaseForBuck(uint8_t buck);

  bool validateAddress(uint8_t addr);
};

#endif
