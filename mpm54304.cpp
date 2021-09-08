/*
 * Arduino library for Monolithic Power Systems MPM54304  
 * https://www.monolithicpower.com/en/mpm54304.html
 * 
 * Copyright (c) Graham Sutherland 2021
 * Released under MIT license.
 * 
 */

#include "Arduino.h"
#include <Wire.h>
#include "mpm54304.h"


MPM54304::MPM54304(uint8_t addr)
{
  // check if the address is a valid one for MPM54304. if not, set the default.
  if (IS_MPM54304_I2C_ADDRESS_VALID(addr))
    this->_address = addr;
  else
    this->_address = MPM54304_DEFAULT_ADDRESS;

  // zero out the config structs
  this->_systemConfig = { 0 };
  for (size_t i = 0; i < 4; i++)
  {
    this->_buckConfig[i] = { 0 };
  }
}


bool MPM54304::begin()
{
  // todo
}


bool MPM54304::update(bool forceWrite)
{
  if (forceWrite || this->_dirty)
  {
    Serial.println("update() : writing config");
    if (!this->writeAllConfig())
      return false;
    this->_dirty = false;
  }
  
  if (!this->readSystemConfig())
    return false;
  
  for (int i = 0; i < 4; i++)
  {
    if (!this->readBuckConfig(i))
      return false;
  }
}


MPM54304_BUCK_CONFIG* MPM54304::getBuckConfigUnsafe(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  
  return &(this->_buckConfig[buck]);
}


MPM54304_SYSTEM_CONFIG* MPM54304::getSystemConfigUnsafe()
{
  return &(this->_systemConfig);
}


void MPM54304::printConfig()
{
  const char* MPM54304SlewRate_names[] = { "MPM54304_SLEW_RATE_2670uV", "MPM54304_SLEW_RATE_1600uV", "MPM54304_SLEW_RATE_1000uV", "MPM54304_SLEW_RATE_670uV", "MPM54304_SLEW_RATE_400uV", "MPM54304_SLEW_RATE_250uV", "MPM54304_SLEW_RATE_167uV", "MPM54304_SLEW_RATE_100uV" };
  const char* MPM54304SwitchingMode_names[] = { "MPM54304_MODE_AUTO_PFM_PWM", "MPM54304_MODE_FORCED_PWM" };
  const char* MPM54304CurrentLimit_names[] = { "MPM54304_CURRENT_LIMIT_2A_VALLEY_1A_OUTPUT", "MPM54304_CURRENT_LIMIT_3A_VALLEY_2A_OUTPUT", "MPM54304_CURRENT_LIMIT_4A_VALLEY_3A_OUTPUT", "MPM54304_CURRENT_LIMIT_5A_VALLEY_5A_OUTPUT" };
  const char* MPM54304PhaseDelay_names[] = { "MPM54304_PHASE_DELAY_0", "MPM54304_PHASE_DELAY_90", "MPM54304_PHASE_DELAY_180", "MPM54304_PHASE_DELAY_270" };
  const char* MPM54304FeedbackRatio_names[] = { "MPM54304_FEEDBACK_DIRECT", "MPM54304_FEEDBACK_ONE_THIRD" };
  const char* MPM54304UnderVoltageThreshold_names[] = { "MPM54304_UVLO_THRESHOLD_3500mV", "MPM54304_UVLO_THRESHOLD_4500mV", "MPM54304_UVLO_THRESHOLD_5800mV", "MPM54304_UVLO_THRESHOLD_8500mV" };
  const char* MPM54304OutputPortSyncMode_names[] = { "MPM54304_OP_SYNCOUT_PULL_LOW", "MPM54304_OP_SYNCOUT_OPEN_DRAIN" };
  const char* MPM54304SwitchingFrequency_names[] = { "MPM54304_SWITCHING_533KHz", "MPM54304_SWITCHING_800KHz", "MPM54304_SWITCHING_1060KHz", "MPM54304_SWITCHING_1600KHz" };
  const char* MPM54304OutputPinFunction_names[] = { "MPM54304_OUTPUT_PIN_ADDRESS", "MPM54304_OUTPUT_PIN_POWER_GOOD", "MPM54304_OUTPUT_PIN_GPIO", "MPM54304_OUTPUT_PIN_SYNC" };
  const char* MPM54304PowerGoodDelay_names[] = { "MPM54304_PG_DELAY_200us", "MPM54304_PG_DELAY_5ms", "MPM54304_PG_DELAY_25ms", "MPM54304_PG_DELAY_75ms", "MPM54304_PG_DELAY_200ms" };
  
  Serial.println("");
  Serial.print("Configuration for MPM54304 device at address 0x");
  Serial.println(this->_address, HEX);
  Serial.println("");

  MPM54304_SYSTEM_CONFIG* syscfg = &this->_systemConfig;
  Serial.println("SYSTEM CONFIG:");
  
  Serial.print("MPM54304_SYSTEM_CONFIG.Reg0x0C = 0x"); Serial.print(syscfg->Reg0x0C, HEX); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.EnableBuck1 = "); Serial.println(syscfg->EnableBuck1 ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.EnableBuck2 = "); Serial.println(syscfg->EnableBuck2 ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.EnableBuck3 = "); Serial.println(syscfg->EnableBuck3 ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.EnableBuck4 = "); Serial.println(syscfg->EnableBuck4 ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.UnderVoltageThreshold = "); Serial.print(MPM54304UnderVoltageThreshold_names[syscfg->UnderVoltageThreshold]); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.OutputPortSyncMode = "); Serial.print(MPM54304OutputPortSyncMode_names[syscfg->OutputPortSyncMode]); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.Reg0x0D = 0x"); Serial.print(syscfg->Reg0x0D, HEX); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.SwitchingFrequency = "); Serial.print(MPM54304SwitchingFrequency_names[syscfg->SwitchingFrequency]); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.ShutdownDelayEnable = "); Serial.println(syscfg->ShutdownDelayEnable ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.I2CAddress = 0x"); Serial.print(syscfg->I2CAddress, HEX); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.Reg0x0E = 0x"); Serial.print(syscfg->Reg0x0E, HEX); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.OutputPinFunction = "); Serial.print(MPM54304OutputPinFunction_names[syscfg->OutputPinFunction]); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.MTPProgram = "); Serial.println(syscfg->MTPProgram ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.PowerGoodDelay = "); Serial.print(MPM54304PowerGoodDelay_names[syscfg->PowerGoodDelay]); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.ParallelBuck34 = "); Serial.println(syscfg->ParallelBuck34 ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.ParallelBuck12 = "); Serial.println(syscfg->ParallelBuck12 ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.Reg0x0F = 0x"); Serial.print(syscfg->Reg0x0F, HEX); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.MTPConfigureCode = 0x"); Serial.print(syscfg->MTPConfigureCode, HEX); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.Reg0x10 = 0x"); Serial.print(syscfg->Reg0x10, HEX); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.MTPRevisionNumber = 0x"); Serial.print(syscfg->MTPRevisionNumber, HEX); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.Reg0x11 = 0x"); Serial.print(syscfg->Reg0x11, HEX); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.MTPProgramPassword = 0x"); Serial.print(syscfg->MTPProgramPassword, HEX); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.Reg0x12 = 0x"); Serial.print(syscfg->Reg0x12, HEX); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.PowerGoodBuck1 = "); Serial.println(syscfg->PowerGoodBuck1 ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.PowerGoodBuck2 = "); Serial.println(syscfg->PowerGoodBuck2 ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.PowerGoodBuck3 = "); Serial.println(syscfg->PowerGoodBuck3 ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.PowerGoodBuck4 = "); Serial.println(syscfg->PowerGoodBuck4 ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.OverTemperatureWarning = "); Serial.println(syscfg->OverTemperatureWarning ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.OverTemperatureProtection = "); Serial.println(syscfg->OverTemperatureProtection ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.Reg0x13 = 0x"); Serial.print(syscfg->Reg0x13, HEX); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.VendorID = 0x"); Serial.print(syscfg->VendorID, HEX); Serial.println("");
  Serial.print("MPM54304_SYSTEM_CONFIG.ChecksumFlag = "); Serial.println(syscfg->ChecksumFlag ? "true" : "false");
  Serial.print("MPM54304_SYSTEM_CONFIG.CurrentMTPPageIndex = 0x"); Serial.print(syscfg->CurrentMTPPageIndex, HEX); Serial.println("");
  Serial.println("");

  for (int i = 0; i < 4; i++)
  {
    MPM54304_BUCK_CONFIG* buckcfg = &this->_buckConfig[i];
    Serial.print("BUCK ");
    Serial.print(i+1);
    Serial.println(" CONFIG:");
    Serial.print("MPM54304_BUCK_CONFIG.Byte0 = 0x"); Serial.print(buckcfg->Byte0, HEX); Serial.println("");
    Serial.print("MPM54304_BUCK_CONFIG.SoftStartDelayMillis = 0x"); Serial.print(buckcfg->SoftStartDelayMillis, HEX); Serial.println("");
    Serial.print("MPM54304_BUCK_CONFIG.AdditionalPhaseDelay = "); Serial.println(buckcfg->AdditionalPhaseDelay ? "true" : "false");
    Serial.print("MPM54304_BUCK_CONFIG.SoftStartTime = "); Serial.print(MPM54304SlewRate_names[buckcfg->SoftStartTime]); Serial.println("");
    Serial.print("MPM54304_BUCK_CONFIG.Byte1 = 0x"); Serial.print(buckcfg->Byte1, HEX); Serial.println("");
    Serial.print("MPM54304_BUCK_CONFIG.VoltageOutLimitEnable = "); Serial.println(buckcfg->VoltageOutLimitEnable ? "true" : "false");
    Serial.print("MPM54304_BUCK_CONFIG.SwitchingMode = "); Serial.print(MPM54304SwitchingMode_names[buckcfg->SwitchingMode]); Serial.println("");
    Serial.print("MPM54304_BUCK_CONFIG.CurrentLimit = "); Serial.print(MPM54304CurrentLimit_names[buckcfg->CurrentLimit]); Serial.println("");
    Serial.print("MPM54304_BUCK_CONFIG.OverVoltageProtectionEnable = "); Serial.println(buckcfg->OverVoltageProtectionEnable ? "true" : "false");
    Serial.print("MPM54304_BUCK_CONFIG.PhaseDelay = "); Serial.print(MPM54304PhaseDelay_names[buckcfg->PhaseDelay]); Serial.println("");
    Serial.print("MPM54304_BUCK_CONFIG.VoltageOutDischargeEnable = "); Serial.println(buckcfg->VoltageOutDischargeEnable ? "true" : "false");
    Serial.print("MPM54304_BUCK_CONFIG.Byte2 = 0x"); Serial.print(buckcfg->Byte2, HEX); Serial.println("");
    Serial.print("MPM54304_BUCK_CONFIG.FeedbackRatio = "); Serial.print(MPM54304FeedbackRatio_names[buckcfg->FeedbackRatio]); Serial.println("");
    Serial.print("MPM54304_BUCK_CONFIG.VoltageReference = 0x"); Serial.print(buckcfg->VoltageReference, HEX); Serial.println("");
    Serial.println("");
  }
}


MPM54304I2CStatus MPM54304::getLastI2CStatus()
{
  return this->_lastI2CStatus;
}


/*
 * System config
 */


void MPM54304::enableBuck(uint8_t buck)
{
  this->setBuckEnabled(buck, true);
}


void MPM54304::disableBuck(uint8_t buck)
{
  this->setBuckEnabled(buck, false);
}


bool MPM54304::getBuckEnabled(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);

  switch (buck)
  {
    case MPM54304_BUCK1:
      return this->_systemConfig.EnableBuck1;
    case MPM54304_BUCK2:
      return this->_systemConfig.EnableBuck2;
    case MPM54304_BUCK3:
      return this->_systemConfig.EnableBuck3;
    case MPM54304_BUCK4:
      return this->_systemConfig.EnableBuck4;
    default:
      assert(false); // should never reach here.
  }
}


void MPM54304::setBuckEnabled(uint8_t buck, bool enable)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  MPM54304_SET_DIRTY;

  switch (buck)
  {
    case MPM54304_BUCK1:
      this->_systemConfig.EnableBuck1 = enable;
      break;
    case MPM54304_BUCK2:
      this->_systemConfig.EnableBuck2 = enable;
      break;
    case MPM54304_BUCK3:
      this->_systemConfig.EnableBuck3 = enable;
      break;
    case MPM54304_BUCK4:
      this->_systemConfig.EnableBuck4 = enable;
      break;
    default:
      assert(false); // should never reach here.
  }
}


// returns a MPM54304UnderVoltageThreshold value representing the currently configured undervoltage threshold (UVLO)
MPM54304UnderVoltageThreshold MPM54304::getUnderVoltageThreshold()
{
  return this->_systemConfig.UnderVoltageThreshold;
}


// sets the undervoltage threshold (UVLO) of the device, using a MPM54304UnderVoltageThreshold value
void MPM54304::setUnderVoltageThreshold(MPM54304UnderVoltageThreshold threshold)
{
  MPM54304_SET_DIRTY;
  this->_systemConfig.UnderVoltageThreshold = threshold;
}


// returns a MPM54304OutputPortSyncMode value representing the current output port sync out mode.
MPM54304OutputPortSyncMode MPM54304::getOutputPortSyncMode()
{
  return this->_systemConfig.OutputPortSyncMode;
}


// sets the behaviour of the output port sync out mode, using a MPM54304OutputPortSyncMode value. this only has meaning when the OP pin is configured for sync out.
void MPM54304::setOutputPortSyncMode(MPM54304OutputPortSyncMode syncMode)
{
  MPM54304_SET_DIRTY;
  this->_systemConfig.OutputPortSyncMode = syncMode;
}


// returns a MPM54304SwitchingFrequency value representing the currently configured switching frequency
MPM54304SwitchingFrequency MPM54304::getSwitchingFrequency()
{
  return this->_systemConfig.SwitchingFrequency;
}


// sets the switching frequency for the buck converters, using a MPM54304SwitchingFrequency value
void MPM54304::setSwitchingFrequency(MPM54304SwitchingFrequency freq)
{
  MPM54304_SET_DIRTY;
  this->_systemConfig.SwitchingFrequency = freq;
}


// returns true if the shutdown delay function is enabled, otherwise false.
bool MPM54304::getShutdownDelay()
{
  return this->_systemConfig.ShutdownDelayEnable;
}


// enables or disables the shutdown delay function. when disabled, all bucks turn off at the same time. when enabled, the shutdown is sequenced.
void MPM54304::setShutdownDelay(bool enable)
{
  MPM54304_SET_DIRTY;
  this->_systemConfig.ShutdownDelayEnable = enable;
}


// returns the I2C address that is currently in use.
uint8_t MPM54304::getCurrentI2CAddress()
{
  // return the address that we're currently using
  return this->_address;
}


// returns true if the I2C address has been changed by setI2CAddress() but we haven't yet updated the device, otherwise false.
bool MPM54304::isI2CAddressChangePending()
{
  return this->_addressChangePending;
}


// returns the I2C address that's in the system config block.
// if setI2CAddress() has been called, this will contain the new I2C address that'll be used after a call to update(), rather than the current one.
uint8_t MPM54304::getConfiguredI2CAddress()
{
  return this->_systemConfig.I2CAddress & 0x60;
}


// sets the I2C address that the device should use after the next update
bool MPM54304::setI2CAddress(uint8_t addr)
{
  ASSERT_MPM54304_I2C_ADDRESS_VALID(addr);
  MPM54304_SET_DIRTY;
  
  this->_addressChangePending = true;
  this->_systemConfig.I2CAddress = addr & 0b11111; // only bottom 5 address bits stored (0b11xxxxx)
}


// gets a MPM54304OutputPinFunction value representing the currently configured output pin (OP) function
MPM54304OutputPinFunction MPM54304::getOutputPinFunction()
{
  return this->_systemConfig.OutputPinFunction;
}


// configures the function of the device's output pin (OP). the pin can be configured for I2C address, power good (PG) out, GPIO, or sync out.
void MPM54304::setOutputPinFunction(MPM54304OutputPinFunction function)
{
  MPM54304_SET_DIRTY;
  this->_systemConfig.OutputPinFunction = function;
}


// returns a MPM54304PowerGoodDelay value representing the current power good (PG) delay
MPM54304PowerGoodDelay MPM54304::getPowerGoodDelay()
{
  return this->_systemConfig.PowerGoodDelay;
}


// sets the power good (PG) delay for the device, using a MPM54304PowerGoodDelay value
void MPM54304::setPowerGoodDelay(MPM54304PowerGoodDelay pgDelay)
{
  MPM54304_SET_DIRTY;
  this->_systemConfig.PowerGoodDelay = pgDelay;
}


// returns true if parallel mode is enabled on bucks 1 and 2, otherwise false
bool MPM54304::getParallelBuck12Enable()
{
  return this->_systemConfig.ParallelBuck12;
}


// enable or disable parallel mode on bucks 1 and 2
void MPM54304::setParallelBuck12Enable(bool enable)
{
  MPM54304_SET_DIRTY;
  this->_systemConfig.ParallelBuck12 = enable;
}


// returns true if parallel mode is enabled on bucks 3 and 4, otherwise false
bool MPM54304::getParallelBuck34Enable()
{
  return this->_systemConfig.ParallelBuck34;
}


// enable or disable parallel mode on bucks 3 and 4
void MPM54304::setParallelBuck34Enable(bool enable)
{
  MPM54304_SET_DIRTY;
  this->_systemConfig.ParallelBuck34 = enable;
}


// enable parallel mode on bucks 1 and 2
void MPM54304::enableParallelBuck12()
{
  this->setParallelBuck12Enable(true);
}


// enable parallel mode on bucks 3 and 4
void MPM54304::enableParallelBuck34()
{
  this->setParallelBuck34Enable(true);
}


// disable parallel mode on bucks 1 and 2
void MPM54304::disableParallelBuck12()
{
  this->setParallelBuck12Enable(false);
}


// disable parallel mode on bucks 3 and 4
void MPM54304::disableParallelBuck34()
{
  this->setParallelBuck34Enable(false);
}


uint8_t MPM54304::getMTPConfigureCode()
{
  return this->_systemConfig.MTPConfigureCode;
}


void MPM54304::setMTPConfigureCode(uint8_t code)
{
  MPM54304_SET_DIRTY;
  this->_systemConfig.MTPConfigureCode = code;
}


uint8_t MPM54304::getMTPRevisionNumber()
{
  return this->_systemConfig.MTPRevisionNumber;
}


void MPM54304::setMTPRevisionNumber(uint8_t revision)
{
  MPM54304_SET_DIRTY;
  this->_systemConfig.MTPRevisionNumber = revision;
}


bool MPM54304::getPowerGood(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);

  switch (buck)
  {
    case MPM54304_BUCK1:
      return this->_systemConfig.PowerGoodBuck1;
    case MPM54304_BUCK2:
      return this->_systemConfig.PowerGoodBuck2;
    case MPM54304_BUCK3:
      return this->_systemConfig.PowerGoodBuck3;
    case MPM54304_BUCK4:
      return this->_systemConfig.PowerGoodBuck4;
    default:
      assert(false); // should never reach here.
  }
}


bool MPM54304::getOverTemperatureWarning()
{
  return this->_systemConfig.OverTemperatureWarning;
}


bool MPM54304::getOverTemperatureProtection()
{
  return this->_systemConfig.OverTemperatureProtection;
}


uint8_t MPM54304::getVendorID()
{
  return this->_systemConfig.VendorID;
}


bool MPM54304::getChecksumFlag()
{
  return this->_systemConfig.ChecksumFlag;
}


uint8_t MPM54304::getCurrentMTPPageIndex()
{
  return this->_systemConfig.CurrentMTPPageIndex;
}


/*
 * Buck settings
 */


uint8_t MPM54304::getSoftStartDelayMillis(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  
  return this->_buckConfig[buck].SoftStartDelayMillis;
}


void MPM54304::setSoftStartDelayMillis(uint8_t buck, uint8_t startDelay)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  MPM54304_SET_DIRTY;
  
  this->_buckConfig[buck].SoftStartDelayMillis = startDelay;
}


bool MPM54304::getAdditionalPhaseDelay(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  
  return this->_buckConfig[buck].AdditionalPhaseDelay;
}


void MPM54304::setAdditionalPhaseDelay(uint8_t buck, bool phaseDelay)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  MPM54304_SET_DIRTY;
  
  this->_buckConfig[buck].AdditionalPhaseDelay = phaseDelay;
}


MPM54304SlewRate MPM54304::getSoftStartTime(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  
  return this->_buckConfig[buck].SoftStartTime;
}


void MPM54304::setSoftStartTime(uint8_t buck, MPM54304SlewRate slewRate)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  MPM54304_SET_DIRTY;
  
  this->_buckConfig[buck].SoftStartTime = slewRate;
}


bool MPM54304::getVoltageOutLimitEnable(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  
  return this->_buckConfig[buck].VoltageOutLimitEnable;
}


void MPM54304::setVoltageOutLimitEnable(uint8_t buck, bool limitEnable)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  MPM54304_SET_DIRTY;
  
  this->_buckConfig[buck].VoltageOutLimitEnable = limitEnable;
}


MPM54304SwitchingMode MPM54304::getSwitchingMode(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  
  return this->_buckConfig[buck].SwitchingMode;
}


void MPM54304::setSwitchingMode(uint8_t buck, MPM54304SwitchingMode switchingMode)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  MPM54304_SET_DIRTY;
  
  this->_buckConfig[buck].SwitchingMode = switchingMode;
}


MPM54304CurrentLimit MPM54304::getCurrentLimit(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  
  return this->_buckConfig[buck].CurrentLimit;
}


void MPM54304::setCurrentLimit(uint8_t buck, MPM54304CurrentLimit currentLimit)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  MPM54304_SET_DIRTY;
  
  this->_buckConfig[buck].CurrentLimit = currentLimit;
}


bool MPM54304::getOverVoltageProtectionEnable(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  
  return this->_buckConfig[buck].OverVoltageProtectionEnable;
}


void MPM54304::setOverVoltageProtectionEnable(uint8_t buck, bool enable)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  MPM54304_SET_DIRTY;
  
  this->_buckConfig[buck].OverVoltageProtectionEnable = enable;
}


MPM54304PhaseDelay MPM54304::getPhaseDelay(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  
  return this->_buckConfig[buck].PhaseDelay;
}


void MPM54304::setPhaseDelay(uint8_t buck, MPM54304PhaseDelay phaseDelay)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  MPM54304_SET_DIRTY;
  
  this->_buckConfig[buck].PhaseDelay = phaseDelay;
}


bool MPM54304::getVoltageOutDischargeEnable(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  
  return this->_buckConfig[buck].VoltageOutDischargeEnable;
}


void MPM54304::setVoltageOutDischargeEnable(uint8_t buck, bool enable)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  MPM54304_SET_DIRTY;
  
  this->_buckConfig[buck].VoltageOutDischargeEnable = enable;
}


MPM54304FeedbackRatio MPM54304::getFeedbackRatio(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  
  return this->_buckConfig[buck].FeedbackRatio;
}


void MPM54304::setFeedbackRatio(uint8_t buck, MPM54304FeedbackRatio feedbackRatio)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  MPM54304_SET_DIRTY;
  
  this->_buckConfig[buck].FeedbackRatio = feedbackRatio;
}


uint16_t MPM54304::getReferenceVoltage(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);
  
  return (this->_buckConfig[buck].VoltageReference * MPM54304_VREF_STEP_INT) + MPM54304_VREF_MIN_INT;
}


bool MPM54304::setReferenceVoltage(uint8_t buck, uint16_t millivolts)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);

  if ((millivolts % MPM54304_VREF_STEP_INT) != 0)
  {
    millivolts -= millivolts % MPM54304_VREF_STEP_INT;
    millivolts += MPM54304_VREF_STEP_INT;
  }

  if ((millivolts < MPM54304_VREF_MIN_INT) || (millivolts > MPM54304_VREF_MAX_INT))
    return false;
  
  MPM54304_SET_DIRTY;
  
  this->_buckConfig[buck].VoltageReference = (millivolts - MPM54304_VREF_MIN_INT) / MPM54304_VREF_STEP_INT;
}


float MPM54304::getOutputVoltage(uint8_t buck)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);

  float voltage = MPM54304_VREF_MIN_FP + (this->_buckConfig[buck].VoltageReference * MPM54304_VREF_STEP_FP);
  if (this->_buckConfig[buck].FeedbackRatio == MPM54304_FEEDBACK_ONE_THIRD)
    voltage *= 3;

  return voltage / 1000.0f;
}


uint16_t MPM54304::setOutputVoltage(uint8_t buck, float volts)
{
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);

  // the minimum voltage that can be set is 550mV (minimum Vref)
  // the maximum voltage that can be set is 3x1830mV (max Vref through 1/3 feed)
  // if the input value is is outside the range by 10mV or more, reject it.
  if (volts <= (MPM54304_VOUT_MIN_FP - 0.01) || volts >= (MPM54304_VOUT_MAX_FP + 0.01))
  {
    return 0;
  }

  // clamp to acceptable range
  volts = constrain(volts, MPM54304_VOUT_MIN_FP, MPM54304_VOUT_MAX_FP);
  
  // convert this to a millivolt value, rounding up to the nearest 10mV
  uint16_t millivolts = (uint16_t)ceil(volts * 100) * 10;
  
  // constrain the result to the the min/max, just in case ceil() does something strange
  millivolts = constrain(millivolts, MPM54304_VOUT_MIN_INT, MPM54304_VOUT_MAX_INT);
  
  // if we're under the maximum Vref, we can use direct feed.
  if (millivolts <= MPM54304_VREF_MAX_INT)
  {
    MPM54304_SET_DIRTY;
    this->_buckConfig[buck].VoltageReference = (millivolts - MPM54304_VOUT_MIN_INT) / MPM54304_VREF_STEP_INT;
    this->_buckConfig[buck].FeedbackRatio = MPM54304_FEEDBACK_DIRECT;
    return millivolts;
  }
  
  // voltage is greater than the maximum Vref, so we need 1/3 feed.
  
  // re-calculate the output voltage as 1/3 the output voltage
  float vref_f = volts / 3.0;
  uint16_t vref = (uint16_t)ceil(vref_f * (1000 / MPM54304_VREF_STEP_INT)) * MPM54304_VREF_STEP_INT;
  vref = constrain(vref, MPM54304_VOUT_MIN_INT, MPM54304_VOUT_MAX_INT);

  // update registers
  MPM54304_SET_DIRTY;
  this->_buckConfig[buck].VoltageReference = (vref - MPM54304_VOUT_MIN_INT) / MPM54304_VREF_STEP_INT;
  this->_buckConfig[buck].FeedbackRatio = MPM54304_FEEDBACK_ONE_THIRD;
  return vref;
}


void MPM54304::setClock100KHz()
{
  Wire.setClock(100000);
}


void MPM54304::setClock400KHz()
{
  Wire.setClock(400000);
}


void MPM54304::setClock1000KHz()
{
  Wire.setClock(1000000); // !! may not be supported !!
}


void MPM54304::setClock3400KHz()
{
  Wire.setClock(3400000); // !! may not be supported !!
}


bool MPM54304::validateAddress(uint8_t addr)
{
  return IS_MPM54304_I2C_ADDRESS_VALID(addr);
}


bool MPM54304::readRegister(uint8_t reg, uint8_t* value)
{
  // assert that the currently configured address is ok
  ASSERT_MPM54304_I2C_ADDRESS_VALID(this->_address);
  // assert that the given register address is in bounds (0x00-0x13, 0x40-0x52)
  ASSERT_MP54304_REGISTER_VALID(reg);
  // assert that return pointer is non-null
  assert(value);

  // start register read
  Wire.beginTransmission(this->_address);

  // write register address
  if (Wire.write(reg) != 1)
  {
    this->_lastI2CStatus = MPM54034_I2C_STATUS_WRITE_LENGTH_MISMATCH;
    Wire.endTransmission();
    return false;
  }

  // end transmission and get I2C bus status
  MPM54304I2CStatus status = (MPM54304I2CStatus)Wire.endTransmission();
  this->_lastI2CStatus = status;
  if (status != MPM54034_I2C_STATUS_OK)
  {
    return false;
  }

  Wire.requestFrom((int)this->_address, 1);

  if (Wire.available() < 1)
  {
    this->_lastI2CStatus = MPM54034_I2C_STATUS_INSUFFICIENT_DATA;
    return false;
  }

  // read result back
  *value = Wire.read();
  
  return true;
}


bool MPM54304::writeRegister(uint8_t reg, uint8_t value)
{
  // check that the currently configured address is ok
  ASSERT_MPM54304_I2C_ADDRESS_VALID(this->_address);
  // check that register address is in bounds (0x00-0x13, 0x40-0x52)
  ASSERT_MP54304_REGISTER_VALID(reg);

  // start register read
  Wire.beginTransmission(this->_address);

  // write register address and data
  uint8_t data[2];
  data[0] = reg;
  data[1] = value;
  if (Wire.write(data, 2) != 2)
  {
    this->_lastI2CStatus = MPM54034_I2C_STATUS_WRITE_LENGTH_MISMATCH;
    Wire.endTransmission();
    return false;
  }

  // end transmission and get I2C bus status
  MPM54304I2CStatus status = (MPM54304I2CStatus)Wire.endTransmission();
  this->_lastI2CStatus = status;
  
  return status == MPM54034_I2C_STATUS_OK;
}


bool MPM54304::writeRegisters(uint8_t base, uint8_t* values, size_t bytes)
{
  // check that the currently configured address is ok
  ASSERT_MPM54304_I2C_ADDRESS_VALID(this->_address);
  // check that register address is in bounds (0x00-0x13, 0x40-0x52)
  ASSERT_MP54304_REGISTER_VALID(base);
  // check that the byte count fits in the I2C buffer (32 bytes)
  assert(bytes <= 31);

  // start register read
  Wire.beginTransmission(this->_address);

  // create buffer and prefix it with the base register address
  uint8_t buffer[32];
  buffer[0] = base;
  // copy the bytes into the buffer in reverse bit order
  for (size_t i = 0; i < bytes; i++)
  {
    buffer[i+1] = values[i];
  }
  
  // write register address and data
  for (size_t i = 0; i < bytes + 1; i++)
  {
    if (Wire.write(buffer[i]) != 1)
    {
      this->_lastI2CStatus = MPM54034_I2C_STATUS_WRITE_LENGTH_MISMATCH;
      Wire.endTransmission();
      return false;
    }
  }

  // end transmission and get I2C bus status
  MPM54304I2CStatus status = (MPM54304I2CStatus)Wire.endTransmission();
  this->_lastI2CStatus = status;
  
  return status == MPM54034_I2C_STATUS_OK;
}


bool MPM54304::readSystemConfig()
{
  uint8_t systemConfigData[MPM54304_SYSTEM_CONFIG_SIZE];

  // read the system config data into an array, returning false if any of the register reads fail.
  for (size_t i = 0; i < MPM54304_SYSTEM_CONFIG_SIZE; i++)
  {
    if (!this->readRegister(MPM54304_SYSTEM_CONFIG_REG_BASE + (uint8_t)i, systemConfigData + i))
      return false;
  }

  // after a successful read, copy the results.
  memcpy(this->_systemConfig.RawData, systemConfigData, MPM54304_SYSTEM_CONFIG_SIZE);

  return true;
}


bool MPM54304::writeSystemConfig()
{
  bool success = this->writeRegisters(MPM54304_SYSTEM_CONFIG_REG_BASE, this->_systemConfig.RawData, MPM54304_SYSTEM_CONFIG_SIZE);
  if (!success)
    return false;

  // update the I2C address if we changed it
  if (this->_addressChangePending)
  {
    this->_address = this->_systemConfig.I2CAddress & 0x60;
  }
  return true;
}


uint8_t MPM54304::getRegisterBaseForBuck(uint8_t buck)
{
  // validate buck
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);

  return MPM54304_BUCK_CONFIG_REG_BASE + (MPM54304_BUCK_CONFIG_SIZE * buck);
}


bool MPM54304::readBuckConfig(uint8_t buck)
{
  // validate buck
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);

  // get the base register for this buck converter
  uint8_t buckRegisterBase = getRegisterBaseForBuck(buck);

  // read the buck config data into an array, returning false if any of the register reads fail.
  uint8_t buckConfigData[MPM54304_BUCK_CONFIG_SIZE];
  for (size_t i = 0; i < MPM54304_BUCK_CONFIG_SIZE; i++)
  {
    if (!this->readRegister(buckRegisterBase + (uint8_t)i, buckConfigData + i))
      return false;
  }

  // after a successful read, copy the results.
  memcpy(this->_buckConfig[buck].RawData, buckConfigData, MPM54304_BUCK_CONFIG_SIZE);

  return true;
}


bool MPM54304::writeBuckConfig(uint8_t buck)
{
  // validate buck index
  ASSERT_MPM54034_BUCK_INDEX_VALID(buck);

  // get the base register for this buck converter
  uint8_t buckRegisterBase = getRegisterBaseForBuck(buck);

  // write to the device
  return this->writeRegisters(buckRegisterBase, this->_buckConfig[buck].RawData, MPM54304_BUCK_CONFIG_SIZE);
}


bool MPM54304::writeAllConfig()
{
  // set up a buffer that holds the whole config data
  // note: we use MPM54304_SYSTEM_CONFIG_WRITE_SIZE rather than MPM54304_SYSTEM_CONFIG_SIZE because the last two registers are read-only.
  const size_t CONFIG_DATA_SIZE = (MPM54304_BUCK_CONFIG_SIZE * 4) + MPM54304_SYSTEM_CONFIG_WRITE_SIZE;
  uint8_t configData[CONFIG_DATA_SIZE];

  // copy the buck register configs in first (0x00-0x0B)
  size_t ofs = 0;
  for (size_t i = 0; i < 4; i++)
  {
    memcpy(configData + ofs, this->_buckConfig[i].RawData, MPM54304_BUCK_CONFIG_SIZE);
    ofs += MPM54304_BUCK_CONFIG_SIZE;
  }
  
  // copy the system config in afterwards (0x0C - 0x10)
  memcpy(configData + ofs, this->_systemConfig.RawData, MPM54304_SYSTEM_CONFIG_WRITE_SIZE);

  // write to the device
  bool success = this->writeRegisters(0, configData, CONFIG_DATA_SIZE);
  if (!success)
    return false;

  // update the I2C address if we changed it
  if (this->_addressChangePending)
  {
    this->_address = this->_systemConfig.I2CAddress & 0x60;
  }
  return true;
}
